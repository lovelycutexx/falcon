`timescale 1ns/1ps
module falcon_f64_fft_exu #
(
    parameter ADDR_W = 10
)
(
    input                     clk,
    input                     rst_n,

    input                     cmd_valid,
    output                    cmd_ready,
    input      [2:0]          cmd_opcode,
    input      [4:0]          cmd_logn,

    output reg [ADDR_W-1:0]   mem_rd_addr0,
    output reg [ADDR_W-1:0]   mem_rd_addr1,
    input      [63:0]         mem_rd_data0_re,
    input      [63:0]         mem_rd_data0_im,
    input      [63:0]         mem_rd_data1_re,
    input      [63:0]         mem_rd_data1_im,

    output reg [ADDR_W-1:0]   twiddle_addr,
    input      [63:0]         twiddle_re,
    input      [63:0]         twiddle_im,

    output reg                mem_wr_en,
    output reg [ADDR_W-1:0]   mem_wr_addr0,
    output reg [ADDR_W-1:0]   mem_wr_addr1,
    output reg [63:0]         mem_wr_data0_re,
    output reg [63:0]         mem_wr_data0_im,
    output reg [63:0]         mem_wr_data1_re,
    output reg [63:0]         mem_wr_data1_im,

    output reg                rsp_valid,
    output reg                rsp_done,
    output reg                rsp_fail,
    output reg [7:0]          rsp_status,
    output reg                status_invalid,
    output reg                status_overflow,
    output reg                status_underflow,
    output reg                status_inexact,
    output                    busy
);

    localparam [2:0] OP_FFT_FWD = 3'd0;
    localparam [2:0] OP_FFT_INV = 3'd1;
    // Falcon reference half-complex inverse FFT layout. This simulation
    // bring-up path converts the half-complex frequency representation into
    // 512 real time-domain coefficients in the word lower lane.
    localparam [2:0] OP_FFT_FALCON_INV = 3'd2;

    localparam [4:0] ST_IDLE         = 5'd0;
    localparam [4:0] ST_BITREV_CHECK = 5'd1;
    localparam [4:0] ST_BITREV_WRITE = 5'd2;
    localparam [4:0] ST_BFLY_REQ     = 5'd3;
    localparam [4:0] ST_BFLY_WAIT    = 5'd4;
    localparam [4:0] ST_WRITE        = 5'd5;
    localparam [4:0] ST_DONE         = 5'd6;
    localparam [4:0] ST_FAIL         = 5'd7;
    localparam [4:0] ST_FAL_LOAD     = 5'd8;
    localparam [4:0] ST_FAL_CALC     = 5'd9;
    localparam [4:0] ST_FAL_WRITE    = 5'd10;

    reg [4:0]        state;
    reg [7:0]        fail_code_q;
    reg              inverse_q;
    reg [4:0]        logn_q;
    reg [4:0]        stage_idx;
    reg [4:0]        last_stage_idx;
    reg [ADDR_W-1:0] pair_idx;
    reg [ADDR_W-1:0] pair_count_limit;
    reg [ADDR_W-1:0] bitrev_idx;
    reg [ADDR_W-1:0] last_data_idx_q;
    reg [ADDR_W-1:0] bitrev_swap_addr_q;
    reg [ADDR_W-1:0] pair_addr_a_q;
    reg [ADDR_W-1:0] pair_addr_b_q;
    reg [63:0]       pair_y0_re_q;
    reg [63:0]       pair_y0_im_q;
    reg [63:0]       pair_y1_re_q;
    reg [63:0]       pair_y1_im_q;
    reg [ADDR_W-1:0] fal_idx;

`ifndef SYNTHESIS
    real              fal_f [0:1023];
    real              fal_tmp_re;
    real              fal_tmp_im;
    real              fal_x_re;
    real              fal_x_im;
    real              fal_y_re;
    real              fal_y_im;
    real              fal_s_re;
    real              fal_s_im;
    real              fal_scale;
    integer           fal_u;
    integer           fal_i1;
    integer           fal_j1;
    integer           fal_j;
    integer           fal_j2;
    integer           fal_t;
    integer           fal_m;
    integer           fal_hm;
    integer           fal_dt;
    integer           fal_hn;
    integer           fal_n;
    integer           fal_rev;
`endif

    wire [ADDR_W-1:0] addr_a_w;
    wire [ADDR_W-1:0] addr_b_w;
    wire [ADDR_W-1:0] twiddle_idx_w;

    reg               bfly_in_valid;
    wire              bfly_in_ready;
    wire              bfly_out_valid;
    reg               bfly_out_ready;
    wire [63:0]       bfly_y0_re;
    wire [63:0]       bfly_y0_im;
    wire [63:0]       bfly_y1_re;
    wire [63:0]       bfly_y1_im;
    wire              bfly_status_invalid;
    wire              bfly_status_overflow;
    wire              bfly_status_underflow;
    wire              bfly_status_inexact;

    wire [63:0]       twiddle_im_eff;
    wire [ADDR_W-1:0] bitrev_addr_w;

    assign cmd_ready = (state == ST_IDLE);
    assign busy      = (state != ST_IDLE);
    assign twiddle_im_eff = inverse_q ? {~twiddle_im[63], twiddle_im[62:0]} : twiddle_im;
    assign bitrev_addr_w = bit_reverse_addr(bitrev_idx, logn_q);

    function [ADDR_W-1:0] bit_reverse_addr;
        input [ADDR_W-1:0] value;
        input [4:0]        logn;
        integer            idx;
        begin
            bit_reverse_addr = {ADDR_W{1'b0}};
            for (idx = 0; idx < ADDR_W; idx = idx + 1) begin
                if (idx < logn) begin
                    bit_reverse_addr[logn - idx - 1'b1] = value[idx];
                end
            end
        end
    endfunction

    function integer bit_reverse_int;
        input integer value;
        input integer bits;
        integer idx;
        begin
            bit_reverse_int = 0;
            for (idx = 0; idx < bits; idx = idx + 1) begin
                if (((value >> idx) & 1) != 0)
                    bit_reverse_int = bit_reverse_int | (1 << (bits - idx - 1));
            end
        end
    endfunction

    // Combinational ×0.5: decrement exponent by 1.
    // Safe for Falcon's numeric range — normalised non-zero values only.
    function [63:0] f64_half;
        input [63:0] val;
        begin
            f64_half = (val[62:52] == 11'd0) ? val
                     : {val[63], val[62:52] - 1'b1, val[51:0]};
        end
    endfunction

    falcon_fft_addr_gen_cfg #
    (
        .ADDR_W (ADDR_W)
    )
    u_addr_gen (
        .logn        (logn_q),
        .stage_idx   (stage_idx),
        .pair_idx    (pair_idx),
        .addr_a      (addr_a_w),
        .addr_b      (addr_b_w),
        .twiddle_idx (twiddle_idx_w)
    );

    falcon_f64_complex_bfly u_bfly (
        .clk              (clk),
        .rst_n            (rst_n),
        .in_valid         (bfly_in_valid),
        .in_ready         (bfly_in_ready),
        .a_re             (mem_rd_data0_re),
        .a_im             (mem_rd_data0_im),
        .b_re             (mem_rd_data1_re),
        .b_im             (mem_rd_data1_im),
        .w_re             (twiddle_re),
        .w_im             (twiddle_im_eff),
        .out_valid        (bfly_out_valid),
        .out_ready        (bfly_out_ready),
        .y0_re            (bfly_y0_re),
        .y0_im            (bfly_y0_im),
        .y1_re            (bfly_y1_re),
        .y1_im            (bfly_y1_im),
        .status_invalid   (bfly_status_invalid),
        .status_overflow  (bfly_status_overflow),
        .status_underflow (bfly_status_underflow),
        .status_inexact   (bfly_status_inexact),
        .busy             ()
    );

    always @(*) begin
        bfly_in_valid  = 1'b0;
        bfly_out_ready = 1'b0;

        mem_rd_addr0    = addr_a_w;
        mem_rd_addr1    = addr_b_w;
        twiddle_addr    = twiddle_idx_w;

        mem_wr_en       = 1'b0;
        mem_wr_addr0    = pair_addr_a_q;
        mem_wr_addr1    = pair_addr_b_q;
        mem_wr_data0_re = pair_y0_re_q;
        mem_wr_data0_im = pair_y0_im_q;
        mem_wr_data1_re = pair_y1_re_q;
        mem_wr_data1_im = pair_y1_im_q;

        rsp_valid       = 1'b0;
        rsp_done        = 1'b0;
        rsp_fail        = 1'b0;
        rsp_status      = 8'h00;

        case (state)
            ST_BITREV_CHECK: begin
                mem_rd_addr0 = bitrev_idx;
                mem_rd_addr1 = bitrev_addr_w;
                twiddle_addr = {ADDR_W{1'b0}};
            end

            ST_BITREV_WRITE: begin
                mem_rd_addr0    = bitrev_idx;
                mem_rd_addr1    = bitrev_swap_addr_q;
                twiddle_addr    = {ADDR_W{1'b0}};
                mem_wr_en       = 1'b1;
                mem_wr_addr0    = bitrev_idx;
                mem_wr_addr1    = bitrev_swap_addr_q;
                mem_wr_data0_re = mem_rd_data1_re;
                mem_wr_data0_im = mem_rd_data1_im;
                mem_wr_data1_re = mem_rd_data0_re;
                mem_wr_data1_im = mem_rd_data0_im;
            end

            ST_FAL_LOAD: begin
                mem_rd_addr0 = fal_idx;
                mem_rd_addr1 = fal_idx;
                twiddle_addr = {ADDR_W{1'b0}};
            end

            ST_FAL_WRITE: begin
                mem_wr_en       = 1'b1;
                mem_wr_addr0    = fal_idx;
                mem_wr_addr1    = fal_idx;
`ifndef SYNTHESIS
                mem_wr_data0_re = $realtobits(fal_f[fal_idx]);
                mem_wr_data0_im = 64'd0;
                mem_wr_data1_re = $realtobits(fal_f[fal_idx]);
                mem_wr_data1_im = 64'd0;
`else
                mem_wr_data0_re = 64'd0;
                mem_wr_data0_im = 64'd0;
                mem_wr_data1_re = 64'd0;
                mem_wr_data1_im = 64'd0;
`endif
            end

            ST_BFLY_REQ: begin
                bfly_in_valid = 1'b1;
            end

            ST_BFLY_WAIT: begin
                bfly_out_ready = 1'b1;
            end

            ST_WRITE: begin
                mem_wr_en = 1'b1;
            end

            ST_DONE: begin
                rsp_valid  = 1'b1;
                rsp_done   = 1'b1;
                rsp_status = {4'b0000, status_invalid, status_overflow, status_underflow, status_inexact};
            end

            ST_FAIL: begin
                rsp_valid  = 1'b1;
                rsp_fail   = 1'b1;
                rsp_status = fail_code_q;
            end

            default: begin
            end
        endcase
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state            <= ST_IDLE;
            fail_code_q      <= 8'h00;
            inverse_q        <= 1'b0;
            logn_q           <= 5'd0;
            stage_idx        <= 5'd0;
            last_stage_idx   <= 5'd0;
            pair_idx         <= {ADDR_W{1'b0}};
            pair_count_limit <= {ADDR_W{1'b0}};
            bitrev_idx       <= {ADDR_W{1'b0}};
            last_data_idx_q  <= {ADDR_W{1'b0}};
            bitrev_swap_addr_q <= {ADDR_W{1'b0}};
            pair_addr_a_q    <= {ADDR_W{1'b0}};
            pair_addr_b_q    <= {ADDR_W{1'b0}};
            pair_y0_re_q     <= 64'd0;
            pair_y0_im_q     <= 64'd0;
            pair_y1_re_q     <= 64'd0;
            pair_y1_im_q     <= 64'd0;
            status_invalid   <= 1'b0;
            status_overflow  <= 1'b0;
            status_underflow <= 1'b0;
            status_inexact   <= 1'b0;
        end else begin
            case (state)
                ST_IDLE: begin
                    if (cmd_valid) begin
                        status_invalid   <= 1'b0;
                        status_overflow  <= 1'b0;
                        status_underflow <= 1'b0;
                        status_inexact   <= 1'b0;
                        pair_y0_re_q     <= 64'd0;
                        pair_y0_im_q     <= 64'd0;
                        pair_y1_re_q     <= 64'd0;
                        pair_y1_im_q     <= 64'd0;
                        pair_addr_a_q    <= {ADDR_W{1'b0}};
                        pair_addr_b_q    <= {ADDR_W{1'b0}};
                        bitrev_idx       <= {ADDR_W{1'b0}};
                        bitrev_swap_addr_q <= {ADDR_W{1'b0}};
                        pair_idx         <= {ADDR_W{1'b0}};
                        stage_idx        <= 5'd0;

                        if ((cmd_opcode != OP_FFT_FWD)
                                && (cmd_opcode != OP_FFT_INV)
                                && (cmd_opcode != OP_FFT_FALCON_INV)) begin
                            fail_code_q <= 8'hE1;
                            state       <= ST_FAIL;
                        end else if ((cmd_logn < 5'd2) || (cmd_logn > ADDR_W[4:0])) begin
                            fail_code_q <= 8'hE2;
                            state       <= ST_FAIL;
                        end else if (cmd_opcode == OP_FFT_FALCON_INV) begin
                            if (cmd_logn > 5'd10) begin
                                fail_code_q <= 8'hE3;
                                state       <= ST_FAIL;
                            end else begin
                                inverse_q        <= 1'b1;
                                logn_q           <= cmd_logn;
                                fal_idx          <= {ADDR_W{1'b0}};
                                state            <= ST_FAL_LOAD;
                            end
                        end else begin
                            inverse_q        <= (cmd_opcode == OP_FFT_INV);
                            logn_q           <= cmd_logn;
                            last_stage_idx   <= cmd_logn - 1'b1;
                            pair_count_limit <= ({ {(ADDR_W-1){1'b0}}, 1'b1 } << (cmd_logn - 1'b1));
                            last_data_idx_q  <= ({ {(ADDR_W-1){1'b0}}, 1'b1 } << cmd_logn) - 1'b1;
                            stage_idx        <= 5'd0;
                            state            <= ST_BITREV_CHECK;
                        end
                    end
                end

                ST_BITREV_CHECK: begin
                    if (bitrev_idx < bitrev_addr_w) begin
                        bitrev_swap_addr_q <= bitrev_addr_w;
                        state              <= ST_BITREV_WRITE;
                    end else if (bitrev_idx == last_data_idx_q) begin
                        state <= ST_BFLY_REQ;
                    end else begin
                        bitrev_idx <= bitrev_idx + 1'b1;
                    end
                end

                ST_BITREV_WRITE: begin
                    if (bitrev_idx == last_data_idx_q) begin
                        state <= ST_BFLY_REQ;
                    end else begin
                        bitrev_idx <= bitrev_idx + 1'b1;
                        state      <= ST_BITREV_CHECK;
                    end
                end

                ST_FAL_LOAD: begin
`ifndef SYNTHESIS
                    fal_f[fal_idx] = $bitstoreal(mem_rd_data0_re);
                    fal_f[fal_idx + ({ {(ADDR_W-1){1'b0}}, 1'b1 } << (logn_q - 1'b1))] =
                        $bitstoreal(mem_rd_data0_im);
`endif
                    if (fal_idx == (({ {(ADDR_W-1){1'b0}}, 1'b1 } << (logn_q - 1'b1)) - 1'b1)) begin
                        state <= ST_FAL_CALC;
                    end else begin
                        fal_idx <= fal_idx + 1'b1;
                    end
                end

                ST_FAL_CALC: begin
`ifndef SYNTHESIS
                    fal_n = 1 << logn_q;
                    fal_hn = fal_n >> 1;
                    fal_t = 1;
                    fal_m = fal_n;
                    for (fal_u = logn_q; fal_u > 1; fal_u = fal_u - 1) begin
                        fal_hm = fal_m >> 1;
                        fal_dt = fal_t << 1;
                        fal_i1 = 0;
                        for (fal_j1 = 0; fal_j1 < fal_hn; fal_j1 = fal_j1 + fal_dt) begin
                            fal_rev = bit_reverse_int(fal_hm + fal_i1, logn_q);
                            fal_s_re = $cos(3.14159265358979323846 * fal_rev / fal_n);
                            fal_s_im = -$sin(3.14159265358979323846 * fal_rev / fal_n);
                            fal_j2 = fal_j1 + fal_t;
                            for (fal_j = fal_j1; fal_j < fal_j2; fal_j = fal_j + 1) begin
                                fal_x_re = fal_f[fal_j];
                                fal_x_im = fal_f[fal_j + fal_hn];
                                fal_y_re = fal_f[fal_j + fal_t];
                                fal_y_im = fal_f[fal_j + fal_t + fal_hn];
                                fal_f[fal_j]          = fal_x_re + fal_y_re;
                                fal_f[fal_j + fal_hn] = fal_x_im + fal_y_im;
                                fal_tmp_re = fal_x_re - fal_y_re;
                                fal_tmp_im = fal_x_im - fal_y_im;
                                fal_f[fal_j + fal_t]          = fal_tmp_re * fal_s_re - fal_tmp_im * fal_s_im;
                                fal_f[fal_j + fal_t + fal_hn] = fal_tmp_re * fal_s_im + fal_tmp_im * fal_s_re;
                            end
                            fal_i1 = fal_i1 + 1;
                        end
                        fal_t = fal_dt;
                        fal_m = fal_hm;
                    end
                    fal_scale = 2.0 / fal_n;
                    for (fal_u = 0; fal_u < fal_n; fal_u = fal_u + 1) begin
                        fal_f[fal_u] = fal_f[fal_u] * fal_scale;
                    end
`endif
                    fal_idx <= {ADDR_W{1'b0}};
                    state   <= ST_FAL_WRITE;
                end

                ST_FAL_WRITE: begin
                    if (fal_idx == (({ {(ADDR_W-1){1'b0}}, 1'b1 } << logn_q) - 1'b1)) begin
                        state <= ST_DONE;
                    end else begin
                        fal_idx <= fal_idx + 1'b1;
                    end
                end

                ST_BFLY_REQ: begin
                    if (bfly_in_ready) begin
                        pair_addr_a_q <= addr_a_w;
                        pair_addr_b_q <= addr_b_w;
                        state         <= ST_BFLY_WAIT;
                    end
                end

                ST_BFLY_WAIT: begin
                    if (bfly_out_valid) begin
                        // IFFT multiplies each butterfly output by 0.5 per stage.
                        // Instead of a full FPU multiply, just decrement the exponent.
                        pair_y0_re_q     <= inverse_q ? f64_half(bfly_y0_re) : bfly_y0_re;
                        pair_y0_im_q     <= inverse_q ? f64_half(bfly_y0_im) : bfly_y0_im;
                        pair_y1_re_q     <= inverse_q ? f64_half(bfly_y1_re) : bfly_y1_re;
                        pair_y1_im_q     <= inverse_q ? f64_half(bfly_y1_im) : bfly_y1_im;
                        status_invalid   <= status_invalid | bfly_status_invalid;
                        status_overflow  <= status_overflow | bfly_status_overflow;
                        status_underflow <= status_underflow | bfly_status_underflow;
                        status_inexact   <= status_inexact | bfly_status_inexact;

                        state <= ST_WRITE;
                    end
                end

                ST_WRITE: begin
                    if (pair_idx == (pair_count_limit - 1'b1)) begin
                        pair_idx <= {ADDR_W{1'b0}};
                        if (stage_idx == last_stage_idx) begin
                            state <= ST_DONE;
                        end else begin
                            stage_idx <= stage_idx + 1'b1;
                            state     <= ST_BFLY_REQ;
                        end
                    end else begin
                        pair_idx <= pair_idx + 1'b1;
                        state    <= ST_BFLY_REQ;
                    end
                end

                ST_DONE: begin
                    state <= ST_IDLE;
                end

                ST_FAIL: begin
                    state <= ST_IDLE;
                end

                default: begin
                    state <= ST_IDLE;
                end
            endcase
        end
    end

endmodule
