`timescale 1ns/1ps
// ffSampling task EXU.
// Implements the non-leaf Falcon dataflow:
//   splitfft:  f0=(a+b)/2, f1=((a-b)*conj(w))/2
//   mergefft:  a=f0+f1*w,  b=f0-f1*w
//   adjust:    t0'=t0+(t1-z1)*l10

module falcon_f64_ffsampling_exu #(parameter ADDR_W=10)(
    input  wire        clk, rst_n,
    input  wire        task_valid,
    output reg         task_ready,
    input  wire [67:0] task_word,
    output reg         task_done,
    output reg         task_fail,
    output reg [7:0]   task_status,

    output reg         mem_rd_en,
    output reg [ADDR_W-1:0] mem_rd_addr,
    input  wire [255:0] mem_rd_data,
    output reg         mem_wr_en,
    output reg [ADDR_W-1:0] mem_wr_addr,
    output reg [255:0] mem_wr_data,

    output reg [ADDR_W-1:0] twiddle_addr,
    input  wire [63:0] twiddle_re,
    input  wire [63:0] twiddle_im,

    output reg         fpu_req_valid,
    input  wire        fpu_req_ready,
    output reg [3:0]   fpu_req_op,
    output reg [63:0]  fpu_req_a,
    output reg [63:0]  fpu_req_b,
    output reg [63:0]  fpu_req_c,
    input  wire        fpu_rsp_valid,
    input  wire [63:0] fpu_rsp_result,

    output reg         sz_cmd_valid,
    input  wire        sz_cmd_ready,
    output reg [63:0]  sz_cmd_mu,
    output reg [63:0]  sz_cmd_sigma_inv,
    output reg         sz_cmd_pair,
    input  wire        sz_rsp_valid,
    input  wire [63:0] sz_rsp_z0,
    input  wire [63:0] sz_rsp_z1
);

localparam [3:0] OP_READ_L10 = 4'd0;
localparam [3:0] OP_SPLIT    = 4'd1;
localparam [3:0] OP_ADJUST   = 4'd2;
localparam [3:0] OP_SAMPLE   = 4'd3;
localparam [3:0] OP_MERGE    = 4'd4;

localparam [5:0] ST_IDLE       = 6'd0;
localparam [5:0] ST_READ_REQ   = 6'd1;
localparam [5:0] ST_READ_CAP   = 6'd2;
localparam [5:0] ST_PAIR_A_REQ = 6'd3;
localparam [5:0] ST_PAIR_A_CAP = 6'd4;
localparam [5:0] ST_PAIR_B_REQ = 6'd5;
localparam [5:0] ST_PAIR_B_CAP = 6'd6;
localparam [5:0] ST_ADJ_T1_REQ = 6'd7;
localparam [5:0] ST_ADJ_T1_CAP = 6'd8;
localparam [5:0] ST_ADJ_Z1_REQ = 6'd9;
localparam [5:0] ST_ADJ_Z1_CAP = 6'd10;
localparam [5:0] ST_ADJ_T0_REQ = 6'd11;
localparam [5:0] ST_ADJ_T0_CAP = 6'd12;
localparam [5:0] ST_FPU_REQ    = 6'd13;
localparam [5:0] ST_FPU_WAIT   = 6'd14;
localparam [5:0] ST_PAIR_WR0   = 6'd15;
localparam [5:0] ST_PAIR_WR1   = 6'd16;
localparam [5:0] ST_ADJ_WR     = 6'd17;
localparam [5:0] ST_SAMPLE_REQ = 6'd18;
localparam [5:0] ST_SAMPLE_WAIT= 6'd19;
localparam [5:0] ST_SAMPLE_WR  = 6'd20;
localparam [5:0] ST_DONE       = 6'd21;
localparam [5:0] ST_FAIL       = 6'd22;
localparam [5:0] ST_SAMPLE_MU_REQ  = 6'd23;
localparam [5:0] ST_SAMPLE_MU_CAP  = 6'd24;
localparam [5:0] ST_SAMPLE_SIG_REQ = 6'd25;
localparam [5:0] ST_SAMPLE_SIG_CAP = 6'd26;
localparam [5:0] ST_ADJ_L_REQ      = 6'd27;
localparam [5:0] ST_ADJ_L_CAP      = 6'd28;
localparam [5:0] ST_ADJ_MIRROR_WR  = 6'd29;
localparam [5:0] ST_PAIR_MIR0      = 6'd30;
localparam [5:0] ST_PAIR_MIR1      = 6'd31;

localparam [3:0] FADD  = 4'd0;
localparam [3:0] FSUB  = 4'd1;
localparam [3:0] FMUL  = 4'd2;
localparam [3:0] FMADD = 4'd3;
localparam [3:0] FMSUB = 4'd4;
localparam [3:0] FNMADD= 4'd6;

reg [5:0] state;
reg [3:0] op_q;
reg [3:0] level_q;
reg [ADDR_W-1:0] src0_q, src1_q, dst_q, l_base_q;
reg [ADDR_W-1:0] adj_t0_base_q;
reg [ADDR_W-1:0] pair_limit_q;
reg [ADDR_W-1:0] idx_q;
reg [2:0] lane_q;
reg [3:0] phase_q;

reg [63:0] a_re_q, a_im_q;
reg [63:0] b_re_q, b_im_q;
reg [63:0] t0_re_q, t0_im_q;
reg [63:0] t1_re_q, t1_im_q;
reg [63:0] z1_re_q, z1_im_q;
reg [63:0] l_re_q, l_im_q;
reg [63:0] sum_re_q, sum_im_q;
reg [63:0] diff_re_q, diff_im_q;
reg [63:0] rot_re_q, rot_im_q;
reg [63:0] out0_re_q, out0_im_q;
reg [63:0] out1_re_q, out1_im_q;
reg [63:0] tmp_q;
reg [63:0] sample_z0_q, sample_z1_q;
reg [63:0] sample_mu0_q, sample_mu1_q;
reg [63:0] sample_sig0_q, sample_sig1_q;

`ifndef SYNTHESIS
reg debug_adjust_nop;
reg debug_trace_exu;
initial debug_adjust_nop = $test$plusargs("FS_ADJUST_NOP");
initial debug_trace_exu = $test$plusargs("FS_TRACE_EXU");
`endif

wire [ADDR_W-1:0] adj_t0_base = adj_t0_base_q;
wire [ADDR_W-1:0] adj_mirror_addr = adj_t0_base_q + (pair_limit_q << 1) - 1'b1 - idx_q;
wire [ADDR_W-1:0] merge_mirror_base = dst_q + (pair_limit_q << 2) - 1'b1;
wire [ADDR_W-1:0] merge_mirror_addr0 = merge_mirror_base - (idx_q << 1);
wire [ADDR_W-1:0] merge_mirror_addr1 = merge_mirror_base - ((idx_q << 1) + 1'b1);

function [ADDR_W-1:0] pair_count_from_level;
    input [3:0] level;
    begin
        if (level >= 4'd8) begin
            pair_count_from_level = {{(ADDR_W-1){1'b0}}, 1'b1};
        end else begin
            pair_count_from_level = {{(ADDR_W-1){1'b0}}, 1'b1} << (4'd8 - level);
        end
    end
endfunction

function [ADDR_W-1:0] word_count_from_level;
    input [3:0] level;
    reg [ADDR_W-1:0] raw_count;
    begin
        if (level >= 4'd9) begin
            raw_count = {{(ADDR_W-1){1'b0}}, 1'b1};
        end else begin
            raw_count = {{(ADDR_W-1){1'b0}}, 1'b1} << (4'd9 - level);
        end
        word_count_from_level = (raw_count < 2) ? {{(ADDR_W-2){1'b0}}, 2'd2} : (raw_count >> 1);
    end
endfunction

function [63:0] f64_half;
    input [63:0] v;
    begin
        f64_half = (v[62:52] == 11'd0) ? v : {v[63], v[62:52] - 1'b1, v[51:0]};
    end
endfunction

function [63:0] f64_neg;
    input [63:0] v;
    begin
        f64_neg = (v[62:0] == 63'd0) ? v : {~v[63], v[62:0]};
    end
endfunction

function [ADDR_W-1:0] twiddle_index;
    input [ADDR_W-1:0] idx;
    input [3:0] level;
    reg [ADDR_W-1:0] base;
    begin
        // GM ROM address mapping: base[L] = 256 - 2^(8-L)
        // Level 0: 0, L1: 128, L2: 192, L3: 224, L4: 240,
        // L5: 248, L6: 252, L7: 254
        if (level >= 4'd8) begin
            twiddle_index = {{(ADDR_W-1){1'b0}}, 1'b1};
        end else begin
            base = {{(ADDR_W-1){1'b0}}, 1'b1} << (4'd8 - level);
            twiddle_index = 256 - base + idx;
        end
    end
endfunction

always @(*) begin
    task_ready       = (state == ST_IDLE);
    mem_rd_en        = 1'b0;
    mem_rd_addr      = {ADDR_W{1'b0}};
    mem_wr_en        = 1'b0;
    mem_wr_addr      = {ADDR_W{1'b0}};
    mem_wr_data      = 256'd0;
    twiddle_addr     = twiddle_index(idx_q, level_q);
    fpu_req_valid    = 1'b0;
    fpu_req_op       = FADD;
    fpu_req_a        = 64'd0;
    fpu_req_b        = 64'd0;
    fpu_req_c        = 64'd0;
    sz_cmd_valid     = 1'b0;
    sz_cmd_mu        = 64'd0;
    sz_cmd_sigma_inv = 64'd0;
    sz_cmd_pair      = 1'b1;

    case (state)
        ST_READ_REQ,
        ST_READ_CAP: begin
            mem_rd_en   = 1'b1;
            mem_rd_addr = src1_q;
        end

        ST_PAIR_A_REQ,
        ST_PAIR_A_CAP: begin
            mem_rd_en = 1'b1;
            if (op_q == OP_SPLIT) begin
                // Falcon FFT is bit-reversed: a = even index, b = odd index
                mem_rd_addr = src0_q + (idx_q << 1);
            end else begin
                mem_rd_addr = src0_q + idx_q;
            end
        end

        ST_PAIR_B_REQ,
        ST_PAIR_B_CAP: begin
            mem_rd_en = 1'b1;
            if (op_q == OP_SPLIT) begin
                // Falcon FFT is bit-reversed: a = even index, b = odd index
                mem_rd_addr = src0_q + (idx_q << 1) + 1'b1;
            end else begin
                mem_rd_addr = src0_q + pair_limit_q + idx_q;
            end
        end

        ST_ADJ_T1_REQ,
        ST_ADJ_T1_CAP: begin
            mem_rd_en   = 1'b1;
            mem_rd_addr = src0_q + idx_q;
        end

        ST_ADJ_Z1_REQ,
        ST_ADJ_Z1_CAP: begin
            mem_rd_en   = 1'b1;
            mem_rd_addr = dst_q + idx_q;
        end

        ST_ADJ_T0_REQ,
        ST_ADJ_T0_CAP: begin
            mem_rd_en   = 1'b1;
            mem_rd_addr = adj_t0_base + idx_q;
        end

        ST_ADJ_L_REQ,
        ST_ADJ_L_CAP: begin
            mem_rd_en   = 1'b1;
            mem_rd_addr = l_base_q + idx_q;
        end

        ST_SAMPLE_MU_REQ,
        ST_SAMPLE_MU_CAP: begin
            mem_rd_en   = 1'b1;
            mem_rd_addr = src0_q;
        end

        ST_SAMPLE_SIG_REQ,
        ST_SAMPLE_SIG_CAP: begin
            mem_rd_en   = 1'b1;
            mem_rd_addr = src1_q;
        end

        ST_FPU_REQ: begin
            fpu_req_valid = 1'b1;
            if ((op_q == OP_SPLIT) || (op_q == OP_MERGE)) begin
                if (op_q == OP_SPLIT) begin
                    case (phase_q)
                        4'd0: begin fpu_req_op = FADD;  fpu_req_a = a_re_q;   fpu_req_b = b_re_q; end
                        4'd1: begin fpu_req_op = FADD;  fpu_req_a = a_im_q;   fpu_req_b = b_im_q; end
                        4'd2: begin fpu_req_op = FSUB;  fpu_req_a = a_re_q;   fpu_req_b = b_re_q; end
                        4'd3: begin fpu_req_op = FSUB;  fpu_req_a = a_im_q;   fpu_req_b = b_im_q; end
                        4'd4: begin fpu_req_op = FMUL;  fpu_req_a = diff_re_q; fpu_req_b = twiddle_re; end
                        4'd5: begin fpu_req_op = FMADD; fpu_req_a = diff_im_q; fpu_req_b = twiddle_im; fpu_req_c = tmp_q; end
                        4'd6: begin fpu_req_op = FMUL;  fpu_req_a = diff_im_q; fpu_req_b = twiddle_re; end
                        default: begin fpu_req_op = FNMADD; fpu_req_a = diff_re_q; fpu_req_b = twiddle_im; fpu_req_c = tmp_q; end
                    endcase
                end else begin
                    case (phase_q)
                        4'd4: begin fpu_req_op = FMUL;  fpu_req_a = b_re_q; fpu_req_b = twiddle_re; end
                        4'd5: begin fpu_req_op = FNMADD; fpu_req_a = b_im_q; fpu_req_b = twiddle_im; fpu_req_c = tmp_q; end
                        4'd6: begin fpu_req_op = FMUL;  fpu_req_a = b_re_q; fpu_req_b = twiddle_im; end
                        4'd7: begin fpu_req_op = FMADD; fpu_req_a = b_im_q; fpu_req_b = twiddle_re; fpu_req_c = tmp_q; end
                        4'd8: begin fpu_req_op = FADD;  fpu_req_a = a_re_q; fpu_req_b = rot_re_q; end
                        4'd9: begin fpu_req_op = FSUB;  fpu_req_a = a_re_q; fpu_req_b = rot_re_q; end
                        4'd10: begin fpu_req_op = FADD; fpu_req_a = a_im_q; fpu_req_b = rot_im_q; end
                        default: begin fpu_req_op = FSUB; fpu_req_a = a_im_q; fpu_req_b = rot_im_q; end
                    endcase
                end
            end else begin
                case (phase_q)
                    4'd0: begin fpu_req_op = FSUB;  fpu_req_a = t1_re_q; fpu_req_b = z1_re_q; end
                    4'd1: begin fpu_req_op = FSUB;  fpu_req_a = t1_im_q; fpu_req_b = z1_im_q; end
                    4'd2: begin fpu_req_op = FMUL;  fpu_req_a = diff_re_q; fpu_req_b = l_re_q; end
                    4'd3: begin fpu_req_op = FNMADD; fpu_req_a = diff_im_q; fpu_req_b = l_im_q; fpu_req_c = tmp_q; end
                    4'd4: begin fpu_req_op = FMUL;  fpu_req_a = diff_re_q; fpu_req_b = l_im_q; end
                    4'd5: begin fpu_req_op = FMADD; fpu_req_a = diff_im_q; fpu_req_b = l_re_q; fpu_req_c = tmp_q; end
                    4'd6: begin fpu_req_op = FADD;  fpu_req_a = t0_re_q; fpu_req_b = rot_re_q; end
                    default: begin fpu_req_op = FADD; fpu_req_a = t0_im_q; fpu_req_b = rot_im_q; end
                endcase
            end
        end

        ST_PAIR_WR0: begin
            mem_wr_en = 1'b1;
            if (op_q == OP_SPLIT) begin
                mem_wr_addr = dst_q + idx_q;
                if (level_q >= 4'd9) begin
                    mem_wr_data = {192'd0, a_re_q};
                end else begin
                    mem_wr_data = {128'd0, f64_half(sum_im_q), f64_half(sum_re_q)};
                end
`ifndef SYNTHESIS
                if (debug_trace_exu && (op_q == OP_SPLIT) && (level_q <= 4'd1) && (idx_q < 2)) begin
                    $display("  FE_SPLIT_WR0 L=%0d I=%0d addr=%0d data=%h", level_q, idx_q, dst_q + idx_q, mem_wr_data);
                end
`endif
            end else begin
                mem_wr_addr = dst_q + (idx_q << 1);
                mem_wr_data = {128'd0, out0_im_q, out0_re_q};
`ifndef SYNTHESIS
                if (debug_trace_exu && (op_q == OP_MERGE) && (level_q == 4'd0) && (idx_q < 2)) begin
                    $display("  FE_MERGE_WR0 level=%0d idx=%0d addr=%0d data=%h", level_q, idx_q, dst_q + (idx_q << 1), {128'd0, out0_im_q, out0_re_q});
                end
`endif
            end
        end

        ST_PAIR_WR1: begin
            mem_wr_en = 1'b1;
            if (op_q == OP_SPLIT) begin
                mem_wr_addr = dst_q + pair_limit_q + idx_q;
                if (level_q >= 4'd9) begin
                    mem_wr_data = {192'd0, a_im_q};
                end else begin
                    mem_wr_data = {128'd0, f64_half(rot_im_q), f64_half(rot_re_q)};
                end
`ifndef SYNTHESIS
                if (debug_trace_exu && (op_q == OP_SPLIT) && (level_q <= 4'd1) && (idx_q < 2)) begin
                    $display("  FE_SPLIT_WR1 L=%0d I=%0d addr=%0d data=%h", level_q, idx_q, dst_q + pair_limit_q + idx_q, mem_wr_data);
                end
`endif
            end else begin
                mem_wr_addr = dst_q + (idx_q << 1) + 1'b1;
                mem_wr_data = {128'd0, out1_im_q, out1_re_q};
`ifndef SYNTHESIS
                if (debug_trace_exu && (op_q == OP_MERGE) && (level_q == 4'd0) && (idx_q < 2)) begin
                    $display("  FE_MERGE_WR1 level=%0d idx=%0d addr=%0d data=%h", level_q, idx_q, dst_q + (idx_q << 1) + 1'b1, {128'd0, out1_im_q, out1_re_q});
                end
`endif
            end
        end

        ST_PAIR_MIR0: begin
            mem_wr_en   = 1'b1;
            mem_wr_addr = merge_mirror_addr0;
            mem_wr_data = {128'd0, f64_neg(out0_im_q), out0_re_q};
        end

        ST_PAIR_MIR1: begin
            mem_wr_en   = 1'b1;
            mem_wr_addr = merge_mirror_addr1;
            mem_wr_data = {128'd0, f64_neg(out1_im_q), out1_re_q};
        end

        ST_ADJ_WR: begin
            mem_wr_en = 1'b1;
            mem_wr_addr = adj_t0_base + idx_q;
            mem_wr_data = {128'd0, out0_im_q, out0_re_q};
`ifndef SYNTHESIS
            if (debug_trace_exu && (level_q <= 4'd0) && (idx_q < 2)) begin
                $display("  FE_ADJ_WR L=%0d I=%0d addr=%0d data=%h", level_q, idx_q, mem_wr_addr, mem_wr_data);
            end
`endif
        end

        ST_ADJ_MIRROR_WR: begin
            mem_wr_en = 1'b1;
            mem_wr_addr = adj_mirror_addr;
            mem_wr_data = {128'd0, f64_neg(out0_im_q), out0_re_q};
        end

        ST_SAMPLE_REQ: begin
            sz_cmd_valid     = 1'b1;
            sz_cmd_mu        = lane_q[0] ? sample_mu1_q  : sample_mu0_q;
            sz_cmd_sigma_inv = lane_q[0] ? sample_sig1_q : sample_sig0_q;
            sz_cmd_pair      = 1'b0;
        end

        ST_SAMPLE_WR: begin
            mem_wr_en   = 1'b1;
            mem_wr_addr = dst_q;
            mem_wr_data = {128'd0, sample_z1_q, sample_z0_q};
        end
    endcase
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state        <= ST_IDLE;
        task_done    <= 1'b0;
        task_fail    <= 1'b0;
        task_status  <= 8'd0;
        op_q         <= 4'd0;
        level_q      <= 4'd0;
        src0_q       <= {ADDR_W{1'b0}};
        src1_q       <= {ADDR_W{1'b0}};
        dst_q        <= {ADDR_W{1'b0}};
        l_base_q     <= {ADDR_W{1'b0}};
        adj_t0_base_q <= {ADDR_W{1'b0}};
        pair_limit_q <= {ADDR_W{1'b0}};
        idx_q        <= {ADDR_W{1'b0}};
        lane_q       <= 3'd0;
        phase_q      <= 4'd0;
        a_re_q       <= 64'd0;
        a_im_q       <= 64'd0;
        b_re_q       <= 64'd0;
        b_im_q       <= 64'd0;
        t0_re_q      <= 64'd0;
        t0_im_q      <= 64'd0;
        t1_re_q      <= 64'd0;
        t1_im_q      <= 64'd0;
        z1_re_q      <= 64'd0;
        z1_im_q      <= 64'd0;
        l_re_q       <= 64'd0;
        l_im_q       <= 64'd0;
        sum_re_q     <= 64'd0;
        sum_im_q     <= 64'd0;
        diff_re_q    <= 64'd0;
        diff_im_q    <= 64'd0;
        rot_re_q     <= 64'd0;
        rot_im_q     <= 64'd0;
        out0_re_q    <= 64'd0;
        out0_im_q    <= 64'd0;
        out1_re_q    <= 64'd0;
        out1_im_q    <= 64'd0;
        tmp_q        <= 64'd0;
        sample_z0_q  <= 64'd0;
        sample_z1_q  <= 64'd0;
        sample_mu0_q <= 64'd0;
        sample_mu1_q <= 64'd0;
        sample_sig0_q <= 64'd0;
        sample_sig1_q <= 64'd0;
    end else begin
        task_done   <= 1'b0;
        task_fail   <= 1'b0;
        task_status <= 8'd0;

        case (state)
            ST_IDLE: begin
                if (task_valid) begin
                    op_q         <= task_word[67:64];
                    level_q      <= task_word[63:60];
                    src0_q       <= task_word[49:36];
                    src1_q       <= task_word[35:22];
                    dst_q        <= task_word[21:8];
                    pair_limit_q <= pair_count_from_level(task_word[63:60]);
                    idx_q        <= {ADDR_W{1'b0}};
                    lane_q       <= 3'd0;
                    phase_q      <= 4'd0;
                    case (task_word[67:64])
                        OP_READ_L10: state <= ST_READ_REQ;
                        OP_SPLIT:    state <= ST_PAIR_A_REQ;
                        OP_MERGE:    state <= ST_PAIR_A_REQ;
                        OP_ADJUST: begin
                            // task_word[7] doubles pair_limit for root-level full-polynomial ADJUST
                            pair_limit_q <= task_word[7]
                                ? (word_count_from_level(task_word[63:60]) << 1)
                                : word_count_from_level(task_word[63:60]);
                            l_base_q     <= task_word[35:22];
                            adj_t0_base_q <= {task_word[7:4], task_word[59:50]};
                            state        <= ST_ADJ_L_REQ;
                        end
                        OP_SAMPLE:   state <= ST_SAMPLE_MU_REQ;
                        default: begin
                            task_done   <= 1'b1;
                            task_fail   <= 1'b1;
                            task_status <= 8'hE1;
                            state       <= ST_IDLE;
                        end
                    endcase
                end
            end

            ST_READ_REQ: begin
                state <= ST_READ_CAP;
            end

            ST_READ_CAP: begin
                l_re_q <= mem_rd_data[63:0];
                l_im_q <= mem_rd_data[127:64];
                l_base_q <= src1_q;
                state  <= ST_DONE;
            end

            ST_ADJ_L_REQ: begin
                state <= ST_ADJ_L_CAP;
            end

            ST_ADJ_L_CAP: begin
                l_re_q <= mem_rd_data[63:0];
                l_im_q <= mem_rd_data[127:64];
                state  <= ST_ADJ_T1_REQ;
            end

            ST_PAIR_A_REQ: begin
                state <= ST_PAIR_A_CAP;
            end

        ST_PAIR_A_CAP: begin
            a_re_q <= mem_rd_data[63:0];
            a_im_q <= mem_rd_data[127:64];
`ifndef SYNTHESIS
            if (debug_trace_exu && (op_q == OP_SPLIT) && (level_q == 4'd0) && (idx_q < 2)) begin
                $display("  FE_SPLIT_A level=%0d idx=%0d addr=%0d data=%h", level_q, idx_q, src0_q + (idx_q << 1), mem_rd_data);
            end
`endif
            if ((op_q == OP_SPLIT) && (level_q >= 4'd9)) begin
                state <= ST_PAIR_WR0;
            end else begin
                state <= ST_PAIR_B_REQ;
            end
        end

            ST_PAIR_B_REQ: begin
                state <= ST_PAIR_B_CAP;
            end

        ST_PAIR_B_CAP: begin
            b_re_q  <= mem_rd_data[63:0];
            b_im_q  <= mem_rd_data[127:64];
`ifndef SYNTHESIS
            if (debug_trace_exu && (op_q == OP_SPLIT) && (level_q == 4'd0) && (idx_q < 2)) begin
                $display("  FE_SPLIT_B level=%0d idx=%0d addr=%0d data=%h", level_q, idx_q, src0_q + (idx_q << 1) + 1'b1, mem_rd_data);
            end
`endif
            if ((op_q == OP_MERGE) && (level_q >= 4'd9)) begin
                out0_re_q <= a_re_q;
                out0_im_q <= mem_rd_data[63:0];
                state     <= ST_PAIR_WR0;
            end else begin
                phase_q <= (op_q == OP_SPLIT) ? 4'd0 : 4'd4;
                state   <= ST_FPU_REQ;
            end
        end

            ST_ADJ_T1_REQ: begin
                state <= ST_ADJ_T1_CAP;
            end

            ST_ADJ_T1_CAP: begin
                t1_re_q <= mem_rd_data[63:0];
                t1_im_q <= mem_rd_data[127:64];
                state <= ST_ADJ_Z1_REQ;
            end

            ST_ADJ_Z1_REQ: begin
                state <= ST_ADJ_Z1_CAP;
            end

            ST_ADJ_Z1_CAP: begin
                z1_re_q <= mem_rd_data[63:0];
                z1_im_q <= mem_rd_data[127:64];
                state <= ST_ADJ_T0_REQ;
            end

            ST_ADJ_T0_REQ: begin
                state <= ST_ADJ_T0_CAP;
            end

            ST_ADJ_T0_CAP: begin
                t0_re_q <= mem_rd_data[63:0];
                t0_im_q <= mem_rd_data[127:64];
`ifndef SYNTHESIS
                if (debug_trace_exu && (level_q <= 4'd0) && (idx_q < 2)) begin
                    $display("  FE_ADJ_IN L=%0d I=%0d: t0={%h,%h} t1={%h,%h} z1={%h,%h} l={%h,%h}",
                             level_q, idx_q,
                             mem_rd_data[63:0], mem_rd_data[127:64],
                             t1_re_q, t1_im_q,
                             z1_re_q, z1_im_q,
                             l_re_q, l_im_q);
                end
                if (debug_adjust_nop) begin
                    out0_re_q <= mem_rd_data[63:0];
                    out0_im_q <= mem_rd_data[127:64];
                    state     <= ST_ADJ_WR;
                end else begin
`endif
                phase_q <= 4'd0;
                state   <= ST_FPU_REQ;
`ifndef SYNTHESIS
                end
`endif
            end

            ST_FPU_REQ: begin
                if (fpu_req_ready) begin
                    state <= ST_FPU_WAIT;
                end
            end

            ST_FPU_WAIT: begin
                if (fpu_rsp_valid) begin
                    if (op_q == OP_SPLIT) begin
                        case (phase_q)
                            4'd0: begin sum_re_q  <= fpu_rsp_result; phase_q <= 4'd1; state <= ST_FPU_REQ; end
                            4'd1: begin sum_im_q  <= fpu_rsp_result; phase_q <= 4'd2; state <= ST_FPU_REQ; end
                            4'd2: begin diff_re_q <= fpu_rsp_result; phase_q <= 4'd3; state <= ST_FPU_REQ; end
                            4'd3: begin diff_im_q <= fpu_rsp_result; phase_q <= 4'd4; state <= ST_FPU_REQ; end
                            4'd4: begin tmp_q     <= fpu_rsp_result; phase_q <= 4'd5; state <= ST_FPU_REQ; end
                            4'd5: begin rot_re_q  <= fpu_rsp_result; phase_q <= 4'd6; state <= ST_FPU_REQ; end
                            4'd6: begin tmp_q     <= fpu_rsp_result; phase_q <= 4'd7; state <= ST_FPU_REQ; end
                            default: begin rot_im_q <= fpu_rsp_result; state <= ST_PAIR_WR0; end
                        endcase
                    end else if (op_q == OP_MERGE) begin
                        case (phase_q)
                            4'd4: begin tmp_q     <= fpu_rsp_result; phase_q <= 4'd5; state <= ST_FPU_REQ; end
                            4'd5: begin rot_re_q  <= fpu_rsp_result; phase_q <= 4'd6; state <= ST_FPU_REQ; end
                            4'd6: begin tmp_q     <= fpu_rsp_result; phase_q <= 4'd7; state <= ST_FPU_REQ; end
                            4'd7: begin rot_im_q  <= fpu_rsp_result; phase_q <= 4'd8; state <= ST_FPU_REQ; end
                            4'd8: begin out0_re_q <= fpu_rsp_result; phase_q <= 4'd9; state <= ST_FPU_REQ; end
                            4'd9: begin out1_re_q <= fpu_rsp_result; phase_q <= 4'd10; state <= ST_FPU_REQ; end
                            4'd10: begin out0_im_q <= fpu_rsp_result; phase_q <= 4'd11; state <= ST_FPU_REQ; end
                            default: begin out1_im_q <= fpu_rsp_result; state <= ST_PAIR_WR0; end
                        endcase
                    end else begin
                        case (phase_q)
                            4'd0: begin diff_re_q <= fpu_rsp_result; phase_q <= 4'd1; state <= ST_FPU_REQ; end
                            4'd1: begin diff_im_q <= fpu_rsp_result; phase_q <= 4'd2; state <= ST_FPU_REQ; end
                            4'd2: begin tmp_q     <= fpu_rsp_result; phase_q <= 4'd3; state <= ST_FPU_REQ; end
                            4'd3: begin rot_re_q  <= fpu_rsp_result; phase_q <= 4'd4; state <= ST_FPU_REQ; end
                            4'd4: begin tmp_q     <= fpu_rsp_result; phase_q <= 4'd5; state <= ST_FPU_REQ; end
                            4'd5: begin rot_im_q  <= fpu_rsp_result; phase_q <= 4'd6; state <= ST_FPU_REQ; end
                            4'd6: begin
                                out0_re_q <= fpu_rsp_result;
                                phase_q <= 4'd7;
                                state <= ST_FPU_REQ;
                            end
                            default: begin
                                out0_im_q <= fpu_rsp_result;
                                state <= ST_ADJ_WR;
                            end
                        endcase
                    end
                end
            end

            ST_PAIR_WR0: begin
                if ((op_q == OP_MERGE) && (level_q >= 4'd9)) begin
                    state <= ST_DONE;
                end else begin
                    state <= ST_PAIR_WR1;
                end
            end

            ST_PAIR_MIR0: begin
                state <= ST_PAIR_WR1;
            end

            ST_PAIR_WR1: begin
                if (idx_q == (pair_limit_q - 1'b1)) begin
                    state <= ST_DONE;
                end else begin
                    idx_q   <= idx_q + 1'b1;
                    phase_q <= 4'd0;
                    state   <= ST_PAIR_A_REQ;
                end
            end

            ST_PAIR_MIR1: begin
                if (idx_q == (pair_limit_q - 1'b1)) begin
                    state <= ST_DONE;
                end else begin
                    idx_q   <= idx_q + 1'b1;
                    phase_q <= 4'd0;
                    state   <= ST_PAIR_A_REQ;
                end
            end

            ST_ADJ_WR: begin
                state <= ST_ADJ_MIRROR_WR;
            end

            ST_ADJ_MIRROR_WR: begin
                if (idx_q == (pair_limit_q - 1'b1)) begin
                    state <= ST_DONE;
                end else begin
                    idx_q   <= idx_q + 1'b1;
                    lane_q  <= 3'd0;
                    phase_q <= 4'd0;
                    state   <= ST_ADJ_L_REQ;
                end
            end

            ST_SAMPLE_MU_REQ: begin
                state <= ST_SAMPLE_MU_CAP;
            end

            ST_SAMPLE_MU_CAP: begin
                sample_mu0_q <= mem_rd_data[63:0];
                sample_mu1_q <= mem_rd_data[127:64];
                state        <= ST_SAMPLE_SIG_REQ;
            end

            ST_SAMPLE_SIG_REQ: begin
                state <= ST_SAMPLE_SIG_CAP;
            end

            ST_SAMPLE_SIG_CAP: begin
                sample_sig0_q <= mem_rd_data[63:0];
                sample_sig1_q <= mem_rd_data[127:64];
                lane_q        <= 3'd0;
                state         <= ST_SAMPLE_REQ;
            end

            ST_SAMPLE_REQ: begin
                if (sz_cmd_ready) begin
                    state <= ST_SAMPLE_WAIT;
                end
            end

            ST_SAMPLE_WAIT: begin
                if (sz_rsp_valid) begin
                    if (lane_q[0]) begin
                        sample_z1_q <= sz_rsp_z0;
                        state       <= ST_SAMPLE_WR;
                    end else begin
                        sample_z0_q <= sz_rsp_z0;
                        lane_q      <= 3'd1;
                        state       <= ST_SAMPLE_REQ;
                    end
                end
            end

            ST_SAMPLE_WR: begin
                state <= ST_DONE;
            end

            ST_DONE: begin
                task_done <= 1'b1;
                state     <= ST_IDLE;
            end

            ST_FAIL: begin
                task_done   <= 1'b1;
                task_fail   <= 1'b1;
                task_status <= 8'hFF;
                state       <= ST_IDLE;
            end

            default: begin
                state <= ST_IDLE;
            end
        endcase
    end
end

endmodule
