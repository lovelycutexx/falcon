`timescale 1ns/1ps

// Falcon frequency-domain basis multiplication, second signature component.
//
// Real Falcon signing computes the lattice point:
//   s1_fft = z0 * b00 + z1 * b10
//   s2_fft = z0 * b01 + z1 * b11
//
// This EXU currently writes the second component because the top-level signing
// path consumes s2 after iFFT.  Memory layout for non-identity mode:
//   t0: t_base,              t1: t_base + word_count
//   z0: z_base,              z1: z_base + word_count
//   b01: b01_base,           b11: b11_base
//   s2_fft: s2_fft_base
//
// identity_mode preserves the earlier bring-up behavior:
//   s2_fft = t0 - z0
module falcon_f64_bhat_mul_exu #(
    parameter ADDR_W = 11
) (
    input  wire              clk,
    input  wire              rst_n,

    input  wire              start,
    output wire              start_ready,
    input  wire              identity_mode,
    input  wire [ADDR_W-1:0] t_base,
    input  wire [ADDR_W-1:0] z_base,
    input  wire [ADDR_W-1:0] b00_base,
    input  wire [ADDR_W-1:0] b01_base,
    input  wire [ADDR_W-1:0] b10_base,
    input  wire [ADDR_W-1:0] b11_base,
    input  wire [ADDR_W-1:0] s2_fft_base,
    input  wire [ADDR_W-1:0] word_count,

    output reg               mem_rd_en,
    output reg  [ADDR_W-1:0] mem_rd_addr,
    input  wire [255:0]      mem_rd_data,
    output reg               mem_wr_en,
    output reg  [ADDR_W-1:0] mem_wr_addr,
    output reg  [255:0]      mem_wr_data,

    output reg               fpu_req_valid,
    input  wire              fpu_req_ready,
    output reg  [3:0]        fpu_req_op,
    output reg  [63:0]       fpu_req_a,
    output reg  [63:0]       fpu_req_b,
    output reg  [63:0]       fpu_req_c,
    input  wire              fpu_rsp_valid,
    input  wire [63:0]       fpu_rsp_result,

    output reg               done,
    output reg               fail,
    output reg  [7:0]        status
);

    localparam [5:0] ST_IDLE      = 6'd0;
    localparam [5:0] ST_RD_T0     = 6'd1;
    localparam [5:0] ST_WAIT1     = 6'd2;
    localparam [5:0] ST_WAIT2     = 6'd3;
    localparam [5:0] ST_CAP_T0    = 6'd4;
    localparam [5:0] ST_RD_Z0     = 6'd5;
    localparam [5:0] ST_CAP_Z0    = 6'd6;
    localparam [5:0] ST_RD_T1     = 6'd7;
    localparam [5:0] ST_CAP_T1    = 6'd8;
    localparam [5:0] ST_RD_Z1     = 6'd9;
    localparam [5:0] ST_CAP_Z1    = 6'd10;
    localparam [5:0] ST_RD_B01    = 6'd11;
    localparam [5:0] ST_CAP_B01   = 6'd12;
    localparam [5:0] ST_RD_B11    = 6'd13;
    localparam [5:0] ST_CAP_B11   = 6'd14;
    localparam [5:0] ST_FPU_REQ   = 6'd15;
    localparam [5:0] ST_FPU_WAIT  = 6'd16;
    localparam [5:0] ST_WR        = 6'd17;
    localparam [5:0] ST_DONE      = 6'd18;
    localparam [5:0] ST_FAIL      = 6'd19;

    localparam [3:0] FADD   = 4'd0;
    localparam [3:0] FSUB   = 4'd1;
    localparam [3:0] FMUL   = 4'd2;
    localparam [3:0] FMADD  = 4'd3;
    localparam [3:0] FNMADD = 4'd6;

    localparam [4:0] PH_D0_RE   = 5'd0;
    localparam [4:0] PH_D0_IM   = 5'd1;
    localparam [4:0] PH_D1_RE   = 5'd2;
    localparam [4:0] PH_D1_IM   = 5'd3;
    localparam [4:0] PH_M0_RE_A = 5'd4;
    localparam [4:0] PH_M0_RE_B = 5'd5;
    localparam [4:0] PH_M0_IM_A = 5'd6;
    localparam [4:0] PH_M0_IM_B = 5'd7;
    localparam [4:0] PH_M1_RE_A = 5'd8;
    localparam [4:0] PH_M1_RE_B = 5'd9;
    localparam [4:0] PH_M1_IM_A = 5'd10;
    localparam [4:0] PH_M1_IM_B = 5'd11;
    localparam [4:0] PH_SUM_RE  = 5'd12;
    localparam [4:0] PH_SUM_IM  = 5'd13;

    reg [5:0] state;
    reg [5:0] read_return_state;
    reg [4:0] phase_q;
    reg [ADDR_W-1:0] idx_q;
    reg mode_identity_q;

    reg [63:0] t0_re_q, t0_im_q, z0_re_q, z0_im_q;
    reg [63:0] t1_re_q, t1_im_q, z1_re_q, z1_im_q;
    reg [63:0] b01_re_q, b01_im_q, b11_re_q, b11_im_q;
    reg [63:0] d0_re_q, d0_im_q, d1_re_q, d1_im_q;
    reg [63:0] tmp_q;
    reg [63:0] m0_re_q, m0_im_q, m1_re_q, m1_im_q;
    reg [63:0] out_re_q, out_im_q;

    wire [ADDR_W-1:0] t1_base = t_base + word_count;
    wire [ADDR_W-1:0] z1_base = z_base + word_count;

    // Ports reserved for the first component path.
    wire unused_first_component_bases = ^(b00_base ^ b10_base);

    assign start_ready = (state == ST_IDLE);

    always @(*) begin
        mem_rd_en     = 1'b0;
        mem_rd_addr   = t_base + idx_q;
        mem_wr_en     = 1'b0;
        mem_wr_addr   = s2_fft_base + idx_q;
        mem_wr_data   = {128'd0, out_im_q, out_re_q};
        fpu_req_valid = 1'b0;
        fpu_req_op    = FSUB;
        fpu_req_a     = 64'd0;
        fpu_req_b     = 64'd0;
        fpu_req_c     = 64'd0;

        case (state)
            ST_RD_T0: begin
                mem_rd_en   = 1'b1;
                mem_rd_addr = t_base + idx_q;
            end
            ST_RD_Z0: begin
                mem_rd_en   = 1'b1;
                mem_rd_addr = z_base + idx_q;
            end
            ST_RD_T1: begin
                mem_rd_en   = 1'b1;
                mem_rd_addr = t1_base + idx_q;
            end
            ST_RD_Z1: begin
                mem_rd_en   = 1'b1;
                mem_rd_addr = z1_base + idx_q;
            end
            ST_RD_B01: begin
                mem_rd_en   = 1'b1;
                mem_rd_addr = b01_base + idx_q;
            end
            ST_RD_B11: begin
                mem_rd_en   = 1'b1;
                mem_rd_addr = b11_base + idx_q;
            end
            ST_FPU_REQ: begin
                fpu_req_valid = 1'b1;
                case (phase_q)
                    PH_D0_RE: begin fpu_req_op = mode_identity_q ? FSUB : FADD; fpu_req_a = mode_identity_q ? t0_re_q : z0_re_q; fpu_req_b = mode_identity_q ? z0_re_q : 64'd0; end
                    PH_D0_IM: begin fpu_req_op = mode_identity_q ? FSUB : FADD; fpu_req_a = mode_identity_q ? t0_im_q : z0_im_q; fpu_req_b = mode_identity_q ? z0_im_q : 64'd0; end
                    PH_D1_RE: begin fpu_req_op = FADD;   fpu_req_a = z1_re_q; fpu_req_b = 64'd0; end
                    PH_D1_IM: begin fpu_req_op = FADD;   fpu_req_a = z1_im_q; fpu_req_b = 64'd0; end
                    PH_M0_RE_A: begin fpu_req_op = FMUL;   fpu_req_a = d0_re_q; fpu_req_b = b01_re_q; end
                    PH_M0_RE_B: begin fpu_req_op = FNMADD; fpu_req_a = d0_im_q; fpu_req_b = b01_im_q; fpu_req_c = tmp_q; end
                    PH_M0_IM_A: begin fpu_req_op = FMUL;   fpu_req_a = d0_re_q; fpu_req_b = b01_im_q; end
                    PH_M0_IM_B: begin fpu_req_op = FMADD;  fpu_req_a = d0_im_q; fpu_req_b = b01_re_q; fpu_req_c = tmp_q; end
                    PH_M1_RE_A: begin fpu_req_op = FMUL;   fpu_req_a = d1_re_q; fpu_req_b = b11_re_q; end
                    PH_M1_RE_B: begin fpu_req_op = FNMADD; fpu_req_a = d1_im_q; fpu_req_b = b11_im_q; fpu_req_c = tmp_q; end
                    PH_M1_IM_A: begin fpu_req_op = FMUL;   fpu_req_a = d1_re_q; fpu_req_b = b11_im_q; end
                    PH_M1_IM_B: begin fpu_req_op = FMADD;  fpu_req_a = d1_im_q; fpu_req_b = b11_re_q; fpu_req_c = tmp_q; end
                    PH_SUM_RE:  begin fpu_req_op = FADD;   fpu_req_a = m0_re_q; fpu_req_b = m1_re_q; end
                    PH_SUM_IM:  begin fpu_req_op = FADD;   fpu_req_a = m0_im_q; fpu_req_b = m1_im_q; end
                    default:    begin fpu_req_op = FSUB; end
                endcase
            end
            ST_WR: begin
                mem_wr_en   = 1'b1;
                mem_wr_addr = s2_fft_base + idx_q;
                mem_wr_data = {128'd0, out_im_q, out_re_q};
            end
            default: begin
            end
        endcase
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= ST_IDLE;
            read_return_state <= ST_IDLE;
            phase_q <= PH_D0_RE;
            idx_q <= {ADDR_W{1'b0}};
            mode_identity_q <= 1'b1;
            t0_re_q <= 64'd0; t0_im_q <= 64'd0; z0_re_q <= 64'd0; z0_im_q <= 64'd0;
            t1_re_q <= 64'd0; t1_im_q <= 64'd0; z1_re_q <= 64'd0; z1_im_q <= 64'd0;
            b01_re_q <= 64'd0; b01_im_q <= 64'd0; b11_re_q <= 64'd0; b11_im_q <= 64'd0;
            d0_re_q <= 64'd0; d0_im_q <= 64'd0; d1_re_q <= 64'd0; d1_im_q <= 64'd0;
            tmp_q <= 64'd0;
            m0_re_q <= 64'd0; m0_im_q <= 64'd0; m1_re_q <= 64'd0; m1_im_q <= 64'd0;
            out_re_q <= 64'd0; out_im_q <= 64'd0;
            done <= 1'b0;
            fail <= 1'b0;
            status <= 8'h00;
        end else begin
            done <= 1'b0;
            fail <= 1'b0;

            case (state)
                ST_IDLE: begin
                    status <= 8'h00;
                    if (start) begin
                        idx_q <= {ADDR_W{1'b0}};
                        mode_identity_q <= identity_mode;
                        if (word_count == {ADDR_W{1'b0}}) begin
                            status <= 8'hE1;
                            state  <= ST_FAIL;
                        end else begin
                            state <= ST_RD_T0;
                        end
                    end
                end

                ST_RD_T0:  begin read_return_state <= ST_CAP_T0;  state <= ST_WAIT1; end
                ST_RD_Z0:  begin read_return_state <= ST_CAP_Z0;  state <= ST_WAIT1; end
                ST_RD_T1:  begin read_return_state <= ST_CAP_T1;  state <= ST_WAIT1; end
                ST_RD_Z1:  begin read_return_state <= ST_CAP_Z1;  state <= ST_WAIT1; end
                ST_RD_B01: begin read_return_state <= ST_CAP_B01; state <= ST_WAIT1; end
                ST_RD_B11: begin read_return_state <= ST_CAP_B11; state <= ST_WAIT1; end
                ST_WAIT1:  begin state <= ST_WAIT2; end
                ST_WAIT2:  begin state <= read_return_state; end

                ST_CAP_T0: begin
                    t0_re_q <= mem_rd_data[63:0];
                    t0_im_q <= mem_rd_data[127:64];
                    state   <= ST_RD_Z0;
                end

                ST_CAP_Z0: begin
                    z0_re_q <= mem_rd_data[63:0];
                    z0_im_q <= mem_rd_data[127:64];
                    if (mode_identity_q) begin
                        phase_q <= PH_D0_RE;
                        state   <= ST_FPU_REQ;
                    end else begin
                        state <= ST_RD_T1;
                    end
                end

                ST_CAP_T1: begin
                    t1_re_q <= mem_rd_data[63:0];
                    t1_im_q <= mem_rd_data[127:64];
                    state   <= ST_RD_Z1;
                end

                ST_CAP_Z1: begin
                    z1_re_q <= mem_rd_data[63:0];
                    z1_im_q <= mem_rd_data[127:64];
                    state   <= ST_RD_B01;
                end

                ST_CAP_B01: begin
                    b01_re_q <= mem_rd_data[63:0];
                    b01_im_q <= mem_rd_data[127:64];
                    state    <= ST_RD_B11;
                end

                ST_CAP_B11: begin
                    b11_re_q <= mem_rd_data[63:0];
                    b11_im_q <= mem_rd_data[127:64];
                    phase_q  <= PH_D0_RE;
                    state    <= ST_FPU_REQ;
                end

                ST_FPU_REQ: begin
                    if (fpu_req_ready)
                        state <= ST_FPU_WAIT;
                end

                ST_FPU_WAIT: begin
                    if (fpu_rsp_valid) begin
                        case (phase_q)
                            PH_D0_RE: begin
                                d0_re_q <= fpu_rsp_result;
                                phase_q <= PH_D0_IM;
                                state   <= ST_FPU_REQ;
                            end
                            PH_D0_IM: begin
                                d0_im_q <= fpu_rsp_result;
                                if (mode_identity_q) begin
                                    out_re_q <= d0_re_q;
                                    out_im_q <= fpu_rsp_result;
                                    state    <= ST_WR;
                                end else begin
                                    phase_q <= PH_D1_RE;
                                    state   <= ST_FPU_REQ;
                                end
                            end
                            PH_D1_RE: begin
                                d1_re_q <= fpu_rsp_result;
                                phase_q <= PH_D1_IM;
                                state   <= ST_FPU_REQ;
                            end
                            PH_D1_IM: begin
                                d1_im_q <= fpu_rsp_result;
                                phase_q <= PH_M0_RE_A;
                                state   <= ST_FPU_REQ;
                            end
                            PH_M0_RE_A: begin
                                tmp_q   <= fpu_rsp_result;
                                phase_q <= PH_M0_RE_B;
                                state   <= ST_FPU_REQ;
                            end
                            PH_M0_RE_B: begin
                                m0_re_q <= fpu_rsp_result;
                                phase_q <= PH_M0_IM_A;
                                state   <= ST_FPU_REQ;
                            end
                            PH_M0_IM_A: begin
                                tmp_q   <= fpu_rsp_result;
                                phase_q <= PH_M0_IM_B;
                                state   <= ST_FPU_REQ;
                            end
                            PH_M0_IM_B: begin
                                m0_im_q <= fpu_rsp_result;
                                phase_q <= PH_M1_RE_A;
                                state   <= ST_FPU_REQ;
                            end
                            PH_M1_RE_A: begin
                                tmp_q   <= fpu_rsp_result;
                                phase_q <= PH_M1_RE_B;
                                state   <= ST_FPU_REQ;
                            end
                            PH_M1_RE_B: begin
                                m1_re_q <= fpu_rsp_result;
                                phase_q <= PH_M1_IM_A;
                                state   <= ST_FPU_REQ;
                            end
                            PH_M1_IM_A: begin
                                tmp_q   <= fpu_rsp_result;
                                phase_q <= PH_M1_IM_B;
                                state   <= ST_FPU_REQ;
                            end
                            PH_M1_IM_B: begin
                                m1_im_q <= fpu_rsp_result;
                                phase_q <= PH_SUM_RE;
                                state   <= ST_FPU_REQ;
                            end
                            PH_SUM_RE: begin
                                out_re_q <= fpu_rsp_result;
                                phase_q  <= PH_SUM_IM;
                                state    <= ST_FPU_REQ;
                            end
                            PH_SUM_IM: begin
                                out_im_q <= fpu_rsp_result;
                                state    <= ST_WR;
                            end
                            default: begin
                                status <= 8'hE2;
                                state  <= ST_FAIL;
                            end
                        endcase
                    end
                end

                ST_WR: begin
                    if (idx_q == (word_count - 1'b1)) begin
                        state <= ST_DONE;
                    end else begin
                        idx_q <= idx_q + 1'b1;
                        state <= ST_RD_T0;
                    end
                end

                ST_DONE: begin
                    done   <= 1'b1;
                    status <= 8'h00;
                    state  <= ST_IDLE;
                end

                ST_FAIL: begin
                    done  <= 1'b1;
                    fail  <= 1'b1;
                    state <= ST_IDLE;
                end

                default: begin
                    state <= ST_IDLE;
                end
            endcase
        end
    end

endmodule
