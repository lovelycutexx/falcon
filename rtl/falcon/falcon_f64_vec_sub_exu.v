`timescale 1ns/1ps

// Vector complex subtraction over FP64 words.
// Each 256-bit memory word carries one complex value in [63:0] real,
// [127:64] imag.  The upper half is cleared on writeback.
module falcon_f64_vec_sub_exu #(
    parameter ADDR_W = 11
) (
    input  wire              clk,
    input  wire              rst_n,

    input  wire              start,
    output wire              start_ready,
    input  wire [ADDR_W-1:0] src_a_base,
    input  wire [ADDR_W-1:0] src_b_base,
    input  wire [ADDR_W-1:0] dst_base,
    input  wire [ADDR_W-1:0] word_count,

    output reg               mem_rd_en,
    output reg  [ADDR_W-1:0] mem_rd_addr,
    input  wire [255:0]      mem_rd_data,
    output reg               mem_wr_en,
    output reg  [ADDR_W-1:0] mem_wr_addr,
    output reg  [255:0]      mem_wr_data,

    output reg               fpu_req_valid,
    input  wire              fpu_req_ready,
    output wire [3:0]        fpu_req_op,
    output reg  [63:0]       fpu_req_a,
    output reg  [63:0]       fpu_req_b,
    output wire [63:0]       fpu_req_c,
    input  wire              fpu_rsp_valid,
    input  wire [63:0]       fpu_rsp_result,

    output reg               done,
    output reg               fail,
    output reg  [7:0]        status
);

    localparam [2:0] ST_IDLE    = 3'd0;
    localparam [2:0] ST_RD_A    = 3'd1;
    localparam [2:0] ST_CAP_A   = 3'd2;
    localparam [2:0] ST_RD_B    = 3'd3;
    localparam [2:0] ST_CAP_B   = 3'd4;
    localparam [2:0] ST_FPU_REQ = 3'd5;
    localparam [2:0] ST_FPU_RSP = 3'd6;
    localparam [2:0] ST_WR      = 3'd7;

    localparam [3:0] FSUB = 4'd1;

    reg [2:0] state;
    reg [ADDR_W-1:0] idx_q;
    reg lane_q;
    reg [63:0] a_re_q, a_im_q;
    reg [63:0] b_re_q, b_im_q;
    reg [63:0] diff_re_q, diff_im_q;

    assign start_ready = (state == ST_IDLE);
    assign fpu_req_op  = FSUB;
    assign fpu_req_c   = 64'd0;

    always @(*) begin
        mem_rd_en    = 1'b0;
        mem_rd_addr  = src_a_base + idx_q;
        mem_wr_en    = 1'b0;
        mem_wr_addr  = dst_base + idx_q;
        mem_wr_data  = {128'd0, diff_im_q, diff_re_q};
        fpu_req_valid = 1'b0;
        fpu_req_a     = lane_q ? a_im_q : a_re_q;
        fpu_req_b     = lane_q ? b_im_q : b_re_q;

        case (state)
            ST_RD_A: begin
                mem_rd_en   = 1'b1;
                mem_rd_addr = src_a_base + idx_q;
            end
            ST_RD_B: begin
                mem_rd_en   = 1'b1;
                mem_rd_addr = src_b_base + idx_q;
            end
            ST_FPU_REQ: begin
                fpu_req_valid = 1'b1;
            end
            ST_WR: begin
                mem_wr_en   = 1'b1;
                mem_wr_addr = dst_base + idx_q;
                mem_wr_data = {128'd0, diff_im_q, diff_re_q};
            end
            default: begin
            end
        endcase
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= ST_IDLE;
            idx_q     <= {ADDR_W{1'b0}};
            lane_q    <= 1'b0;
            a_re_q    <= 64'd0;
            a_im_q    <= 64'd0;
            b_re_q    <= 64'd0;
            b_im_q    <= 64'd0;
            diff_re_q <= 64'd0;
            diff_im_q <= 64'd0;
            done      <= 1'b0;
            fail      <= 1'b0;
            status    <= 8'h00;
        end else begin
            done <= 1'b0;
            fail <= 1'b0;

            case (state)
                ST_IDLE: begin
                    status <= 8'h00;
                    if (start) begin
                        idx_q  <= {ADDR_W{1'b0}};
                        lane_q <= 1'b0;
                        if (word_count == {ADDR_W{1'b0}}) begin
                            status <= 8'hE1;
                            fail   <= 1'b1;
                            done   <= 1'b1;
                            state  <= ST_IDLE;
                        end else begin
                            state <= ST_RD_A;
                        end
                    end
                end

                ST_RD_A: begin
                    state <= ST_CAP_A;
                end

                ST_CAP_A: begin
                    a_re_q <= mem_rd_data[63:0];
                    a_im_q <= mem_rd_data[127:64];
                    state  <= ST_RD_B;
                end

                ST_RD_B: begin
                    state <= ST_CAP_B;
                end

                ST_CAP_B: begin
                    b_re_q <= mem_rd_data[63:0];
                    b_im_q <= mem_rd_data[127:64];
                    lane_q <= 1'b0;
                    state  <= ST_FPU_REQ;
                end

                ST_FPU_REQ: begin
                    if (fpu_req_ready)
                        state <= ST_FPU_RSP;
                end

                ST_FPU_RSP: begin
                    if (fpu_rsp_valid) begin
                        if (!lane_q) begin
                            diff_re_q <= fpu_rsp_result;
                            lane_q    <= 1'b1;
                            state     <= ST_FPU_REQ;
                        end else begin
                            diff_im_q <= fpu_rsp_result;
                            state     <= ST_WR;
                        end
                    end
                end

                ST_WR: begin
                    if (idx_q == (word_count - 1'b1)) begin
                        done  <= 1'b1;
                        state <= ST_IDLE;
                    end else begin
                        idx_q <= idx_q + 1'b1;
                        state <= ST_RD_A;
                    end
                end

                default: begin
                    state <= ST_IDLE;
                end
            endcase
        end
    end

endmodule
