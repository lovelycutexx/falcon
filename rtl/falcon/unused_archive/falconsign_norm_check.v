`timescale 1ns/1ps

module falconsign_norm_check #(
    parameter ADDR_W = 10
) (
    input  wire             clk,
    input  wire             rst_n,

    input  wire             start,
    output wire             start_ready,
    input  wire [ADDR_W-1:0] base_addr,
    input  wire [ADDR_W-1:0] word_count,
    input  wire [63:0]      bound_sq,

    output reg              mem_rd_en,
    output reg  [ADDR_W-1:0] mem_rd_addr,
    input  wire [255:0]     mem_rd_data,

    output reg              done,
    output reg              accept,
    output reg              fail,
    output reg  [7:0]       status,
    output reg  [63:0]      norm_sq
);

    localparam [2:0] ST_IDLE = 3'd0;
    localparam [2:0] ST_REQ  = 3'd1;
    localparam [2:0] ST_CAP  = 3'd2;
    localparam [2:0] ST_DONE = 3'd3;

    reg [2:0] state;
    reg [ADDR_W-1:0] idx_q;
    reg reject_q;

    reg signed [31:0] lane_i [0:3];
    reg [63:0] word_sum;
    reg sign_q;
    reg [10:0] exp_q;
    reg [51:0] frac_q;
    reg [63:0] mant_q;
    reg [63:0] mag_q;
    reg [31:0] abs_lane_q;
    integer sh_q;
    integer lane_idx_q;
    wire [63:0] next_norm_sq = norm_sq + word_sum;

    assign start_ready = (state == ST_IDLE);

    always @(*) begin
        word_sum = 64'd0;
        for (lane_idx_q = 0; lane_idx_q < 4; lane_idx_q = lane_idx_q + 1) begin
            sign_q = mem_rd_data[lane_idx_q*64 + 63];
            exp_q  = mem_rd_data[lane_idx_q*64 + 62 -: 11];
            frac_q = mem_rd_data[lane_idx_q*64 + 51 -: 52];
            mant_q = {12'd0, 1'b1, frac_q};
            sh_q = 0;

            if (exp_q == 11'd0) begin
                mag_q = 64'd0;
            end else if (exp_q < 11'd1023) begin
                mag_q = 64'd0;
            end else if (exp_q > 11'd1053) begin
                mag_q = 64'h000000007fffffff;
            end else begin
                sh_q = exp_q - 11'd1023;
                if (sh_q >= 52)
                    mag_q = mant_q << (sh_q - 52);
                else
                    mag_q = mant_q >> (52 - sh_q);
            end

            if (mag_q[63:31] != 33'd0)
                lane_i[lane_idx_q] = sign_q ? -32'sh7fffffff : 32'sh7fffffff;
            else
                lane_i[lane_idx_q] = sign_q ? -$signed(mag_q[31:0]) : $signed(mag_q[31:0]);

            abs_lane_q = lane_i[lane_idx_q][31] ? (~lane_i[lane_idx_q] + 1'b1) : lane_i[lane_idx_q];
            word_sum = word_sum + (abs_lane_q * abs_lane_q);
        end
    end

    always @(*) begin
        mem_rd_en   = 1'b0;
        mem_rd_addr = base_addr + idx_q;
        if (state == ST_REQ || state == ST_CAP)
            mem_rd_en = 1'b1;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state    <= ST_IDLE;
            idx_q    <= {ADDR_W{1'b0}};
            done     <= 1'b0;
            accept   <= 1'b0;
            fail     <= 1'b0;
            status   <= 8'h00;
            norm_sq  <= 64'd0;
            reject_q <= 1'b0;
        end else begin
            done <= 1'b0;
            fail <= 1'b0;

            case (state)
                ST_IDLE: begin
                    accept   <= 1'b0;
                    status   <= 8'h00;
                    reject_q <= 1'b0;
                    if (start) begin
                        idx_q   <= {ADDR_W{1'b0}};
                        norm_sq <= 64'd0;
                        if (word_count == {ADDR_W{1'b0}}) begin
                            accept <= 1'b0;
                            status <= 8'hE1;
                            fail   <= 1'b1;
                            done   <= 1'b1;
                            state  <= ST_IDLE;
                        end else begin
                            state <= ST_REQ;
                        end
                    end
                end

                ST_REQ: begin
                    state <= ST_CAP;
                end

                ST_CAP: begin
                    norm_sq <= next_norm_sq;
                    if (next_norm_sq > bound_sq)
                        reject_q <= 1'b1;

                    if (idx_q == (word_count - 1'b1)) begin
                        state <= ST_DONE;
                    end else begin
                        idx_q <= idx_q + 1'b1;
                        state <= ST_REQ;
                    end
                end

                ST_DONE: begin
                    done   <= 1'b1;
                    accept <= !reject_q && (norm_sq <= bound_sq);
                    status <= (!reject_q && (norm_sq <= bound_sq)) ? 8'h00 : 8'h01;
                    state  <= ST_IDLE;
                end

                default: begin
                    state <= ST_IDLE;
                end
            endcase
        end
    end

endmodule
