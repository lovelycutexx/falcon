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

    wire signed [31:0] lane0_i = f64_to_i32(mem_rd_data[ 63:  0]);
    wire signed [31:0] lane1_i = f64_to_i32(mem_rd_data[127: 64]);
    wire signed [31:0] lane2_i = f64_to_i32(mem_rd_data[191:128]);
    wire signed [31:0] lane3_i = f64_to_i32(mem_rd_data[255:192]);

    wire [63:0] lane0_sq = lane_square(lane0_i);
    wire [63:0] lane1_sq = lane_square(lane1_i);
    wire [63:0] lane2_sq = lane_square(lane2_i);
    wire [63:0] lane3_sq = lane_square(lane3_i);
    wire [63:0] word_sum = lane0_sq + lane1_sq + lane2_sq + lane3_sq;
    wire [63:0] next_norm_sq = norm_sq + word_sum;

    assign start_ready = (state == ST_IDLE);

    function signed [31:0] f64_to_i32;
        input [63:0] f;
        reg sign;
        reg [10:0] exp;
        reg [51:0] frac;
        reg [63:0] mant;
        reg [63:0] mag;
        integer sh;
        begin
            sign = f[63];
            exp  = f[62:52];
            frac = f[51:0];
            mant = {12'd0, 1'b1, frac};

            if (exp == 11'd0) begin
                mag = 64'd0;
            end else if (exp < 11'd1023) begin
                mag = 64'd0;
            end else if (exp > 11'd1053) begin
                mag = 64'h000000007fffffff;
            end else begin
                sh = exp - 11'd1023;
                if (sh >= 52)
                    mag = mant << (sh - 52);
                else
                    mag = mant >> (52 - sh);
            end

            if (mag[63:31] != 33'd0)
                f64_to_i32 = sign ? -32'sh7fffffff : 32'sh7fffffff;
            else
                f64_to_i32 = sign ? -$signed(mag[31:0]) : $signed(mag[31:0]);
        end
    endfunction

    function [63:0] lane_square;
        input signed [31:0] v;
        reg [31:0] abs_v;
        begin
            abs_v = v[31] ? (~v + 1'b1) : v;
            lane_square = abs_v * abs_v;
        end
    endfunction

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
