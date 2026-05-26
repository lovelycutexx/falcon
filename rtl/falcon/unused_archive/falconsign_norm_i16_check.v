`timescale 1ns/1ps

module falconsign_norm_i16_check #(
    parameter ADDR_W = 11
) (
    input  wire              clk,
    input  wire              rst_n,

    input  wire              start,
    output wire              start_ready,
    input  wire [ADDR_W-1:0] base_addr,
    input  wire [ADDR_W-1:0] word_count,
    input  wire [63:0]       bound_sq,

    output reg               mem_rd_en,
    output reg  [ADDR_W-1:0] mem_rd_addr,
    input  wire [255:0]      mem_rd_data,

    output reg               done,
    output reg               accept,
    output reg               fail,
    output reg  [7:0]        status,
    output reg  [63:0]       norm_sq
);

    localparam [2:0] ST_IDLE = 3'd0;
    localparam [2:0] ST_REQ  = 3'd1;
    localparam [2:0] ST_CAP  = 3'd2;
    localparam [2:0] ST_DONE = 3'd3;

    reg [2:0] state;
    reg [ADDR_W-1:0] idx_q;
    reg reject_q;

    wire [63:0] word_sum = packed_signed_square_sum(mem_rd_data);
    wire [63:0] next_norm_sq = norm_sq + word_sum;

    assign start_ready = (state == ST_IDLE);

    function [63:0] lane_square;
        input signed [15:0] v;
        reg [15:0] abs_v;
        begin
            abs_v = v[15] ? (~v + 1'b1) : v;
            lane_square = abs_v * abs_v;
        end
    endfunction

    function [63:0] packed_signed_square_sum;
        input [255:0] w;
        integer i;
        reg [63:0] acc;
        reg signed [15:0] lane;
        begin
            acc = 64'd0;
            for (i = 0; i < 16; i = i + 1) begin
                lane = w[i*16 +: 16];
                acc = acc + lane_square(lane);
            end
            packed_signed_square_sum = acc;
        end
    endfunction

    always @(*) begin
        mem_rd_en   = 1'b0;
        mem_rd_addr = base_addr + idx_q;
        if ((state == ST_REQ) || (state == ST_CAP))
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
