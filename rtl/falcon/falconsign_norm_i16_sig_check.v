`timescale 1ns/1ps

// Falcon signature norm check for packed int16 buffers.
// s2 is stored as signed int16; s1 is stored modulo q and center-lifted here.
module falconsign_norm_i16_sig_check #(
    parameter ADDR_W = 11
) (
    input  wire              clk,
    input  wire              rst_n,

    input  wire              start,
    output wire              start_ready,
    input  wire [ADDR_W-1:0] s2_base,
    input  wire [ADDR_W-1:0] s1_base,
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

    localparam signed [15:0] Q_I16      = 16'sd12289;
    localparam        [15:0] HALF_Q_U16 = 16'd6144;

    reg [2:0] state;
    reg [ADDR_W-1:0] idx_q;
    reg check_s1_q;
    reg reject_q;

    reg [63:0] word_sum_s2;
    reg [63:0] word_sum_s1;
    reg signed [15:0] sum_lane;
    reg [15:0] sum_abs;
    reg [15:0] sum_center_u;
    integer sum_i;

    wire [63:0] word_sum    = check_s1_q ? word_sum_s1 : word_sum_s2;
    wire [63:0] next_norm_sq = norm_sq + word_sum;

    assign start_ready = (state == ST_IDLE);

    always @(*) begin
        word_sum_s2 = 64'd0;
        word_sum_s1 = 64'd0;
        sum_lane = 16'sd0;
        sum_abs = 16'd0;
        sum_center_u = 16'd0;

        for (sum_i = 0; sum_i < 16; sum_i = sum_i + 1) begin
            sum_lane = mem_rd_data[sum_i*16 +: 16];
            sum_abs = sum_lane[15] ? (~sum_lane + 1'b1) : sum_lane;
            word_sum_s2 = word_sum_s2 + (sum_abs * sum_abs);

            sum_center_u = mem_rd_data[sum_i*16 +: 16];
            sum_lane = (sum_center_u > HALF_Q_U16) ? ($signed({1'b0, sum_center_u}) - Q_I16) :
                                                     $signed({1'b0, sum_center_u});
            sum_abs = sum_lane[15] ? (~sum_lane + 1'b1) : sum_lane;
            word_sum_s1 = word_sum_s1 + (sum_abs * sum_abs);
        end
    end

    always @(*) begin
        mem_rd_en = 1'b0;
        mem_rd_addr = (check_s1_q ? s1_base : s2_base) + idx_q;

        if ((state == ST_REQ) || (state == ST_CAP))
            mem_rd_en = 1'b1;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= ST_IDLE;
            idx_q      <= {ADDR_W{1'b0}};
            check_s1_q <= 1'b0;
            reject_q   <= 1'b0;
            done       <= 1'b0;
            accept     <= 1'b0;
            fail       <= 1'b0;
            status     <= 8'h00;
            norm_sq    <= 64'd0;
        end else begin
            done <= 1'b0;
            fail <= 1'b0;

            case (state)
                ST_IDLE: begin
                    accept     <= 1'b0;
                    status     <= 8'h00;
                    reject_q   <= 1'b0;
                    check_s1_q <= 1'b0;
                    if (start) begin
                        idx_q   <= {ADDR_W{1'b0}};
                        norm_sq <= 64'd0;
                        if (word_count == {ADDR_W{1'b0}}) begin
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
                        if (!check_s1_q) begin
                            check_s1_q <= 1'b1;
                            idx_q      <= {ADDR_W{1'b0}};
                            state      <= ST_REQ;
                        end else begin
                            state <= ST_DONE;
                        end
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
