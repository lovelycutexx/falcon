`timescale 1ns/1ps

module falconsign_fpr_to_int16 #(
    parameter ADDR_W = 11
) (
    input  wire              clk,
    input  wire              rst_n,

    input  wire              start,
    output wire              start_ready,
    input  wire [ADDR_W-1:0] src_base,
    input  wire [ADDR_W-1:0] dst_base,
    input  wire [ADDR_W-1:0] coeff_count,

    output reg               mem_rd_en,
    output reg  [ADDR_W-1:0] mem_rd_addr,
    input  wire [255:0]      mem_rd_data,
    output reg               mem_wr_en,
    output reg  [ADDR_W-1:0] mem_wr_addr,
    output reg  [255:0]      mem_wr_data,

    output reg               done,
    output reg               fail,
    output reg  [7:0]        status
);

    localparam [2:0] ST_IDLE = 3'd0;
    localparam [2:0] ST_REQ  = 3'd1;
    localparam [2:0] ST_CAP  = 3'd2;
    localparam [2:0] ST_WR   = 3'd3;
    localparam [2:0] ST_DONE = 3'd4;

    reg [2:0] state;
    reg [ADDR_W-1:0] coeff_idx;
    reg [ADDR_W-1:0] word_idx;
    reg [3:0] lane_idx;
    reg [255:0] pack_word;
    reg last_coeff_q;

    wire signed [15:0] coeff_i16 = f64_to_i16(f64_neg(mem_rd_data[63:0]));
    wire last_lane = (lane_idx == 4'd15);
    wire last_coeff = (coeff_idx == (coeff_count - 1'b1));

    assign start_ready = (state == ST_IDLE);

    function signed [15:0] sat_i32_to_i16;
        input signed [31:0] v;
        begin
            if (v > 32'sd32767)
                sat_i32_to_i16 = 16'sh7fff;
            else if (v < -32'sd32768)
                sat_i32_to_i16 = -16'sd32768;
            else
                sat_i32_to_i16 = v[15:0];
        end
    endfunction

    function [63:0] f64_neg;
        input [63:0] v;
        begin
            f64_neg = (v[62:0] == 63'd0) ? v : {~v[63], v[62:0]};
        end
    endfunction

    function signed [15:0] f64_to_i16;
        input [63:0] f;
        reg sign;
        reg [10:0] exp;
        reg [51:0] frac;
        reg [63:0] int_part;
        reg guard;
        reg sticky;
        integer exp_unb;
        integer rsh;
        reg signed [31:0] rounded;
        begin
            sign = f[63];
            exp  = f[62:52];
            frac = f[51:0];
            int_part = 64'd0;
            guard = 1'b0;
            sticky = 1'b0;

            if (exp < 11'd1022) begin
                rounded = 32'sd0;
            end else if (exp == 11'd1022) begin
                // |f| is in [0.5, 1). Falcon uses fpr_rint(), i.e.
                // round-to-nearest with ties to even.  Exactly +/-0.5
                // rounds to 0; any larger magnitude rounds to +/-1.
                rounded = (frac == 52'd0) ? 32'sd0 :
                          (sign ? -32'sd1 : 32'sd1);
            end else if (exp >= 11'd1054) begin
                rounded = sign ? -32'sd32768 : 32'sd32767;
            end else begin
                exp_unb = exp - 11'd1023;
                if (exp_unb <= 52) begin
                    rsh = 52 - exp_unb;
                    int_part = (64'd1 << exp_unb) + (frac >> rsh);
                    guard = (rsh > 0) ? ((frac >> (rsh - 1)) & 1'b1) : 1'b0;
                    sticky = (rsh > 1) ? |(frac & ((64'd1 << (rsh - 1)) - 1'b1)) : 1'b0;
                end else begin
                    int_part = (64'd1 << exp_unb) | (frac << (exp_unb - 52));
                end

                if (guard && (int_part[0] || sticky))
                    int_part = int_part + 1'b1;

                if (int_part > 64'd2147483647)
                    rounded = sign ? -32'sd2147483648 : 32'sd2147483647;
                else
                    rounded = sign ? -$signed(int_part[31:0]) : $signed(int_part[31:0]);
            end

            f64_to_i16 = sat_i32_to_i16(rounded);
        end
    endfunction

    always @(*) begin
        mem_rd_en   = 1'b0;
        mem_rd_addr = src_base + coeff_idx;
        mem_wr_en   = 1'b0;
        mem_wr_addr = dst_base + word_idx;
        mem_wr_data = pack_word;

        if ((state == ST_REQ) || (state == ST_CAP))
            mem_rd_en = 1'b1;

        if (state == ST_WR) begin
            mem_wr_en = 1'b1;
            mem_wr_data = pack_word;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= ST_IDLE;
            coeff_idx    <= {ADDR_W{1'b0}};
            word_idx     <= {ADDR_W{1'b0}};
            lane_idx     <= 4'd0;
            pack_word    <= 256'd0;
            last_coeff_q <= 1'b0;
            done         <= 1'b0;
            fail         <= 1'b0;
            status       <= 8'h00;
        end else begin
            done <= 1'b0;
            fail <= 1'b0;

            case (state)
                ST_IDLE: begin
                    status <= 8'h00;
                    if (start) begin
                        coeff_idx    <= {ADDR_W{1'b0}};
                        word_idx     <= {ADDR_W{1'b0}};
                        lane_idx     <= 4'd0;
                        pack_word    <= 256'd0;
                        last_coeff_q <= 1'b0;
                        if (coeff_count == {ADDR_W{1'b0}}) begin
                            fail   <= 1'b1;
                            done   <= 1'b1;
                            status <= 8'hE2;
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
                    pack_word[lane_idx*16 +: 16] <= coeff_i16;
                    last_coeff_q <= last_coeff;

                    if (last_lane || last_coeff) begin
                        state <= ST_WR;
                    end else begin
                        coeff_idx <= coeff_idx + 1'b1;
                        lane_idx  <= lane_idx + 1'b1;
                        state     <= ST_REQ;
                    end
                end

                ST_WR: begin
                    pack_word <= 256'd0;
                    if (last_coeff_q) begin
                        state <= ST_DONE;
                    end else begin
                        coeff_idx <= coeff_idx + 1'b1;
                        word_idx  <= word_idx + 1'b1;
                        lane_idx  <= 4'd0;
                        state     <= ST_REQ;
                    end
                end

                ST_DONE: begin
                    done   <= 1'b1;
                    status <= 8'h00;
                    state  <= ST_IDLE;
                end

                default: begin
                    state <= ST_IDLE;
                end
            endcase
        end
    end

endmodule
