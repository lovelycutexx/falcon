`timescale 1ns/1ps

module falconsign_keccak_core(
    input  wire        clk,
    input  wire        start,
    output reg         ready,

    input  wire [63:0] di0, di1, di2, di3, di4,
                       di5, di6, di7, di8, di9,
                       di10,di11,di12,di13,di14,
                       di15,di16,di17,di18,di19,
                       di20,di21,di22,di23,di24,

    output reg  [63:0] do0, do1, do2, do3, do4,
                       do5, do6, do7, do8, do9,
                       do10,do11,do12,do13,do14,
                       do15,do16,do17,do18,do19,
                       do20,do21,do22,do23,do24
);

    reg [63:0] s [0:24];
    reg [4:0]  rnd;

    reg [63:0] a [0:24];
    reg [63:0] b [0:24];
    reg [63:0] c [0:4];
    reg [63:0] d [0:4];

    integer i;
    integer x;
    integer y;
    integer src_idx;
    integer dst_idx;

    initial begin
        rnd   = 5'd24;
        ready = 1'b1;
        for (i = 0; i < 25; i = i + 1)
            s[i] = 64'd0;
    end

    function [63:0] rotl64;
        input [63:0] v;
        input integer n;
        begin
            if (n == 0)
                rotl64 = v;
            else
                rotl64 = (v << n) | (v >> (64 - n));
        end
    endfunction

    function integer rho_offset;
        input integer xi;
        input integer yi;
        begin
            case (xi + 5 * yi)
                0:  rho_offset = 0;
                1:  rho_offset = 1;
                2:  rho_offset = 62;
                3:  rho_offset = 28;
                4:  rho_offset = 27;
                5:  rho_offset = 36;
                6:  rho_offset = 44;
                7:  rho_offset = 6;
                8:  rho_offset = 55;
                9:  rho_offset = 20;
                10: rho_offset = 3;
                11: rho_offset = 10;
                12: rho_offset = 43;
                13: rho_offset = 25;
                14: rho_offset = 39;
                15: rho_offset = 41;
                16: rho_offset = 45;
                17: rho_offset = 15;
                18: rho_offset = 21;
                19: rho_offset = 8;
                20: rho_offset = 18;
                21: rho_offset = 2;
                22: rho_offset = 61;
                23: rho_offset = 56;
                default: rho_offset = 14;
            endcase
        end
    endfunction

    function [63:0] round_const;
        input [4:0] r;
        begin
            case (r)
                5'd0 : round_const = 64'h0000000000000001;
                5'd1 : round_const = 64'h0000000000008082;
                5'd2 : round_const = 64'h800000000000808A;
                5'd3 : round_const = 64'h8000000080008000;
                5'd4 : round_const = 64'h000000000000808B;
                5'd5 : round_const = 64'h0000000080000001;
                5'd6 : round_const = 64'h8000000080008081;
                5'd7 : round_const = 64'h8000000000008009;
                5'd8 : round_const = 64'h000000000000008A;
                5'd9 : round_const = 64'h0000000000000088;
                5'd10: round_const = 64'h0000000080008009;
                5'd11: round_const = 64'h000000008000000A;
                5'd12: round_const = 64'h000000008000808B;
                5'd13: round_const = 64'h800000000000008B;
                5'd14: round_const = 64'h8000000000008089;
                5'd15: round_const = 64'h8000000000008003;
                5'd16: round_const = 64'h8000000000008002;
                5'd17: round_const = 64'h8000000000000080;
                5'd18: round_const = 64'h000000000000800A;
                5'd19: round_const = 64'h800000008000000A;
                5'd20: round_const = 64'h8000000080008081;
                5'd21: round_const = 64'h8000000000008080;
                5'd22: round_const = 64'h0000000080000001;
                5'd23: round_const = 64'h8000000080008008;
                default: round_const = 64'd0;
            endcase
        end
    endfunction

    always @(posedge clk) begin
        if (start) begin
            s[0]  <= di0;   s[1]  <= di1;   s[2]  <= di2;   s[3]  <= di3;   s[4]  <= di4;
            s[5]  <= di5;   s[6]  <= di6;   s[7]  <= di7;   s[8]  <= di8;   s[9]  <= di9;
            s[10] <= di10;  s[11] <= di11;  s[12] <= di12;  s[13] <= di13;  s[14] <= di14;
            s[15] <= di15;  s[16] <= di16;  s[17] <= di17;  s[18] <= di18;  s[19] <= di19;
            s[20] <= di20;  s[21] <= di21;  s[22] <= di22;  s[23] <= di23;  s[24] <= di24;
            rnd   <= 5'd0;
            ready <= 1'b0;
        end else if (rnd < 5'd24) begin
            for (i = 0; i < 25; i = i + 1)
                a[i] = s[i];

            for (x = 0; x < 5; x = x + 1)
                c[x] = a[x] ^ a[x + 5] ^ a[x + 10] ^ a[x + 15] ^ a[x + 20];

            for (x = 0; x < 5; x = x + 1)
                d[x] = c[(x + 4) % 5] ^ rotl64(c[(x + 1) % 5], 1);

            for (x = 0; x < 5; x = x + 1) begin
                for (y = 0; y < 5; y = y + 1) begin
                    src_idx = x + 5 * y;
                    dst_idx = y + 5 * ((2 * x + 3 * y) % 5);
                    b[dst_idx] = rotl64(a[src_idx] ^ d[x], rho_offset(x, y));
                end
            end

            for (x = 0; x < 5; x = x + 1) begin
                for (y = 0; y < 5; y = y + 1) begin
                    dst_idx = x + 5 * y;
                    s[dst_idx] <= b[dst_idx] ^ ((~b[((x + 1) % 5) + 5 * y]) & b[((x + 2) % 5) + 5 * y]);
                end
            end

            s[0] <= (b[0] ^ ((~b[1]) & b[2])) ^ round_const(rnd);
            rnd <= rnd + 1'b1;

            if (rnd == 5'd23)
                ready <= 1'b1;
        end
    end

    always @(*) begin
        do0  = s[0];   do1  = s[1];   do2  = s[2];   do3  = s[3];   do4  = s[4];
        do5  = s[5];   do6  = s[6];   do7  = s[7];   do8  = s[8];   do9  = s[9];
        do10 = s[10];  do11 = s[11];  do12 = s[12];  do13 = s[13];  do14 = s[14];
        do15 = s[15];  do16 = s[16];  do17 = s[17];  do18 = s[18];  do19 = s[19];
        do20 = s[20];  do21 = s[21];  do22 = s[22];  do23 = s[23];  do24 = s[24];
    end

endmodule
