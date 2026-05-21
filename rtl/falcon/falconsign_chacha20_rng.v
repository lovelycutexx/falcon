`timescale 1ns/1ps

module falconsign_chacha20_rng #(
    parameter KEY_W   = 256,
    parameter BLOCK_W = 512
) (
    input  wire        clk,
    input  wire        rst_n,

    input  wire        seed_valid,
    output wire        seed_ready,
    input  wire [KEY_W-1:0] seed_key,
    input  wire [95:0]       seed_nonce,

    output reg         rng_valid,
    input  wire        rng_ready,
    output reg  [BLOCK_W-1:0] rng_block,

    output wire        busy
);

    localparam [3:0] S_IDLE    = 4'd0;
    localparam [3:0] S_LOAD    = 4'd1;
    localparam [3:0] S_DROUND  = 4'd2;
    localparam [3:0] S_FINAL   = 4'd3;
    localparam [3:0] S_DONE    = 4'd4;

    reg [3:0] state, state_next;

    // ChaCha20 16 x 32-bit working state
    reg [31:0] st [0:15];
    reg [31:0] orig [0:15];
    reg [4:0]  dround_cnt;  // 0..9 double rounds
    reg [31:0] block_ctr;
    reg        seeded;

    integer i;

    assign seed_ready = (state == S_IDLE);
    assign busy       = (state != S_IDLE) && (state != S_DONE);

    // Quarter round on four state elements (combinational)
    function [127:0] qr4;
        input [127:0] in_abcd;
        reg [31:0] a, b, c, d;
        begin
            a = in_abcd[127:96];
            b = in_abcd[ 95:64];
            c = in_abcd[ 63:32];
            d = in_abcd[ 31: 0];

            a = a + b;  d = d ^ a;  d = {d[15:0], d[31:16]};
            c = c + d;  b = b ^ c;  b = {b[19:0], b[31:20]};
            a = a + b;  d = d ^ a;  d = {d[23:0], d[31:24]};
            c = c + d;  b = b ^ c;  b = {b[24:0], b[31:25]};

            qr4 = {a, b, c, d};
        end
    endfunction

    // One double round = column round + diagonal round
    // Modifies all 16 state words
    function [511:0] double_round;
        input [511:0] state_in;
        reg [31:0] x0, x1, x2, x3, x4, x5, x6, x7;
        reg [31:0] x8, x9, x10, x11, x12, x13, x14, x15;
        reg [127:0] qr_in, qr_out;
        begin
            {x0,x1,x2,x3,x4,x5,x6,x7,
             x8,x9,x10,x11,x12,x13,x14,x15} = state_in;

            // Column round: (0,4,8,12), (1,5,9,13), (2,6,10,14), (3,7,11,15)
            qr_in  = {x0, x4, x8,  x12}; qr_out = qr4(qr_in);
            {x0, x4, x8,  x12} = qr_out;
            qr_in  = {x1, x5, x9,  x13}; qr_out = qr4(qr_in);
            {x1, x5, x9,  x13} = qr_out;
            qr_in  = {x2, x6, x10, x14}; qr_out = qr4(qr_in);
            {x2, x6, x10, x14} = qr_out;
            qr_in  = {x3, x7, x11, x15}; qr_out = qr4(qr_in);
            {x3, x7, x11, x15} = qr_out;

            // Diagonal round: (0,5,10,15), (1,6,11,12), (2,7,8,13), (3,4,9,14)
            qr_in  = {x0, x5, x10, x15}; qr_out = qr4(qr_in);
            {x0, x5, x10, x15} = qr_out;
            qr_in  = {x1, x6, x11, x12}; qr_out = qr4(qr_in);
            {x1, x6, x11, x12} = qr_out;
            qr_in  = {x2, x7, x8,  x13}; qr_out = qr4(qr_in);
            {x2, x7, x8,  x13} = qr_out;
            qr_in  = {x3, x4, x9,  x14}; qr_out = qr4(qr_in);
            {x3, x4, x9,  x14} = qr_out;

            double_round = {x0,x1,x2,x3,x4,x5,x6,x7,
                            x8,x9,x10,x11,x12,x13,x14,x15};
        end
    endfunction

    wire [511:0] st_packed = {
        st[0],  st[1],  st[2],  st[3],
        st[4],  st[5],  st[6],  st[7],
        st[8],  st[9],  st[10], st[11],
        st[12], st[13], st[14], st[15]
    };

    wire [511:0] dr_result = double_round(st_packed);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state       <= S_IDLE;
            dround_cnt  <= 5'd0;
            block_ctr   <= 32'd0;
            seeded      <= 1'b0;
            rng_valid   <= 1'b0;
            rng_block   <= 512'd0;
            for (i = 0; i < 16; i = i + 1) st[i]   <= 32'd0;
            for (i = 0; i < 16; i = i + 1) orig[i] <= 32'd0;
        end else begin
            state <= state_next;

            case (state)
                S_IDLE: begin
                    rng_valid <= 1'b0;
                    if (seed_valid && seed_ready) begin
                        seeded <= 1'b1;
                        block_ctr <= 32'd0;

                        orig[0]  <= 32'h61707865;
                        orig[1]  <= 32'h3320646e;
                        orig[2]  <= 32'h79622d32;
                        orig[3]  <= 32'h6b206574;
                        orig[4]  <= seed_key[31:0];
                        orig[5]  <= seed_key[63:32];
                        orig[6]  <= seed_key[95:64];
                        orig[7]  <= seed_key[127:96];
                        orig[8]  <= seed_key[159:128];
                        orig[9]  <= seed_key[191:160];
                        orig[10] <= seed_key[223:192];
                        orig[11] <= seed_key[255:224];
                        orig[12] <= 32'd0;
                        orig[13] <= seed_nonce[31:0];
                        orig[14] <= seed_nonce[63:32];
                        orig[15] <= seed_nonce[95:64];

                        st[0]  <= 32'h61707865;
                        st[1]  <= 32'h3320646e;
                        st[2]  <= 32'h79622d32;
                        st[3]  <= 32'h6b206574;
                        st[4]  <= seed_key[31:0];
                        st[5]  <= seed_key[63:32];
                        st[6]  <= seed_key[95:64];
                        st[7]  <= seed_key[127:96];
                        st[8]  <= seed_key[159:128];
                        st[9]  <= seed_key[191:160];
                        st[10] <= seed_key[223:192];
                        st[11] <= seed_key[255:224];
                        st[12] <= 32'd0;
                        st[13] <= seed_nonce[31:0];
                        st[14] <= seed_nonce[63:32];
                        st[15] <= seed_nonce[95:64];
                        dround_cnt <= 5'd0;
                    end else if (seeded && rng_ready) begin
                        for (i = 0; i < 16; i = i + 1) st[i] <= orig[i];
                        st[12]   <= block_ctr;
                        orig[12] <= block_ctr;
                        dround_cnt <= 5'd0;
                    end
                end

                S_LOAD: begin
                    // st[] already initialized from S_IDLE
                    dround_cnt <= 5'd1;
                end

                S_DROUND: begin
                    // Apply double_round result, increment counter
                    {st[0],st[1],st[2],st[3],st[4],st[5],st[6],st[7],
                     st[8],st[9],st[10],st[11],st[12],st[13],st[14],st[15]} <= dr_result;
                    if (dround_cnt == 5'd10) begin
                        // done with 10 double rounds
                    end else begin
                        dround_cnt <= dround_cnt + 5'd1;
                    end
                end

                S_FINAL: begin
                    // Add original state
                    for (i = 0; i < 16; i = i + 1) st[i] <= st[i] + orig[i];
                end

                S_DONE: begin
                    rng_block <= {
                        st[15], st[14], st[13], st[12],
                        st[11], st[10], st[ 9], st[ 8],
                        st[ 7], st[ 6], st[ 5], st[ 4],
                        st[ 3], st[ 2], st[ 1], st[ 0]
                    };
                    rng_valid  <= 1'b1;
                    if (rng_valid && rng_ready) begin
                        rng_valid <= 1'b0;
                        block_ctr <= block_ctr + 32'd1;
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

    always @(*) begin
        state_next = state;
        case (state)
            S_IDLE:   if ((seed_valid && seed_ready) || (seeded && rng_ready)) state_next = S_LOAD;
            S_LOAD:                                state_next = S_DROUND;
            S_DROUND: if (dround_cnt == 5'd10)     state_next = S_FINAL;
            S_FINAL:                               state_next = S_DONE;
            S_DONE:   if (rng_valid && rng_ready)  state_next = S_IDLE;
            default:                               state_next = S_IDLE;
        endcase
    end

endmodule
