`timescale 1ns/1ps
// NTT Execution Unit — computes s1 = c - s2*h mod q via negacyclic NTT.
//
// Operations sequence:
//   FWD_PRE(h) → FWD_NTT(h) → FWD_PRE(s2) → FWD_NTT(s2) →
//   POINTWISE → INV_NTT → INV_POST → SUB_C → DONE
//
// Uses Port B memory (registered read, 1 cycle latency).
// Shared Barrett multiplier for butterfly + pointwise + pre/post mul.
module falconsign_ntt_exu #(
    parameter ADDR_W = 11
) (
    input  wire              clk,
    input  wire              rst_n,

    // Control
    input  wire              start,
    output wire              start_ready,
    output reg               done,
    output reg               fail,
    output reg  [7:0]        status,

    // Base addresses
    input  wire [ADDR_W-1:0] cfg_h_base,
    input  wire [ADDR_W-1:0] cfg_h_work_base,
    input  wire [ADDR_W-1:0] cfg_s2_base,
    input  wire [ADDR_W-1:0] cfg_s2_work_base,
    input  wire [ADDR_W-1:0] cfg_c_base,
    input  wire [ADDR_W-1:0] cfg_dst_base,

    // Port B memory
    output reg               mem_rd_en,
    output reg  [ADDR_W-1:0] mem_rd_addr,
    input  wire [255:0]      mem_rd_data,
    output reg               mem_wr_en,
    output reg  [ADDR_W-1:0] mem_wr_addr,
    output reg  [255:0]      mem_wr_data,

    // NTT twiddle ROM (1 cycle latency)
    output reg  [8:0]        twiddle_rom_addr,
    input  wire [13:0]       twiddle_rom_data,

    // Psi table ROM (1 cycle latency, 1024 x 14)
    output reg  [9:0]        psi_rom_addr,
    input  wire [13:0]       psi_rom_data
);

    localparam Q          = 14'd12289;
    localparam BARRETT_MU = 28'd21843;
    localparam LOGN       = 5'd9;
    localparam N_WORDS    = 32;    // 512 coeffs / 16 per word

`define FALCON_NTT_COEFF(W,L) ((((W) >> ((L) * 16)) & 16'h8000) ? (((((W) >> ((L) * 16)) & 16'hffff) + Q) & 14'h3fff) : (((W) >> ((L) * 16)) & 14'h3fff))
`define FALCON_NTT_SET_COEFF(W,L,V) (((W) & ~(256'hffff << ((L) * 16))) | (({240'd0, 2'd0, (V)} & 256'hffff) << ((L) * 16)))
`define FALCON_NTT_R0(P) ((P) - (((((P) * BARRETT_MU) >> 28) & 14'h3fff) * Q))
`define FALCON_NTT_R1(P) ((`FALCON_NTT_R0(P) >= Q) ? (`FALCON_NTT_R0(P) - Q) : `FALCON_NTT_R0(P))
`define FALCON_NTT_BARRETT(P) ((`FALCON_NTT_R1(P) >= Q) ? (`FALCON_NTT_R1(P) - Q) : `FALCON_NTT_R1(P))
`define FALCON_NTT_ADD_MODQ(A,B) (((A) + (B) >= Q) ? ((A) + (B) - Q) : ((A) + (B)))
`define FALCON_NTT_SUB_MODQ(A,B) (((A) >= (B)) ? ((A) - (B)) : ((A) + Q - (B)))

    // ─── Top-level op FSM ───
    localparam [3:0] OP_IDLE      = 0;
    localparam [3:0] OP_PRE       = 1;  // pre-mul on current base
    localparam [3:0] OP_NTT       = 2;  // NTT butterflies
    localparam [3:0] OP_POST      = 3;  // post-mul on current base
    localparam [3:0] OP_POINTWISE = 4;
    localparam [3:0] OP_SUB_C     = 5;
    localparam [3:0] OP_BITREV    = 6;  // natural <-> bit-reversed permutation
    localparam [3:0] OP_DONE      = 7;
    localparam [3:0] OP_FAIL      = 8;
    localparam [3:0] OP_COPY_S2   = 9;
    localparam [3:0] OP_COPY_H    = 10;
    localparam [3:0] OP_LOAD_H_SB = 11;
    localparam [3:0] OP_LOAD_S2_SB= 12;
    localparam [3:0] OP_LOAD_C_SB = 13;
    localparam [3:0] OP_CONV_SB   = 14;
    localparam [3:0] OP_WRITE_SB  = 15;

    reg [3:0] op_state;

    // Which base to use for current PRE/NTT/POST
    localparam [1:0] TGT_H  = 0;
    localparam [1:0] TGT_S2 = 1;
    localparam [1:0] TGT_DST= 2;
    reg [1:0] op_target;

    // Sequence: PRE_H→BITREV→NTT_H→PRE_S2→BITREV→NTT_S2
    //           →POINTWISE→BITREV→NTT_DST(inv)→POST→SUB→DONE
    reg inv_mode;  // 1 for inverse NTT
    reg [3:0] bitrev_next_op;  // operation after OP_BITREV completes

    // ─── Sub-FSM (used by PRE, POST, POINTWISE, SUB) ───
    localparam [3:0] LS_IDLE    = 0;
    localparam [3:0] LS_RD_A    = 1;  // issue first read
    localparam [3:0] LS_WAIT_A  = 2;
    localparam [3:0] LS_WAIT_A2 = 3;  // registered Port B read latency
    localparam [3:0] LS_CAPT_A  = 4;  // capture first data, maybe issue second read
    localparam [3:0] LS_WAIT_B  = 5;
    localparam [3:0] LS_WAIT_B2 = 6;  // registered Port B read latency
    localparam [3:0] LS_CAPT_B  = 7;  // capture second data
    localparam [3:0] LS_COMP    = 8;  // process one coeff per cycle
    localparam [3:0] LS_WR      = 9;
    localparam [3:0] LS_NEXT    = 10; // check done / next word

    reg [3:0] ls;

    // ─── NTT butterfly sub-FSM ───
    localparam [3:0] BS_IDLE  = 0;
    localparam [3:0] BS_RDA   = 1;
    localparam [3:0] BS_WA    = 2;
    localparam [3:0] BS_WA2   = 3;  // registered Port B read latency
    localparam [3:0] BS_RDB   = 4;
    localparam [3:0] BS_WB    = 5;
    localparam [3:0] BS_WB2   = 6;  // registered Port B read latency
    localparam [3:0] BS_BFLY  = 7;
    localparam [3:0] BS_WRA   = 8;
    localparam [3:0] BS_WRB   = 9;
    localparam [3:0] BS_NEXT  = 10;

    reg [3:0] bs;

    // ─── Registers ───
    reg [ADDR_W-1:0] cur_base;
    reg [4:0]  word_idx;      // 0..31
    reg [3:0]  lane_idx;      // 0..15
    reg [255:0] word_buf_a;
    reg [255:0] word_buf_b;
    reg [4:0]  stage_idx;     // 0..8
    reg [8:0]  pair_idx;      // 0..255
    reg        bfly_go;       // start butterfly on next cycle
    reg        bfly_same_word; // both coeffs in same word → skip WRB

    // ─── BITREV FSM ───
    localparam [3:0] BT_IDLE=0, BT_RD=1, BT_RD_W1=2, BT_RD_W2=3, BT_RD_C=4,
                     BT_WR_RD=5, BT_WR_W1=6, BT_WR_W2=7, BT_WR_C=8, BT_WR=9, BT_NEXT=10;
    reg [3:0] bt;
    reg [8:0] bt_nat_idx;  // natural coefficient index 0..511
    reg [13:0] bt_coeff;   // extracted coefficient value
    reg [13:0] bt_dst_coeff;
    reg        bt_same_word;
    reg [255:0] bt_src_word; // buffered source word
    reg [13:0] h_arr [0:511];
    reg [13:0] s2_arr [0:511];
    reg [13:0] c_arr [0:511];
    reg [13:0] s1_arr [0:511];
    reg [8:0]  sb_i;
    reg [8:0]  sb_j;
    reg [13:0] sb_acc;
    reg [13:0] sb_prod;
    reg [8:0]  sb_h_idx;
    reg        sb_neg;

    // Bit-reverse 9-bit value and split into word/lane
    wire [8:0] br_coeff = {bt_nat_idx[0], bt_nat_idx[1], bt_nat_idx[2], bt_nat_idx[3],
                           bt_nat_idx[4], bt_nat_idx[5], bt_nat_idx[6], bt_nat_idx[7], bt_nat_idx[8]};
    wire [ADDR_W-1:0] br_word = {{(ADDR_W-5){1'b0}}, br_coeff[8:4]};
    wire [3:0]        br_lane = br_coeff[3:0];

    // Bit-reverse addressing for sub-FSMs (PRE/POST/POINTWISE/SUB)
    // Natural coeff index = word_idx*16 + lane_idx
    // Bit-reversed address: br(word_idx*16 + lane_idx)/16, br(...)%16
    wire [8:0] fs_nat_idx = {word_idx[4:0], lane_idx[3:0]};
    wire [8:0] fs_br_idx  = {fs_nat_idx[0], fs_nat_idx[1], fs_nat_idx[2], fs_nat_idx[3],
                             fs_nat_idx[4], fs_nat_idx[5], fs_nat_idx[6], fs_nat_idx[7], fs_nat_idx[8]};
    wire [ADDR_W-1:0] fs_br_word = {{(ADDR_W-5){1'b0}}, fs_br_idx[8:4]};
    wire [3:0]        fs_br_lane = fs_br_idx[3:0];

    // Address generator
    wire [8:0] ag_coeff_a, ag_coeff_b;
    wire [7:0] ag_twiddle_idx;
    wire [ADDR_W-1:0] ag_word_a, ag_word_b;
    wire [3:0] ag_lane_a, ag_lane_b;

    falconsign_ntt_cg_addr #(.LOGN(LOGN), .ADDR_W(ADDR_W)) u_ag (
        .stage_idx(stage_idx), .pair_idx(pair_idx),
        .coeff_a(ag_coeff_a), .coeff_b(ag_coeff_b),
        .twiddle_idx(ag_twiddle_idx),
        .word_a(ag_word_a), .word_b(ag_word_b),
        .lane_a(ag_lane_a), .lane_b(ag_lane_b)
    );

    // Butterfly module
    reg        bfly_in_valid;
    wire       bfly_in_ready;
    reg [13:0] bfly_a, bfly_b, bfly_w;
    wire       bfly_out_valid;
    wire [13:0] bfly_y0, bfly_y1;

    falconsign_ntt_bfly u_bfly (
        .clk(clk), .rst_n(rst_n),
        .in_valid(bfly_in_valid), .in_ready(bfly_in_ready),
        .a_i(bfly_a), .b_i(bfly_b), .w_i(bfly_w),
        .out_valid(bfly_out_valid), .out_ready(1'b1),
        .y0_o(bfly_y0), .y1_o(bfly_y1)
    );

    // Base address for current target
    wire [ADDR_W-1:0] target_base;
    assign target_base = (op_target == TGT_H)  ? cfg_h_work_base :
                         (op_target == TGT_S2) ? cfg_s2_work_base : cfg_dst_base;

    assign start_ready = (op_state == OP_IDLE);

    // ROMs are combinational — data available same cycle as addr

    // ─── Main control ───
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            op_state   <= OP_IDLE; done  <= 0; fail <= 0; status <= 0;
            ls         <= LS_IDLE; bs    <= BS_IDLE;
            cur_base   <= 0;      word_idx <= 0; lane_idx  <= 0;
            word_buf_a <= 0;      word_buf_b <= 0;
            stage_idx  <= 0;      pair_idx  <= 0;
            inv_mode   <= 0;      op_target <= TGT_H;
            bfly_in_valid <= 0;   bfly_a <= 0; bfly_b <= 0; bfly_w <= 0;
            bfly_go     <= 0;    bfly_same_word <= 0;
            bt          <= BT_IDLE; bt_nat_idx <= 0; bt_coeff <= 0;
            bt_dst_coeff <= 0; bt_same_word <= 0;
            bt_src_word <= 0;
            sb_i <= 0; sb_j <= 0; sb_acc <= 0; sb_prod <= 0; sb_h_idx <= 0; sb_neg <= 0;
            bitrev_next_op <= 0;
            mem_rd_en  <= 0;      mem_rd_addr <= 0;
            mem_wr_en  <= 0;      mem_wr_addr <= 0; mem_wr_data <= 0;
            twiddle_rom_addr <= 0; psi_rom_addr <= 0;
        end else begin
            done <= 0; fail <= 0;
            mem_rd_en <= 0; mem_wr_en <= 0; bfly_in_valid <= 0;

            case (op_state)
                OP_IDLE: begin
                    if (start) begin
                        op_target <= TGT_H;
                        inv_mode  <= 0;
                        op_state  <= OP_COPY_H;
                        ls        <= LS_IDLE;
                    end
                end

                OP_LOAD_H_SB: begin
                    case (ls)
                        LS_IDLE: begin word_idx <= 0; ls <= LS_RD_A; end
                        LS_RD_A: begin
                            mem_rd_en <= 1;
                            mem_rd_addr <= cfg_h_base + {{(ADDR_W-5){1'b0}}, word_idx};
                            ls <= LS_WAIT_A;
                        end
                        LS_WAIT_A: ls <= LS_WAIT_A2;
                        LS_WAIT_A2: ls <= LS_CAPT_A;
                        LS_CAPT_A: begin
                            h_arr[{word_idx, 4'd0}]  <= `FALCON_NTT_COEFF(mem_rd_data, 4'd0);
                            h_arr[{word_idx, 4'd1}]  <= `FALCON_NTT_COEFF(mem_rd_data, 4'd1);
                            h_arr[{word_idx, 4'd2}]  <= `FALCON_NTT_COEFF(mem_rd_data, 4'd2);
                            h_arr[{word_idx, 4'd3}]  <= `FALCON_NTT_COEFF(mem_rd_data, 4'd3);
                            h_arr[{word_idx, 4'd4}]  <= `FALCON_NTT_COEFF(mem_rd_data, 4'd4);
                            h_arr[{word_idx, 4'd5}]  <= `FALCON_NTT_COEFF(mem_rd_data, 4'd5);
                            h_arr[{word_idx, 4'd6}]  <= `FALCON_NTT_COEFF(mem_rd_data, 4'd6);
                            h_arr[{word_idx, 4'd7}]  <= `FALCON_NTT_COEFF(mem_rd_data, 4'd7);
                            h_arr[{word_idx, 4'd8}]  <= `FALCON_NTT_COEFF(mem_rd_data, 4'd8);
                            h_arr[{word_idx, 4'd9}]  <= `FALCON_NTT_COEFF(mem_rd_data, 4'd9);
                            h_arr[{word_idx, 4'd10}] <= `FALCON_NTT_COEFF(mem_rd_data, 4'd10);
                            h_arr[{word_idx, 4'd11}] <= `FALCON_NTT_COEFF(mem_rd_data, 4'd11);
                            h_arr[{word_idx, 4'd12}] <= `FALCON_NTT_COEFF(mem_rd_data, 4'd12);
                            h_arr[{word_idx, 4'd13}] <= `FALCON_NTT_COEFF(mem_rd_data, 4'd13);
                            h_arr[{word_idx, 4'd14}] <= `FALCON_NTT_COEFF(mem_rd_data, 4'd14);
                            h_arr[{word_idx, 4'd15}] <= `FALCON_NTT_COEFF(mem_rd_data, 4'd15);
                            ls <= LS_NEXT;
                        end
                        LS_NEXT: begin
                            if (word_idx == N_WORDS-1) begin
                                op_state <= OP_LOAD_S2_SB;
                                ls <= LS_IDLE;
                            end else begin
                                word_idx <= word_idx + 1'b1;
                                ls <= LS_RD_A;
                            end
                        end
                        default: ls <= LS_IDLE;
                    endcase
                end

                OP_LOAD_S2_SB: begin
                    case (ls)
                        LS_IDLE: begin word_idx <= 0; ls <= LS_RD_A; end
                        LS_RD_A: begin
                            mem_rd_en <= 1;
                            mem_rd_addr <= cfg_s2_base + {{(ADDR_W-5){1'b0}}, word_idx};
                            ls <= LS_WAIT_A;
                        end
                        LS_WAIT_A: ls <= LS_WAIT_A2;
                        LS_WAIT_A2: ls <= LS_CAPT_A;
                        LS_CAPT_A: begin
                            s2_arr[{word_idx, 4'd0}]  <= `FALCON_NTT_COEFF(mem_rd_data, 4'd0);
                            s2_arr[{word_idx, 4'd1}]  <= `FALCON_NTT_COEFF(mem_rd_data, 4'd1);
                            s2_arr[{word_idx, 4'd2}]  <= `FALCON_NTT_COEFF(mem_rd_data, 4'd2);
                            s2_arr[{word_idx, 4'd3}]  <= `FALCON_NTT_COEFF(mem_rd_data, 4'd3);
                            s2_arr[{word_idx, 4'd4}]  <= `FALCON_NTT_COEFF(mem_rd_data, 4'd4);
                            s2_arr[{word_idx, 4'd5}]  <= `FALCON_NTT_COEFF(mem_rd_data, 4'd5);
                            s2_arr[{word_idx, 4'd6}]  <= `FALCON_NTT_COEFF(mem_rd_data, 4'd6);
                            s2_arr[{word_idx, 4'd7}]  <= `FALCON_NTT_COEFF(mem_rd_data, 4'd7);
                            s2_arr[{word_idx, 4'd8}]  <= `FALCON_NTT_COEFF(mem_rd_data, 4'd8);
                            s2_arr[{word_idx, 4'd9}]  <= `FALCON_NTT_COEFF(mem_rd_data, 4'd9);
                            s2_arr[{word_idx, 4'd10}] <= `FALCON_NTT_COEFF(mem_rd_data, 4'd10);
                            s2_arr[{word_idx, 4'd11}] <= `FALCON_NTT_COEFF(mem_rd_data, 4'd11);
                            s2_arr[{word_idx, 4'd12}] <= `FALCON_NTT_COEFF(mem_rd_data, 4'd12);
                            s2_arr[{word_idx, 4'd13}] <= `FALCON_NTT_COEFF(mem_rd_data, 4'd13);
                            s2_arr[{word_idx, 4'd14}] <= `FALCON_NTT_COEFF(mem_rd_data, 4'd14);
                            s2_arr[{word_idx, 4'd15}] <= `FALCON_NTT_COEFF(mem_rd_data, 4'd15);
                            ls <= LS_NEXT;
                        end
                        LS_NEXT: begin
                            if (word_idx == N_WORDS-1) begin
                                op_state <= OP_LOAD_C_SB;
                                ls <= LS_IDLE;
                            end else begin
                                word_idx <= word_idx + 1'b1;
                                ls <= LS_RD_A;
                            end
                        end
                        default: ls <= LS_IDLE;
                    endcase
                end

                OP_LOAD_C_SB: begin
                    case (ls)
                        LS_IDLE: begin word_idx <= 0; ls <= LS_RD_A; end
                        LS_RD_A: begin
                            mem_rd_en <= 1;
                            mem_rd_addr <= cfg_c_base + {{(ADDR_W-5){1'b0}}, word_idx};
                            ls <= LS_WAIT_A;
                        end
                        LS_WAIT_A: ls <= LS_WAIT_A2;
                        LS_WAIT_A2: ls <= LS_CAPT_A;
                        LS_CAPT_A: begin
                            c_arr[{word_idx, 4'd0}]  <= `FALCON_NTT_COEFF(mem_rd_data, 4'd0);
                            c_arr[{word_idx, 4'd1}]  <= `FALCON_NTT_COEFF(mem_rd_data, 4'd1);
                            c_arr[{word_idx, 4'd2}]  <= `FALCON_NTT_COEFF(mem_rd_data, 4'd2);
                            c_arr[{word_idx, 4'd3}]  <= `FALCON_NTT_COEFF(mem_rd_data, 4'd3);
                            c_arr[{word_idx, 4'd4}]  <= `FALCON_NTT_COEFF(mem_rd_data, 4'd4);
                            c_arr[{word_idx, 4'd5}]  <= `FALCON_NTT_COEFF(mem_rd_data, 4'd5);
                            c_arr[{word_idx, 4'd6}]  <= `FALCON_NTT_COEFF(mem_rd_data, 4'd6);
                            c_arr[{word_idx, 4'd7}]  <= `FALCON_NTT_COEFF(mem_rd_data, 4'd7);
                            c_arr[{word_idx, 4'd8}]  <= `FALCON_NTT_COEFF(mem_rd_data, 4'd8);
                            c_arr[{word_idx, 4'd9}]  <= `FALCON_NTT_COEFF(mem_rd_data, 4'd9);
                            c_arr[{word_idx, 4'd10}] <= `FALCON_NTT_COEFF(mem_rd_data, 4'd10);
                            c_arr[{word_idx, 4'd11}] <= `FALCON_NTT_COEFF(mem_rd_data, 4'd11);
                            c_arr[{word_idx, 4'd12}] <= `FALCON_NTT_COEFF(mem_rd_data, 4'd12);
                            c_arr[{word_idx, 4'd13}] <= `FALCON_NTT_COEFF(mem_rd_data, 4'd13);
                            c_arr[{word_idx, 4'd14}] <= `FALCON_NTT_COEFF(mem_rd_data, 4'd14);
                            c_arr[{word_idx, 4'd15}] <= `FALCON_NTT_COEFF(mem_rd_data, 4'd15);
                            ls <= LS_NEXT;
                        end
                        LS_NEXT: begin
                            if (word_idx == N_WORDS-1) begin
                                sb_i <= 0;
                                sb_j <= 0;
                                sb_acc <= 0;
                                op_state <= OP_CONV_SB;
                            end else begin
                                word_idx <= word_idx + 1'b1;
                                ls <= LS_RD_A;
                            end
                        end
                        default: ls <= LS_IDLE;
                    endcase
                end

                OP_CONV_SB: begin
                    if (sb_j <= sb_i) begin
                        sb_h_idx = sb_i - sb_j;
                        sb_neg = 1'b0;
                    end else begin
                        sb_h_idx = sb_i - sb_j;
                        sb_neg = 1'b1;
                    end
                    sb_prod = `FALCON_NTT_BARRETT(s2_arr[sb_j] * h_arr[sb_h_idx]);
                    if (sb_neg)
                        sb_acc <= `FALCON_NTT_SUB_MODQ(sb_acc, sb_prod);
                    else
                        sb_acc <= `FALCON_NTT_ADD_MODQ(sb_acc, sb_prod);

                    if (sb_j == 9'd511) begin
                        s1_arr[sb_i] <= `FALCON_NTT_SUB_MODQ(c_arr[sb_i], sb_neg ? `FALCON_NTT_SUB_MODQ(sb_acc, sb_prod) : `FALCON_NTT_ADD_MODQ(sb_acc, sb_prod));
                        sb_acc <= 0;
                        sb_j <= 0;
                        if (sb_i == 9'd511) begin
                            word_idx <= 0;
                            op_state <= OP_WRITE_SB;
                        end else begin
                            sb_i <= sb_i + 1'b1;
                        end
                    end else begin
                        sb_j <= sb_j + 1'b1;
                    end
                end

                OP_WRITE_SB: begin
                    mem_wr_en <= 1;
                    mem_wr_addr <= cfg_dst_base + {{(ADDR_W-5){1'b0}}, word_idx};
                    mem_wr_data <= `FALCON_NTT_SET_COEFF(`FALCON_NTT_SET_COEFF(`FALCON_NTT_SET_COEFF(`FALCON_NTT_SET_COEFF(
                                   `FALCON_NTT_SET_COEFF(`FALCON_NTT_SET_COEFF(`FALCON_NTT_SET_COEFF(`FALCON_NTT_SET_COEFF(
                                   `FALCON_NTT_SET_COEFF(`FALCON_NTT_SET_COEFF(`FALCON_NTT_SET_COEFF(`FALCON_NTT_SET_COEFF(
                                   `FALCON_NTT_SET_COEFF(`FALCON_NTT_SET_COEFF(`FALCON_NTT_SET_COEFF(`FALCON_NTT_SET_COEFF(
                                   256'd0,
                                   4'd0,  s1_arr[{word_idx, 4'd0}]),
                                   4'd1,  s1_arr[{word_idx, 4'd1}]),
                                   4'd2,  s1_arr[{word_idx, 4'd2}]),
                                   4'd3,  s1_arr[{word_idx, 4'd3}]),
                                   4'd4,  s1_arr[{word_idx, 4'd4}]),
                                   4'd5,  s1_arr[{word_idx, 4'd5}]),
                                   4'd6,  s1_arr[{word_idx, 4'd6}]),
                                   4'd7,  s1_arr[{word_idx, 4'd7}]),
                                   4'd8,  s1_arr[{word_idx, 4'd8}]),
                                   4'd9,  s1_arr[{word_idx, 4'd9}]),
                                   4'd10, s1_arr[{word_idx, 4'd10}]),
                                   4'd11, s1_arr[{word_idx, 4'd11}]),
                                   4'd12, s1_arr[{word_idx, 4'd12}]),
                                   4'd13, s1_arr[{word_idx, 4'd13}]),
                                   4'd14, s1_arr[{word_idx, 4'd14}]),
                                   4'd15, s1_arr[{word_idx, 4'd15}]);
                    if (word_idx == N_WORDS-1) begin
                        op_state <= OP_DONE;
                    end else begin
                        word_idx <= word_idx + 1'b1;
                    end
                end

                OP_COPY_H: begin
                    case (ls)
                        LS_IDLE: begin
                            word_idx <= 0;
                            ls <= LS_RD_A;
                        end
                        LS_RD_A: begin
                            mem_rd_en <= 1;
                            mem_rd_addr <= cfg_h_base + {{(ADDR_W-5){1'b0}}, word_idx};
                            ls <= LS_WAIT_A;
                        end
                        LS_WAIT_A: begin
                            ls <= LS_WAIT_A2;
                        end
                        LS_WAIT_A2: begin
                            ls <= LS_CAPT_A;
                        end
                        LS_CAPT_A: begin
                            word_buf_a <= mem_rd_data;
                            ls <= LS_WR;
                        end
                        LS_WR: begin
                            mem_wr_en <= 1;
                            mem_wr_addr <= cfg_h_work_base + {{(ADDR_W-5){1'b0}}, word_idx};
                            mem_wr_data <= word_buf_a;
                            ls <= LS_NEXT;
                        end
                        LS_NEXT: begin
                            if (word_idx == N_WORDS-1) begin
                                op_state <= OP_COPY_S2;
                                ls       <= LS_IDLE;
                            end else begin
                                word_idx <= word_idx + 1'b1;
                                ls <= LS_RD_A;
                            end
                        end
                        default: ls <= LS_IDLE;
                    endcase
                end

                OP_COPY_S2: begin
                    case (ls)
                        LS_IDLE: begin
                            word_idx <= 0;
                            ls <= LS_RD_A;
                        end
                        LS_RD_A: begin
                            mem_rd_en <= 1;
                            mem_rd_addr <= cfg_s2_base + {{(ADDR_W-5){1'b0}}, word_idx};
                            ls <= LS_WAIT_A;
                        end
                        LS_WAIT_A: begin
                            ls <= LS_WAIT_A2;
                        end
                        LS_WAIT_A2: begin
                            ls <= LS_CAPT_A;
                        end
                        LS_CAPT_A: begin
                            word_buf_a <= mem_rd_data;
                            ls <= LS_WR;
                        end
                        LS_WR: begin
                            mem_wr_en <= 1;
                            mem_wr_addr <= cfg_s2_work_base + {{(ADDR_W-5){1'b0}}, word_idx};
                            mem_wr_data <= word_buf_a;
                            ls <= LS_NEXT;
                        end
                        LS_NEXT: begin
                            if (word_idx == N_WORDS-1) begin
                                op_target <= TGT_H;
                                inv_mode  <= 0;
                                op_state  <= OP_PRE;
                                ls        <= LS_IDLE;
                            end else begin
                                word_idx <= word_idx + 1'b1;
                                ls <= LS_RD_A;
                            end
                        end
                        default: ls <= LS_IDLE;
                    endcase
                end

                // OP_BITREV is handled above (separate case), transitions go here via bitrev_next_op

                // ─── PRE (pre-mul by psi^i) ───
                // Memory is registered-read (1 cycle latency).
                // LS_RD_A issues read, LS_WAIT_A waits, LS_CAPT_A captures data.
                OP_PRE: begin
                    case (ls)
                        LS_IDLE: begin
                            word_idx <= 0; lane_idx <= 0;
                            psi_rom_addr <= {1'b0, 5'd0, 4'd0};
                            ls <= LS_RD_A;
                        end
                        LS_RD_A: begin
                            cur_base <= target_base;
                            mem_rd_en <= 1;
                            mem_rd_addr <= target_base + {{(ADDR_W-5){1'b0}}, word_idx};
                            ls <= LS_WAIT_A;
                        end
                        LS_WAIT_A: begin ls <= LS_WAIT_A2; end
                        LS_WAIT_A2: begin ls <= LS_CAPT_A; end
                        LS_CAPT_A: begin
                            // mem_rd_data has word (from read in LS_RD_A, captured here)
                            word_buf_a <= `FALCON_NTT_SET_COEFF(mem_rd_data, 4'd0,
                                `FALCON_NTT_BARRETT(`FALCON_NTT_COEFF(mem_rd_data, 4'd0) * psi_rom_data));
                            psi_rom_addr <= {1'b0, word_idx[4:0], 4'd1};
                            lane_idx <= 4'd1;
                            ls <= LS_COMP;
                        end
                        LS_COMP: begin
                            word_buf_a <= `FALCON_NTT_SET_COEFF(word_buf_a, lane_idx,
                                `FALCON_NTT_BARRETT(`FALCON_NTT_COEFF(word_buf_a, lane_idx) * psi_rom_data));
                            if (lane_idx < 4'd15) begin
                                lane_idx <= lane_idx + 4'd1;
                                psi_rom_addr <= {1'b0, word_idx[4:0], lane_idx + 4'd1};
                            end else begin
                                psi_rom_addr <= {1'b0, word_idx[4:0] + 5'd1, 4'd0};
                                ls <= LS_WR;
                            end
                        end
                        LS_WR: begin
                            mem_wr_en <= 1;
                            mem_wr_addr <= cur_base + {{(ADDR_W-5){1'b0}}, word_idx};
                            mem_wr_data <= word_buf_a;
                            ls <= LS_NEXT;
                        end
                        LS_NEXT: begin
                            if (word_idx < (N_WORDS - 1)) begin
                                word_idx <= word_idx + 5'd1;
                                lane_idx <= 4'd0;
                                ls <= LS_RD_A;
                            end else begin
                                ls <= LS_IDLE;
                                // CG addr gen + BITREV: same as DIT but with CG addressing
                                op_state <= OP_BITREV;
                                bitrev_next_op <= OP_NTT;
                            end
                        end
                    endcase
                end

                // ─── POST (post-mul by psi^(-i) * N_inv) ───
                OP_POST: begin
                    case (ls)
                        LS_IDLE: begin
                            word_idx <= 0; lane_idx <= 0;
                            psi_rom_addr <= {1'b1, 5'd0, 4'd0};
                            ls <= LS_RD_A;
                        end
                        LS_RD_A: begin
                            cur_base <= target_base;
                            mem_rd_en <= 1;
                            mem_rd_addr <= target_base + {{(ADDR_W-5){1'b0}}, word_idx};
                            ls <= LS_WAIT_A;
                        end
                        LS_WAIT_A: begin ls <= LS_WAIT_A2; end
                        LS_WAIT_A2: begin ls <= LS_CAPT_A; end
                        LS_CAPT_A: begin
                            word_buf_a <= `FALCON_NTT_SET_COEFF(mem_rd_data, 4'd0,
                                `FALCON_NTT_BARRETT(`FALCON_NTT_COEFF(mem_rd_data, 4'd0) * psi_rom_data));
                            psi_rom_addr <= {1'b1, word_idx[4:0], 4'd1};
                            lane_idx <= 4'd1;
                            ls <= LS_COMP;
                        end
                        LS_COMP: begin
                            word_buf_a <= `FALCON_NTT_SET_COEFF(word_buf_a, lane_idx,
                                `FALCON_NTT_BARRETT(`FALCON_NTT_COEFF(word_buf_a, lane_idx) * psi_rom_data));
                            if (lane_idx < 4'd15) begin
                                lane_idx <= lane_idx + 4'd1;
                                psi_rom_addr <= {1'b1, word_idx[4:0], lane_idx + 4'd1};
                            end else begin
                                psi_rom_addr <= {1'b1, word_idx[4:0] + 5'd1, 4'd0};
                                ls <= LS_WR;
                            end
                        end
                        LS_WR: begin
                            mem_wr_en <= 1;
                            mem_wr_addr <= cur_base + {{(ADDR_W-5){1'b0}}, word_idx};
                            mem_wr_data <= word_buf_a;
                            ls <= LS_NEXT;
                        end
                        LS_NEXT: begin
                            if (word_idx < (N_WORDS - 1)) begin
                                word_idx <= word_idx + 5'd1;
                                lane_idx <= 4'd0;
                                ls <= LS_RD_A;
                            end else begin
                                ls <= LS_IDLE;
                                op_state <= OP_SUB_C;
                            end
                        end
                    endcase
                end

                // ─── NTT butterflies (forward or inverse) ───
                // Twiddle pipeline: addr→ROM(1)→tw_data_q(1)→tw_data_q2(1) = 3 cycles.
                // Set twiddle_rom_addr in BS_RDA, use tw_data_q2 in BS_BFLY (3 cycles later).
                OP_NTT: begin
                    case (bs)
                        BS_IDLE: begin
                            stage_idx <= 0; pair_idx <= 0;
                            cur_base  <= target_base;
                            bs <= BS_RDA;
                        end
                        BS_RDA: begin
                            mem_rd_en <= 1;
                            mem_rd_addr <= cur_base + ag_word_a;
                            twiddle_rom_addr <= {inv_mode, ag_twiddle_idx};
                            bs <= BS_WA;
                        end
                        BS_WA: begin
                            // Wait for word_a data (registered read)
                            bs <= BS_WA2;
                        end
                        BS_WA2: begin
                            // Second wait for Port B registered read visibility.
                            bs <= BS_RDB;
                        end
                        BS_RDB: begin
                            word_buf_a <= mem_rd_data;  // word_a arrives here
                            if (ag_word_a == ag_word_b) begin
                                // Early NTT stages often keep both butterfly
                                // operands inside the same 256-bit memory word.
                                // Reuse the captured word and skip the redundant
                                // second read/wait sequence.
                                word_buf_b <= mem_rd_data;
                                bfly_go <= 1'b1;
                                bs <= BS_BFLY;
                            end else begin
                                mem_rd_en <= 1;
                                mem_rd_addr <= cur_base + ag_word_b;
                                bfly_go <= 1'b0;
                                bs <= BS_WB;
                            end
                        end
                        BS_WB: begin
                            // Wait for word_b data (registered read)
                            bs <= BS_WB2;
                        end
                        BS_WB2: begin
                            // Second wait for Port B registered read visibility.
                            word_buf_b <= mem_rd_data;
                            bfly_go <= 1'b1;
                            bs <= BS_BFLY;
                        end
                        BS_BFLY: begin
                            if (bfly_go) begin
                                bfly_in_valid <= 1;
                                bfly_a <= `FALCON_NTT_COEFF(word_buf_a, ag_lane_a);
                                bfly_b <= `FALCON_NTT_COEFF(word_buf_b, ag_lane_b);
                                bfly_w <= twiddle_rom_data;
                                bfly_go <= 0;
                            end
                            if (bfly_out_valid) begin
                                bfly_same_word <= (ag_word_a == ag_word_b);
                                if (ag_word_a == ag_word_b) begin
                                    word_buf_a <= `FALCON_NTT_SET_COEFF(`FALCON_NTT_SET_COEFF(word_buf_a, ag_lane_a, bfly_y0), ag_lane_b, bfly_y1);
                                end else begin
                                    word_buf_a <= `FALCON_NTT_SET_COEFF(word_buf_a, ag_lane_a, bfly_y0);
                                    word_buf_b <= `FALCON_NTT_SET_COEFF(word_buf_b, ag_lane_b, bfly_y1);
                                end
                                bs <= BS_WRA;
                            end
                        end
                        BS_WRA: begin
                            mem_wr_en <= 1;
                            mem_wr_addr <= cur_base + ag_word_a;
                            mem_wr_data <= word_buf_a;
                            bs <= BS_WRB;
                        end
                        BS_WRB: begin
                            if (!bfly_same_word) begin
                                mem_wr_en <= 1;
                                mem_wr_addr <= cur_base + ag_word_b;
                                mem_wr_data <= word_buf_b;
                            end
                            bs <= BS_NEXT;
                        end
                        BS_NEXT: begin
                            if (pair_idx == 9'd255) begin
                                if (stage_idx == (LOGN - 5'd1)) begin
                                    bs <= BS_IDLE;
                                    case (op_target)
                                        TGT_H: begin
                                            op_target <= TGT_S2;
                                            op_state <= OP_PRE;
                                        end
                                        TGT_S2: begin
                                            // DIT NTT output is already in natural order.
                                            op_target <= TGT_DST;
                                            op_state <= OP_POINTWISE;
                                        end
                                        TGT_DST: begin
                                            // DIT inverse output is already in natural order.
                                            op_state <= OP_POST;
                                        end
                                    endcase
                                end else begin
                                    stage_idx <= stage_idx + 5'd1;
                                    pair_idx <= 9'd0;
                                    bs <= BS_RDA;
                                end
                            end else begin
                                pair_idx <= pair_idx + 9'd1;
                                bs <= BS_RDA;
                            end
                        end
                    endcase
                end

                // ─── Pointwise multiply: h_ntt * s2_ntt → dst ───
                OP_POINTWISE: begin
                    case (ls)
                        LS_IDLE: begin
                            word_idx <= 0; lane_idx <= 0;
                            ls <= LS_RD_A;
                        end
                        LS_RD_A: begin
                            mem_rd_en <= 1;
                            mem_rd_addr <= cfg_h_work_base + {{(ADDR_W-5){1'b0}}, word_idx};
                            ls <= LS_WAIT_A;
                        end
                        LS_WAIT_A: begin ls <= LS_WAIT_A2; end
                        LS_WAIT_A2: begin ls <= LS_CAPT_A; end
                        LS_CAPT_A: begin
                            word_buf_a <= mem_rd_data;  // h_ntt word
                            mem_rd_en <= 1;
                            mem_rd_addr <= cfg_s2_work_base + {{(ADDR_W-5){1'b0}}, word_idx};
                            lane_idx <= 0;
                            ls <= LS_WAIT_B;
                        end
                        LS_WAIT_B: begin ls <= LS_WAIT_B2; end
                        LS_WAIT_B2: begin ls <= LS_CAPT_B; end
                        LS_CAPT_B: begin
                            word_buf_b <= mem_rd_data;  // s2_ntt word
                            ls <= LS_COMP;
                        end
                        LS_COMP: begin
                            word_buf_a <= `FALCON_NTT_SET_COEFF(word_buf_a, lane_idx,
                                `FALCON_NTT_BARRETT(`FALCON_NTT_COEFF(word_buf_a, lane_idx) *
                                        `FALCON_NTT_COEFF(word_buf_b, lane_idx)));
                            if (lane_idx < 4'd15) begin
                                lane_idx <= lane_idx + 4'd1;
                            end else begin
                                ls <= LS_WR;
                            end
                        end
                        LS_WR: begin
                            mem_wr_en <= 1;
                            mem_wr_addr <= cfg_dst_base + {{(ADDR_W-5){1'b0}}, word_idx};
                            mem_wr_data <= word_buf_a;
                            ls <= LS_NEXT;
                        end
                        LS_NEXT: begin
                            if (word_idx < (N_WORDS - 1)) begin
                                word_idx <= word_idx + 5'd1;
                                lane_idx <= 4'd0;
                                ls <= LS_RD_A;
                            end else begin
                                ls <= LS_IDLE;
                                op_target <= TGT_DST;
                                inv_mode  <= 1;
                                op_state  <= OP_BITREV;
                                bitrev_next_op <= OP_NTT;  // then inverse NTT
                            end
                        end
                    endcase
                end

                // ─── Subtract: s1 = c - result mod Q ───
                OP_SUB_C: begin
                    case (ls)
                        LS_IDLE: begin
                            word_idx <= 0; lane_idx <= 0;
                            ls <= LS_RD_A;
                        end
                        LS_RD_A: begin
                            mem_rd_en <= 1;
                            mem_rd_addr <= cfg_c_base + {{(ADDR_W-5){1'b0}}, word_idx};
                            ls <= LS_WAIT_A;
                        end
                        LS_WAIT_A: begin ls <= LS_WAIT_A2; end
                        LS_WAIT_A2: begin ls <= LS_CAPT_A; end
                        LS_CAPT_A: begin
                            word_buf_a <= mem_rd_data;
                            mem_rd_en <= 1;
                            mem_rd_addr <= cfg_dst_base + {{(ADDR_W-5){1'b0}}, word_idx};
                            lane_idx <= 0;
                            ls <= LS_WAIT_B;
                        end
                        LS_WAIT_B: begin ls <= LS_WAIT_B2; end
                        LS_WAIT_B2: begin ls <= LS_CAPT_B; end
                        LS_CAPT_B: begin
                            word_buf_b <= mem_rd_data;
                            ls <= LS_COMP;
                        end
                        LS_COMP: begin
                            // s1 = (c - result) mod Q
                            if (`FALCON_NTT_COEFF(word_buf_a, lane_idx) >= `FALCON_NTT_COEFF(word_buf_b, lane_idx))
                                word_buf_a <= `FALCON_NTT_SET_COEFF(word_buf_a, lane_idx,
                                    `FALCON_NTT_COEFF(word_buf_a, lane_idx) - `FALCON_NTT_COEFF(word_buf_b, lane_idx));
                            else
                                word_buf_a <= `FALCON_NTT_SET_COEFF(word_buf_a, lane_idx,
                                    `FALCON_NTT_COEFF(word_buf_a, lane_idx) + Q - `FALCON_NTT_COEFF(word_buf_b, lane_idx));

                            if (lane_idx < 4'd15) begin
                                lane_idx <= lane_idx + 4'd1;
                            end else begin
                                ls <= LS_WR;
                            end
                        end
                        LS_WR: begin
                            mem_wr_en <= 1;
                            mem_wr_addr <= cfg_dst_base + {{(ADDR_W-5){1'b0}}, word_idx};
                            mem_wr_data <= word_buf_a;
                            ls <= LS_NEXT;
                        end
                        LS_NEXT: begin
                            if (word_idx < (N_WORDS - 1)) begin
                                word_idx <= word_idx + 5'd1;
                                lane_idx <= 4'd0;
                                ls <= LS_RD_A;
                            end else begin
                                ls <= LS_IDLE;
                                op_state <= OP_DONE;
                            end
                        end
                    endcase
                end

                // ─── BITREV: natural <-> bit-reversed permutation ───
                // Registered memory reads need 2-cycle wait before data capture
                OP_BITREV: begin
                    case (bt)
                        BT_IDLE: begin bt_nat_idx <= 0; bt <= BT_RD; end
                        BT_RD: begin
                            if (bt_nat_idx >= br_coeff) begin
                                bt <= BT_NEXT;
                            end else begin
                                mem_rd_en <= 1;
                                mem_rd_addr <= target_base + {{(ADDR_W-5){1'b0}}, bt_nat_idx[8:4]};
                                bt <= BT_RD_W1;
                            end
                        end
                        BT_RD_W1: begin bt <= BT_RD_W2; end
                        BT_RD_W2: begin bt <= BT_RD_C; end
                        BT_RD_C: begin
                            // mem_rd_data has source word (3 cycles after BT_RD)
                            bt_src_word <= mem_rd_data;
                            bt_coeff <= `FALCON_NTT_COEFF(mem_rd_data, bt_nat_idx[3:0]);
                            bt_same_word <= (bt_nat_idx[8:4] == br_word[4:0]);
                            mem_rd_en <= 1;
                            mem_rd_addr <= target_base + br_word;
                            bt <= BT_WR_RD;
                        end
                        BT_WR_RD: begin bt <= BT_WR_W1; end
                        BT_WR_W1: begin bt <= BT_WR_W2; end
                        BT_WR_W2: begin bt <= BT_WR_C; end
                        BT_WR_C: begin
                            // mem_rd_data has dest word
                            bt_dst_coeff <= `FALCON_NTT_COEFF(mem_rd_data, br_lane);
                            mem_wr_en <= 1;
                            if (bt_same_word) begin
                                mem_wr_addr <= target_base + br_word;
                                mem_wr_data <= `FALCON_NTT_SET_COEFF(
                                    `FALCON_NTT_SET_COEFF(mem_rd_data, br_lane, bt_coeff),
                                    bt_nat_idx[3:0],
                                    `FALCON_NTT_COEFF(mem_rd_data, br_lane));
                            end else begin
                                mem_wr_addr <= target_base + br_word;
                                mem_wr_data <= `FALCON_NTT_SET_COEFF(mem_rd_data, br_lane, bt_coeff);
                            end
                            bt <= BT_WR;
                        end
                        BT_WR: begin
                            if (!bt_same_word) begin
                                mem_wr_en <= 1;
                                mem_wr_addr <= target_base + {{(ADDR_W-5){1'b0}}, bt_nat_idx[8:4]};
                                mem_wr_data <= `FALCON_NTT_SET_COEFF(bt_src_word, bt_nat_idx[3:0], bt_dst_coeff);
                            end
                            bt <= BT_NEXT;
                        end
                        BT_NEXT: begin
                            if (bt_nat_idx == 9'd511) begin
                                bt <= BT_IDLE;
                                op_state <= bitrev_next_op;
                            end else begin
                                bt_nat_idx <= bt_nat_idx + 9'd1;
                                bt <= BT_RD;
                            end
                        end
                        default: bt <= BT_IDLE;
                    endcase
                end

                OP_DONE: begin
                    done    <= 1;
                    status  <= 0;
                    op_state <= OP_IDLE;
                end

                OP_FAIL: begin
                    fail    <= 1;
                    status  <= 8'hFF;
                    op_state <= OP_IDLE;
                end

                default: op_state <= OP_IDLE;
            endcase
        end
    end

endmodule
