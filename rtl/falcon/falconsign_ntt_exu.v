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
    input  wire [ADDR_W-1:0] cfg_s2_base,
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

    falconsign_ntt_addr_gen #(.LOGN(LOGN), .ADDR_W(ADDR_W)) u_ag (
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

    // Barrett reduction
    function [13:0] barrett;
        input [27:0] prod;
        reg [42:0] qh;
        reg [13:0] q;
        reg [27:0] r;
        begin
            qh = prod * BARRETT_MU;
            q  = qh[41:28];
            r  = prod - q * Q;
            r  = (r >= Q) ? (r - Q) : r;
            barrett = (r >= Q) ? (r[13:0] - Q) : r[13:0];
        end
    endfunction

    // Word helpers (normalize signed 16-bit -> unsigned [0, Q-1])
    function [13:0] get_coeff;
        input [255:0] w;
        input [3:0]   la;
        reg signed [15:0] raw;
        begin
            raw = w[la*16 +: 16];
            if (raw < 0)
                get_coeff = raw + Q;  // raw is negative, Q is positive, result in [0, Q-1]
            else
                get_coeff = raw[13:0];
        end
    endfunction

    function [255:0] set_coeff;
        input [255:0] w;
        input [3:0]   la;
        input [13:0]  v;
        begin
            set_coeff = w;
            set_coeff[la*16 +: 16] = {2'd0, v};
        end
    endfunction

    // Base address for current target
    wire [ADDR_W-1:0] target_base;
    assign target_base = (op_target == TGT_H)  ? cfg_h_base :
                         (op_target == TGT_S2) ? cfg_s2_base : cfg_dst_base;

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
                        op_state  <= OP_PRE;
                        ls        <= LS_IDLE;
                    end
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
                            word_buf_a <= set_coeff(mem_rd_data, 4'd0,
                                barrett(get_coeff(mem_rd_data, 4'd0) * psi_rom_data));
                            psi_rom_addr <= {1'b0, word_idx[4:0], 4'd1};
                            lane_idx <= 4'd1;
                            ls <= LS_COMP;
                        end
                        LS_COMP: begin
                            word_buf_a <= set_coeff(word_buf_a, lane_idx,
                                barrett(get_coeff(word_buf_a, lane_idx) * psi_rom_data));
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
                                // After PRE: BITREV before NTT
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
                            word_buf_a <= set_coeff(mem_rd_data, 4'd0,
                                barrett(get_coeff(mem_rd_data, 4'd0) * psi_rom_data));
                            psi_rom_addr <= {1'b1, word_idx[4:0], 4'd1};
                            lane_idx <= 4'd1;
                            ls <= LS_COMP;
                        end
                        LS_COMP: begin
                            word_buf_a <= set_coeff(word_buf_a, lane_idx,
                                barrett(get_coeff(word_buf_a, lane_idx) * psi_rom_data));
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
                            mem_rd_en <= 1;
                            mem_rd_addr <= cur_base + ag_word_b;
                            bfly_go <= 1'b0;
                            bs <= BS_WB;
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
                                bfly_a <= get_coeff(word_buf_a, ag_lane_a);
                                bfly_b <= get_coeff(word_buf_b, ag_lane_b);
                                bfly_w <= twiddle_rom_data;
                                bfly_go <= 0;
                            end
                            if (bfly_out_valid) begin
                                bfly_same_word <= (ag_word_a == ag_word_b);
                                if (ag_word_a == ag_word_b) begin
                                    word_buf_a <= set_coeff(set_coeff(word_buf_a, ag_lane_a, bfly_y0), ag_lane_b, bfly_y1);
                                end else begin
                                    word_buf_a <= set_coeff(word_buf_a, ag_lane_a, bfly_y0);
                                    word_buf_b <= set_coeff(word_buf_b, ag_lane_b, bfly_y1);
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
                                            // DIT NTT output is already in natural order.
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
                            mem_rd_addr <= cfg_h_base + {{(ADDR_W-5){1'b0}}, word_idx};
                            ls <= LS_WAIT_A;
                        end
                        LS_WAIT_A: begin ls <= LS_WAIT_A2; end
                        LS_WAIT_A2: begin ls <= LS_CAPT_A; end
                        LS_CAPT_A: begin
                            word_buf_a <= mem_rd_data;  // h_ntt word
                            mem_rd_en <= 1;
                            mem_rd_addr <= cfg_s2_base + {{(ADDR_W-5){1'b0}}, word_idx};
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
                            word_buf_a <= set_coeff(word_buf_a, lane_idx,
                                barrett(get_coeff(word_buf_a, lane_idx) *
                                        get_coeff(word_buf_b, lane_idx)));
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
                            if (get_coeff(word_buf_a, lane_idx) >= get_coeff(word_buf_b, lane_idx))
                                word_buf_a <= set_coeff(word_buf_a, lane_idx,
                                    get_coeff(word_buf_a, lane_idx) - get_coeff(word_buf_b, lane_idx));
                            else
                                word_buf_a <= set_coeff(word_buf_a, lane_idx,
                                    get_coeff(word_buf_a, lane_idx) + Q - get_coeff(word_buf_b, lane_idx));

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
                            bt_coeff <= get_coeff(mem_rd_data, bt_nat_idx[3:0]);
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
                            bt_dst_coeff <= get_coeff(mem_rd_data, br_lane);
                            mem_wr_en <= 1;
                            if (bt_same_word) begin
                                mem_wr_addr <= target_base + br_word;
                                mem_wr_data <= set_coeff(
                                    set_coeff(mem_rd_data, br_lane, bt_coeff),
                                    bt_nat_idx[3:0],
                                    get_coeff(mem_rd_data, br_lane));
                            end else begin
                                mem_wr_addr <= target_base + br_word;
                                mem_wr_data <= set_coeff(mem_rd_data, br_lane, bt_coeff);
                            end
                            bt <= BT_WR;
                        end
                        BT_WR: begin
                            if (!bt_same_word) begin
                                mem_wr_en <= 1;
                                mem_wr_addr <= target_base + {{(ADDR_W-5){1'b0}}, bt_nat_idx[8:4]};
                                mem_wr_data <= set_coeff(bt_src_word, bt_nat_idx[3:0], bt_dst_coeff);
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
