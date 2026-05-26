`timescale 1ns/1ps
// NTT DIT address generator — same pattern as falcon_fft_addr_gen_cfg.v
// but addressing individual coefficients (0..N-1) instead of complex-pair words.
// Outputs: coefficient indices a_idx, b_idx (9-bit), twiddle index (8-bit),
// and word-level addresses (word = coeff >> 4, lane = coeff & 15).
module falconsign_ntt_addr_gen #(
    parameter LOGN  = 5'd9,       // log2(N) = 9 for Falcon-512
    parameter ADDR_W = 11          // memory address width
) (
    input  wire [4:0]  stage_idx,  // 0..8
    input  wire [8:0]  pair_idx,   // 0..255
    output wire [8:0]  coeff_a,    // 0..511
    output wire [8:0]  coeff_b,    // 0..511
    output wire [7:0]  twiddle_idx,// 0..255
    output wire [ADDR_W-1:0] word_a, // word address for coeff a
    output wire [3:0]  lane_a,     // lane within word for coeff a (0..15)
    output wire [ADDR_W-1:0] word_b, // word address for coeff b
    output wire [3:0]  lane_b      // lane within word for coeff b (0..15)
);

    // DIT butterfly addressing:
    //   half    = 1 << stage_idx
    //   m_size  = half << 1  (butterfly group size)
    //   j_idx   = pair_idx & (half - 1)
    //   group   = pair_idx >> stage_idx
    //   base    = group * m_size
    //   coeff_a = base + j_idx
    //   coeff_b = coeff_a + half
    //   twiddle_idx = j_idx << (LOGN - stage_idx - 1)

    wire [8:0] half      = 9'd1 << stage_idx;
    wire [8:0] half_mask = half - 9'd1;
    wire [8:0] j_idx     = pair_idx & half_mask;
    wire [8:0] group     = pair_idx >> stage_idx;
    wire [8:0] m_size    = half << 1;
    wire [8:0] base      = group * m_size;

    assign coeff_a = base + j_idx;
    assign coeff_b = coeff_a + half;

    // Twiddle index: j_idx << (LOGN - stage_idx - 1)
    wire [4:0]  tw_shift = LOGN - stage_idx - 5'd1;
    assign twiddle_idx   = j_idx[7:0] << tw_shift;

    // DIT stages operate on natural logical addresses. Any required
    // bit-reversal is performed explicitly by the NTT EXU before the stages.
    assign word_a = {{(ADDR_W-5){1'b0}}, coeff_a[8:4]};
    assign lane_a = coeff_a[3:0];
    assign word_b = {{(ADDR_W-5){1'b0}}, coeff_b[8:4]};
    assign lane_b = coeff_b[3:0];

endmodule
