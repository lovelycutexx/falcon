//=============================================================================
// Falcon ffSampling (SampleZ) 閳?discrete Gaussian sampler over integers.
//
// Implements Algorithm 12 (ffSampling) from the Falcon specification.
// Given a floating-point center mu and sigma=1/sigma_inv, returns an
// integer z sampled from the discrete Gaussian distribution D_{Z,mu,sigma}.
//
// The base sampler uses a fixed CDT table for sigma_max=1.8205 (half-Gaussian
// D_{Z+}), then corrects for the actual target sigma via rejection sampling.
// The rejection step evaluates exp(-x) with a 4th-order Taylor polynomial
// and range reduction for x >= 1.0.
//
// Ports:
//   cmd_mu        閳?distribution center 纰?(FP64)
//   cmd_sigma_inv 閳?1/锜?(FP64)
//   cmd_sigma_min 閳?锜絖min for security lower bound (FP64)
//   cmd_pair_mode 閳?1 = sample two independent z values in one command
//   rsp_z0, rsp_z1 閳?sampled integers as FP64 (z1 valid only in pair mode)
//   rsp_accept     閳?1 = sample accepted, 0 = rejected (retry loop)
//   rsp_status     閳?0xFF on failure (99 rejections exhausted)
//=============================================================================
`timescale 1ns/1ps
module falconsign_samplerz_top #(parameter RNG_DATA_W=256)(
  input wire clk,rst_n,cmd_valid, cmd_pair_mode,
  input wire[63:0] cmd_mu,cmd_sigma_inv,cmd_sigma_min,
  output wire cmd_ready,
  output reg rsp_valid, rsp_accept, done, fail,
  input wire rsp_ready,  output reg[63:0] rsp_z0,rsp_z1, output reg[7:0] rsp_status,
  output reg fpu_req_valid, input wire fpu_req_ready,
  output reg[3:0] fpu_req_op, output reg[63:0] fpu_req_a,fpu_req_b,fpu_req_c,
  output wire[1:0] fpu_req_fmt,
  output wire[2:0] fpu_req_rm,
  output wire[1:0] fpu_req_fcvt_op,
  input wire fpu_rsp_valid, output wire fpu_rsp_ready, input wire[63:0] fpu_rsp_result,

  output reg rng_req, input wire rng_ack, input wire[RNG_DATA_W-1:0] rng_data,
  output wire busy
);

  // FPU opcodes
  localparam[3:0] FA  = 0,   // FADD  閳?a + b
                  FS  = 1,   // FSUB  閳?a - b
                  FM  = 2,   // FMUL  閳?a * b
                  FMA = 3,   // FMADD 閳?a * b + c
                  FC  = 9,   // FCVT / compare
                  FF  = 12,  // FFLOOR
                  FI  = 13;  // FINT-TO-FLOAT

  // Taylor coefficients for exp(-x) 閳?1 - x + x铏?2 - x椴?6 + x閳?24
  // Evaluated as Horner: (((C4*x + C3)*x + C2)*x + C1)*x + 1.0
  localparam[63:0] C1 = 64'hBFF0000000000000,  // -1.0
                   C2 = 64'h3FE0000000000000,  //  0.5
                   C3 = 64'hBFC5555555555555,  // -1/6  閳?-0.1666667
                   C4 = 64'h3FA5555555555555,  //  1/24 閳? 0.0416667
                   F1 = 64'h3FF0000000000000,  //  1.0
                   FH = 64'h3FE0000000000000,  //  0.5
  // Range-reduction constants: ENn = exp(-n)
                   EN1 = 64'h3FD78B56362CEF38, // exp(-1) 閳?0.3679
                   EN2 = 64'h3FC152AAA3BF81CC, // exp(-2) 閳?0.1353
                   EN3 = 64'h3FA97DB0CCCEB0AF, // exp(-3) 閳?0.0498
  // z0 correction: NEG_INV_2SQRSIGMA0 = -1/(2*sigma_max^2) = -1/(2*1.8205^2)
  // Used in FMA to subtract z0铏?(2锜絖max铏? from rejection exponent
                   NEG_INV_2SQRSIGMA0 = 64'hBFC34F8BC183BBC2;

  //-----------------------------------------------------------------
  // FSM state encoding 閳?Algorithm 12 (ffSampling) step-by-step
  //
  // Setup phase:
  //   SI:  idle, wait for command
  //   SRR: request 256-bit RNG word
  //   SRW: wait for RNG acknowledge
  //   SRF: rfp = floor(mu) via hardware function f64_floor_i64
  //   SRI: rfp = int_to_float(rfp)          (FI op)
  //   SRS: r_frac = mu - rfp                (FS op)  fractional part 閳?[0,1)
  //   SRC: branch on r_frac sign (always non-negative in practice)
  //   SR1: r_frac = r_frac - 1.0            (FS op)  unreachable safety path
  //   SCM: ccs = sigma_min * sigma_inv       (FM op)  = 锜絖min/锜?  //
  // Base sampler phase (CDT scan):
  //   SSB: extract zs=sign-bit(b), ut=16-bit uniform, sc=0 from RNG buffer
  //   SSS: CDT scan loop 閳?find smallest sc where CDT[sc] >= ut, or sc=15
  //        On match: zi = zs ? (sc+1) : (-sc)
  //        This matches C-code: z = b + ((b<<1)-1)*z0
  //
  // Rejection sampling phase:
  //   SSI: z_fp = int_to_float(zi)           (FI op)
  //   SZA: z_out = z_fp + rfp                (FA op)  pre-compute accepted value
  //   SZR: zmr = z_fp - r_frac               (FS op)  = (z - r_frac)
  //   SDQ: dsq = zmr * zmr                   (FM op)  = (z - r_frac)^2
  //   SYM: by  = dsq * sigma_inv             (FM op)
  //   SYH: by  = by  * sigma_inv             (FM op)
  //   SYQ: by  = by  * 0.5                   (FM op)  閳?(z-r)铏?(2锜借檹)
  //
  // z0 correction 閳?subtracts base-sampler sigma_max bias from rejection:
  //   SZ0: ba = int_to_float(sc)              (FI op)  sc = z0 = |zi|
  //   SZ1: ba = ba * ba                       (FM op)  = z0铏?  //   SZ2: by = ba * NEG_INV + by            (FMA op)  by -= z0铏?(2锜絖max铏?
  //
  // BerExp 閳?Bernoulli experiment for exp(-x):
  //   SYR: check by exponent:
  //          > 1024 (by>2.0) 閳?instant reject
  //          ==1024 (by閳溂2,4)) 閳?fire FSUB to range-reduce (subtract 2 or 3)
  //          ==1023 (by閳溂1,2)) 閳?fire FSUB to range-reduce (subtract 1)
  //          < 1023 (by<1.0)  閳?skip FSUB, go to polynomial eval
  //        Also sets bs = exp(-n) for the subtracted amount.
  //   SYT: by = FSUB_result  (latch range-reduced by)
  //   SYP0: ba = C4*by + C3                  (FMA op)
  //   SYP1: ba = ba*by + C2                  (FMA op)
  //   SYP2: ba = ba*by + C1                  (FMA op)
  //   SYP3: ba = ba*by + 1.0                 (FMA op)  Horner done: ba 閳?exp(-x)
  //   SYS:  ba = ba * bs                     (FM op)   apply range-reduction scale
  //   SYC:  ru = uniform random float 閳?[0,1) from RNG buffer
  //   SY2:  rsp_accept = (ba > ru)           (FC op)   Bernoulli comparison
  //
  // Result dispatch:
  //   SCH: if accepted 閳?store z_out into rsp_z0/z1
  //        if rejected 閳?rc++, retry from SSB (fail at 99 rejections)
  //   SNS: switch to second sample (sp=1), loop to SSB (pair mode)
  //   SDO: drive rsp_valid, done
  //   SFA: drive rsp_valid, done, fail
  //-----------------------------------------------------------------
  localparam[5:0] SI=0,SRR=1,SRW=2,SRF=3,SRI=4,SRS=5,
    SRC=6,SR1=7,SCM=8,SSB=9,SSS=29,SSI=10,SZR=11,SDQ=12,SZA=30,
    SYM=13,SYH=14,SYR=25,SYP0=15,SYP1=16,SYP2=17,SYP3=18,SYS=28,
    SYC=19,SY2=20,SCH=21,SNS=22,SDO=23,SFA=24,SYQ=31,SYT=32,
    SZ0=26,SZ1=27,SZ2=33,SYS2=34,
    SZ2A=35,SYP0A=36,SYP1A=37,SYP2A=38,SYP3A=39;

  //-----------------------------------------------------------------
  // Internal registers
  //-----------------------------------------------------------------
  reg[5:0]  st, sn;             // FSM state, next-state
  // Command inputs (latched at SI)
  reg[63:0] mu,                  // 纰? distribution center (FP64)
            si,                  // 1/锜? inverse of standard deviation (FP64)
            sn_min;              // 锜絖min: security lower bound on sigma (FP64)
  reg       pm;                  // pair-mode flag: 1 = produce two samples
  // Computation pipeline
  reg[63:0] rfp;                 // floor(纰?: integer part (from f64_floor_i64), then as FP64
  reg[63:0] rint;                // floor(mu) as signed integer
  reg[63:0] r_frac;              // 纰?- floor(纰?: fractional part 閳?[0, 1)
  reg[63:0] ccs;                 // 锜絖min / 锜?(computed but unused in current rejection path)
  reg[15:0] zi;                  // sampled integer offset z (before adding floor(纰?)
  reg[63:0] z_fp,                // float(zi)
            z_out;               // float(zi) + float(floor(纰?) = accepted sample as FP64
  reg[63:0] zmr,                 // z_fp - r_frac = (z - r_frac)
            dsq;                 // zmr^2 = (z - r_frac)^2
  reg[63:0] by,                  // x = (z-r_frac)铏?(2锜借檹) 閳?input to BerExp
            ba,                  // polynomial result 閳?exp(-x)
            bs;                  // scale factor: exp(-n) from range reduction (or 1.0)
  reg[63:0] x_orig;              // original BerExp exponent before range reduction
  reg[63:0] sz_prod;             // registered product for local two-stage FMA
  reg       bn;                  // number of integer range-reduction steps (1..3)
  reg[63:0] ru;                  // uniform random float comparator 閳?[0, 1)
  // Base sampler state
  reg[23:0] bs_v0, bs_v1, bs_v2; // 72-bit draw, split into 24-bit limbs
  reg[4:0]  sc;                  // BaseSampler z0 = |zi| (0..18)
  reg       zs;                  // sign bit from RNG (b in spec: 1=positive, 0=negative)
  // RNG buffer and control
  reg[255:0] rb;                 // 256-bit random buffer (shift register)
  reg[7:0]  ra;                  // remaining random bits in rb (< 32 triggers auto-refill)
  reg       sr;                  // RNG request pending flag
  // Retry / pair-mode control
  reg[7:0]  rc;                  // consecutive rejection counter (fail at 99)
  reg       sp;                  // second-sample-produced flag (pair mode)
  // Cache sigma-dependent pre-computation. Adjacent Falcon leaf samples often
  // reuse the same sigma_inv, so ccs = sigma_min * sigma_inv can be carried
  // across commands instead of recomputed for every sample.
  reg       ccs_cache_valid;
  reg[63:0] ccs_cache_si;
  reg[63:0] ccs_cache_value;
  reg[31:0] debug_cmd_attempts;
  reg[31:0] debug_cmd_rejects;
  reg[31:0] debug_cmd_accepts;

  wire      ccs_cache_hit = ccs_cache_valid && (ccs_cache_si == si);

  wire frf = fpu_rsp_valid && fpu_rsp_ready;  // FPU result fire (handshake complete)
  wire [4:0] bs_now = bs_gaussian0(rb[23:0], rb[47:24], rb[71:48]);

  // Local SamplerZ arithmetic datapath.  The top-level FPU is shared with the
  // ffSampling EXU and serializes every operation; these local combinational
  // units let the SamplerZ stages advance one cycle at a time.
  reg  [63:0] sz_add_a, sz_add_b;
  reg         sz_add_sub;
  wire [63:0] sz_add_y;
  wire        sz_add_invalid, sz_add_overflow, sz_add_underflow, sz_add_inexact;
  reg  [63:0] sz_mul_a, sz_mul_b;
  wire [63:0] sz_mul_y;
  wire        sz_mul_invalid, sz_mul_overflow, sz_mul_underflow, sz_mul_inexact;

  falcon_f64_add u_sz_add (
    .a(sz_add_a), .b(sz_add_b), .sub(sz_add_sub), .y(sz_add_y),
    .invalid(sz_add_invalid), .overflow(sz_add_overflow),
    .underflow(sz_add_underflow), .inexact(sz_add_inexact)
  );

  falcon_f64_mul u_sz_mul (
    .a(sz_mul_a), .b(sz_mul_b), .y(sz_mul_y),
    .invalid(sz_mul_invalid), .overflow(sz_mul_overflow),
    .underflow(sz_mul_underflow), .inexact(sz_mul_inexact)
  );

  function bs_lt72;
    input [23:0] v0;
    input [23:0] v1;
    input [23:0] v2;
    input [23:0] w0;
    input [23:0] w1;
    input [23:0] w2;
    reg [24:0] d0;
    reg [24:0] d1;
    reg [24:0] d2;
    reg        c0;
    reg        c1;
    begin
      d0 = {1'b0, v0} - {1'b0, w0};
      c0 = d0[24];
      d1 = {1'b0, v1} - {1'b0, w1} - {{24{1'b0}}, c0};
      c1 = d1[24];
      d2 = {1'b0, v2} - {1'b0, w2} - {{24{1'b0}}, c1};
      bs_lt72 = d2[24];
    end
  endfunction

  function [4:0] bs_gaussian0;
    input [23:0] v0;
    input [23:0] v1;
    input [23:0] v2;
    reg [4:0] z;
    begin
      z = 5'd0;
      z = z + bs_lt72(v0, v1, v2, 24'd3104126,  24'd0,        24'd0);
      z = z + bs_lt72(v0, v1, v2, 24'd28824,    24'd0,        24'd0);
      z = z + bs_lt72(v0, v1, v2, 24'd198,      24'd0,        24'd0);
      z = z + bs_lt72(v0, v1, v2, 24'd1,        24'd0,        24'd0);
      z = z + bs_lt72(v0, v1, v2, 24'd12545723, 24'd14,       24'd0);
      z = z + bs_lt72(v0, v1, v2, 24'd6138264,  24'd870,      24'd0);
      z = z + bs_lt72(v0, v1, v2, 24'd9111839,  24'd38047,    24'd0);
      z = z + bs_lt72(v0, v1, v2, 24'd13644283, 24'd1232676,  24'd0);
      z = z + bs_lt72(v0, v1, v2, 24'd265321,   24'd12844466, 24'd1);
      z = z + bs_lt72(v0, v1, v2, 24'd8086568,  24'd8444042,  24'd31);
      z = z + bs_lt72(v0, v1, v2, 24'd11363290, 24'd16768101, 24'd417);
      z = z + bs_lt72(v0, v1, v2, 24'd7826148,  24'd14505003, 24'd4132);
      z = z + bs_lt72(v0, v1, v2, 24'd7650655,  24'd13063405, 24'd30538);
      z = z + bs_lt72(v0, v1, v2, 24'd4136815,  24'd7122675,  24'd169348);
      z = z + bs_lt72(v0, v1, v2, 24'd10046180, 24'd4421575,  24'd708981);
      z = z + bs_lt72(v0, v1, v2, 24'd2736639,  24'd13669192, 24'd2260429);
      z = z + bs_lt72(v0, v1, v2, 24'd8248194,  24'd1580863,  24'd5559083);
      z = z + bs_lt72(v0, v1, v2, 24'd3741698,  24'd3068844,  24'd10745844);
      bs_gaussian0 = z;
    end
  endfunction

  // FPU interface: always ready, format=0 (FP64), round-to-nearest, no fcvt
  assign fpu_rsp_ready = 1'b1;
  assign fpu_req_fmt   = 2'd0;
  assign fpu_req_rm    = 3'd0;
  assign fpu_req_fcvt_op = 2'd0;

  assign cmd_ready = (st == SI);
  assign busy = (st != SI && st != SDO && st != SFA);

  // Auto-refill: SSB consumes 80 bits (official 72-bit BaseSampler draw
  // plus one byte for b&1), and SYC consumes 52 bits for the Bernoulli draw.
  // Suppressed during SI, SDO, SFA, SRR, SRW states.
  wire rl = (ra < 8'd132) && !rng_req && !sr
         && (st != SI) && (st != SDO) && (st != SFA)
         && (st != SRR) && (st != SRW);

`ifndef SYNTHESIS
  // Debug: when +FS_SAMPLE_MU is set, return mu directly as samples
  // (bypasses the Gaussian sampler 閳?useful for testing caller logic)
  reg debug_return_mu;
  reg debug_trace_sampler;
  integer debug_sample_count;
  initial begin
    debug_return_mu = $test$plusargs("FS_SAMPLE_MU");
    debug_trace_sampler = $test$plusargs("FS_TRACE_SAMPLER");
    debug_sample_count = 0;
  end
`endif

  //===================================================================
  // f64_floor_i64: software-style FP64 floor returning 64-bit integer.
  //
  // Decomposes IEEE-754 FP64 into sign, exponent, fraction, then shifts
  // the mantissa to extract the integer part. Handles denormals (exp==0),
  // fractional-only values (|x|<1 閳?0), and overflow (exp >= 1086).
  //
  // For negative non-integer values, returns the floor toward -閳?  // (e.g., floor(-1.5) = -2), matching C's fpr_floor().
  //
  // Uses combinational logic only 閳?no FPU involvement, single-cycle.
  //===================================================================
  function [63:0] f64_floor_i64;
    input [63:0] x;
    reg sign;
    reg [10:0] exp;
    reg [51:0] frac;
    integer e;
    integer sh;
    reg [63:0] ip;
    reg has_frac;
    begin
      sign = x[63];
      exp  = x[62:52];
      frac = x[51:0];
      ip = 64'd0;
      has_frac = 1'b0;

      if (exp == 11'd0) begin
        // Denormal or zero: |x| < 2^-1022, so integer part is 0
        has_frac = (x[62:0] != 63'd0);
        ip = 64'd0;
      end else if (exp < 11'd1023) begin
        // |x| < 1: no integer part
        has_frac = 1'b1;
        ip = 64'd0;
      end else begin
        e = exp - 11'd1023;          // effective binary exponent
        if (e >= 63) begin
          // Overflow: value >= 2^63, saturate
          ip = 64'h7fffffffffffffff;
          has_frac = 1'b0;
        end else if (e >= 52) begin
          // Integer part fits in [52,62] bits: shift left
          ip = (64'd1 << e) | ({{12{1'b0}}, frac} << (e - 52));
          has_frac = 1'b0;
        end else begin
          // Fraction straddles binary point: shift right
          sh = 52 - e;
          ip = (64'd1 << e) | (frac >> sh);
          has_frac = |(frac & ((64'd1 << sh) - 1'b1));
        end
      end

      if (!sign) begin
        // Positive or zero: floor is the integer part
        f64_floor_i64 = ip;
      end else if ((x[62:0] == 63'd0)) begin
        // -0.0 閳?0 (negation of zero is zero)
        f64_floor_i64 = 64'd0;
      end else begin
        // Negative with fractional part: floor = -(ip + 1)
        // (2's complement negation of ip+has_frac)
        f64_floor_i64 = ~(ip + (has_frac ? 64'd1 : 64'd0)) + 1'b1;
      end
    end
  endfunction

  function [63:0] f64_i64;
    input [63:0] a;
    reg        neg;
    reg [63:0] abs_val;
    reg [10:0] exp;
    reg [51:0] frac;
    integer    ii;
    integer    pos;
    begin
      if (a == 64'd0) begin
        f64_i64 = 64'd0;
      end else begin
        neg = a[63];
        abs_val = neg ? (~a + 1'b1) : a;
        pos = 63;
        for (ii = 0; ii < 64; ii = ii + 1) begin
          if (abs_val[63 - ii]) begin
            pos = 63 - ii;
            ii = 63;
          end
        end
        exp = 11'd1023 + pos;
        frac = (abs_val << (63 - pos)) >> 11;
        f64_i64 = {neg, exp, frac};
      end
    end
  endfunction

  function f64_ge;
    input [63:0] a;
    input [63:0] b;
    begin
      if (a == b) begin
        f64_ge = 1'b1;
      end else if (a[63] && !b[63]) begin
        f64_ge = 1'b0;
      end else if (!a[63] && b[63]) begin
        f64_ge = 1'b1;
      end else begin
        f64_ge = ({~a[63], a[62:0]} >= {~b[63], b[62:0]});
      end
    end
  endfunction

  function [63:0] f64_uniform01;
    input [51:0] r;
    integer i;
    integer msb;
    reg [10:0] exp;
    reg [51:0] frac;
    reg [51:0] rem;
    begin
      msb = -1;
      for (i = 0; i < 52; i = i + 1) begin
        if (r[i]) begin
          msb = i;
        end
      end

      if (msb < 0) begin
        f64_uniform01 = 64'd0;
      end else begin
        exp = 11'd1023 + msb - 11'd52;
        rem = r ^ (52'd1 << msb);
        frac = rem << (52 - msb);
        f64_uniform01 = {1'b0, exp, frac};
      end
    end
  endfunction

  function [63:0] f64_small_uint;
    input [3:0] k;
    begin
      case (k)
        4'd0: f64_small_uint = 64'h0000000000000000;
        4'd1: f64_small_uint = 64'h3FF0000000000000;
        4'd2: f64_small_uint = 64'h4000000000000000;
        4'd3: f64_small_uint = 64'h4008000000000000;
        4'd4: f64_small_uint = 64'h4010000000000000;
        4'd5: f64_small_uint = 64'h4014000000000000;
        4'd6: f64_small_uint = 64'h4018000000000000;
        4'd7: f64_small_uint = 64'h401C000000000000;
        4'd8: f64_small_uint = 64'h4020000000000000;
        4'd9: f64_small_uint = 64'h4022000000000000;
        4'd10: f64_small_uint = 64'h4024000000000000;
        4'd11: f64_small_uint = 64'h4026000000000000;
        4'd12: f64_small_uint = 64'h4028000000000000;
        4'd13: f64_small_uint = 64'h402A000000000000;
        4'd14: f64_small_uint = 64'h402C000000000000;
        default: f64_small_uint = 64'h402E000000000000;
      endcase
    end
  endfunction

  function [63:0] exp_neg_uint;
    input [3:0] k;
    begin
      case (k)
        4'd0: exp_neg_uint = 64'h3FF0000000000000;
        4'd1: exp_neg_uint = 64'h3FD78B56362CEF38;
        4'd2: exp_neg_uint = 64'h3FC152AAA3BF81CC;
        4'd3: exp_neg_uint = 64'h3FA97DB0CCCEB0AF;
        4'd4: exp_neg_uint = 64'h3F92C155B8213CF4;
        4'd5: exp_neg_uint = 64'h3F7B993FE00D5376;
        4'd6: exp_neg_uint = 64'h3F644E51F113D4D6;
        4'd7: exp_neg_uint = 64'h3F4DE16B9C24A98F;
        4'd8: exp_neg_uint = 64'h3F35FC21041027AD;
        4'd9: exp_neg_uint = 64'h3F202CF22526545A;
        4'd10: exp_neg_uint = 64'h3F07CD79B5647C9A;
        4'd11: exp_neg_uint = 64'h3EF18354238F6764;
        4'd12: exp_neg_uint = 64'h3ED9C54C3B43BC8B;
        4'd13: exp_neg_uint = 64'h3EC2F6053B981D98;
        4'd14: exp_neg_uint = 64'h3EABE6C6FDB01612;
        default: exp_neg_uint = 64'h3E94875CA227EC38;
      endcase
    end
  endfunction

  wire [63:0] ber_floor64 = f64_floor_i64(by);
  wire [63:0] mu_floor_i64 = f64_floor_i64(mu);
  // The FP polynomial path is only a bring-up approximation of Falcon's
  // integer BerExp.  Be conservative for large x; otherwise rare tail
  // candidates are accepted too often and the final norm explodes.
  wire        ber_too_large = (!ber_floor64[63]) && (ber_floor64 > 64'd63);
  wire [3:0]  ber_k = ber_too_large ? 4'd15 : ber_floor64[3:0];
`ifndef SYNTHESIS
  wire        sim_exact_berexp = 1'b1;
`else
  wire        sim_exact_berexp = 1'b0;
`endif

  always @(*) begin
    sz_add_a   = 64'd0;
    sz_add_b   = 64'd0;
    sz_add_sub = 1'b0;
    sz_mul_a   = 64'd0;
    sz_mul_b   = 64'd0;

    case (st)
      SRS: begin
        sz_add_a = mu;
        sz_add_b = rfp;
        sz_add_sub = 1'b1;
      end
      SR1: begin
        sz_add_a = r_frac;
        sz_add_b = F1;
        sz_add_sub = 1'b1;
      end
      SCM: begin
        sz_mul_a = sn_min;
        sz_mul_b = si;
      end
      SZR: begin
        sz_add_a = z_fp;
        sz_add_b = r_frac;
        sz_add_sub = 1'b1;
      end
      SDQ: begin
        sz_mul_a = zmr;
        sz_mul_b = zmr;
      end
      SYM: begin
        sz_mul_a = dsq;
        sz_mul_b = si;
      end
      SYH: begin
        sz_mul_a = by;
        sz_mul_b = si;
      end
      SYQ: begin
        sz_mul_a = by;
        sz_mul_b = FH;
      end
      SZ1: begin
        sz_mul_a = ba;
        sz_mul_b = ba;
      end
      SZ2: begin
        sz_mul_a = ba;
        sz_mul_b = NEG_INV_2SQRSIGMA0;
      end
      SZ2A: begin
        sz_add_a = sz_prod;
        sz_add_b = by;
      end
      SYT: begin
        sz_add_a = by;
        sz_add_b = f64_small_uint(ber_k);
        sz_add_sub = 1'b1;
      end
      SYP0: begin
        sz_mul_a = C4;
        sz_mul_b = by;
      end
      SYP0A: begin
        sz_add_a = sz_prod;
        sz_add_b = C3;
      end
      SYP1: begin
        sz_mul_a = ba;
        sz_mul_b = by;
      end
      SYP1A: begin
        sz_add_a = sz_prod;
        sz_add_b = C2;
      end
      SYP2: begin
        sz_mul_a = ba;
        sz_mul_b = by;
      end
      SYP2A: begin
        sz_add_a = sz_prod;
        sz_add_b = C1;
      end
      SYP3: begin
        sz_mul_a = ba;
        sz_mul_b = by;
      end
      SYP3A: begin
        sz_add_a = sz_prod;
        sz_add_b = F1;
      end
      SYS: begin
        sz_mul_a = ba;
        sz_mul_b = bs;
      end
      SYS2: begin
        sz_mul_a = ba;
        sz_mul_b = ccs;
      end
      default: begin
      end
    endcase
  end

  //===================================================================
  // Sequential block: state register, RNG handshake, datapath writes
  //===================================================================
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      st <= SI; mu <= 0; si <= 0; sn_min <= 0; pm <= 0;
      r_frac <= 0; ccs <= 0; zi <= 0;
      z_fp <= 0; z_out <= 0; zmr <= 0; dsq <= 0;
      by <= 0; x_orig <= 0; sz_prod <= 0; ba <= 0; ru <= 0; rb <= 0; ra <= 0;
      sr <= 0; sp <= 0; rc <= 0;
      zs <= 0; bs_v0 <= 0; bs_v1 <= 0; bs_v2 <= 0; sc <= 0; bs <= 0; bn <= 0; rfp <= 0; rint <= 0;
      ccs_cache_valid <= 0; ccs_cache_si <= 0; ccs_cache_value <= 0;
      debug_cmd_attempts <= 0; debug_cmd_rejects <= 0; debug_cmd_accepts <= 0;
      rsp_valid <= 0; rsp_z0 <= 0; rsp_z1 <= 0;
      rsp_accept <= 0; rsp_status <= 0; done <= 0; fail <= 0;
      rng_req <= 0;
    end else begin
      st <= sn;

      // RNG auto-refill request
      if (rl) begin rng_req <= 1; sr <= 1; end

      // RNG data arrival: load 256-bit buffer, reset available count
      if (rng_ack) begin rb <= rng_data; ra <= 8'd255; rng_req <= 0; sr <= 0; end

      case (st)
        // SI: latch command inputs, clear output flags
        SI: begin
          rsp_valid <= 0; done <= 0; fail <= 0; rc <= 0; sp <= 0;
          if (cmd_valid) begin
            mu <= cmd_mu; si <= cmd_sigma_inv;
            sn_min <= cmd_sigma_min; pm <= cmd_pair_mode;
            debug_cmd_attempts <= 0;
            debug_cmd_rejects <= 0;
            debug_cmd_accepts <= 0;
            if (ccs_cache_valid && (ccs_cache_si == cmd_sigma_inv)) begin
              ccs <= ccs_cache_value;
            end
`ifndef SYNTHESIS
            if (debug_return_mu) begin
              rsp_z0 <= cmd_mu; rsp_z1 <= cmd_mu; rsp_accept <= 1;
            end
`endif
          end
        end

        // SRR/SRW: RNG request / wait
        SRR: begin rng_req <= 1; sr <= 1; end
        SRW: if (rng_ack) begin rb <= rng_data; ra <= 8'd255; rng_req <= 0; sr <= 0; end

        // Setup phase
        SRF: begin
          rint <= mu_floor_i64;
          rfp  <= f64_i64(mu_floor_i64);         // floor(mu) as FP64, without a separate FPU op
        end
        SRI: rfp <= f64_i64(rfp);                 // legacy state, normally skipped
        SRS: r_frac <= sz_add_y;                  // r_frac = mu - floor(mu)
        SRC: begin end                             // NOP: branch point
        SR1: r_frac <= sz_add_y;                  // r_frac = r_frac - 1.0
        SCM: begin                                // ccs = sigma_min * sigma_inv
          ccs <= sz_mul_y;
          ccs_cache_valid <= 1'b1;
          ccs_cache_si <= si;
          ccs_cache_value <= sz_mul_y;
        end

        // Base sampler: consume 80 random bits, matching the reference
        // sequence prng_get_u64(), prng_get_u8(), prng_get_u8()&1.
        SSB: begin
          debug_cmd_attempts <= debug_cmd_attempts + 1'b1;
          bs_v0 <= rb[23:0];
          bs_v1 <= rb[47:24];
          bs_v2 <= rb[71:48];
          zs <= rb[72];
          sc <= bs_now;
          zi <= rb[72] ? ({11'd0, bs_now} + 16'd1)
                       : (-{11'd0, bs_now});
          rb <= {80'd0, rb[255:80]};
          ra <= ra - 8'd80;
        end

        SSS: begin end

        // Rejection pipeline
        SSI: begin
          z_fp  <= f64_i64({{48{zi[15]}}, zi});   // int_to_float(zi)
          z_out <= f64_i64(rint + {{48{zi[15]}}, zi});
        end
        SZA: z_out <= sz_add_y;                   // legacy FP add path, normally skipped
        SZR: zmr <= sz_add_y;                     // z_fp - r_frac
        SDQ: dsq <= sz_mul_y;                     // zmr * zmr
        SYM: by <= sz_mul_y;                      // dsq * sigma_inv
        SYH: by <= sz_mul_y;                      // by * sigma_inv
        SYQ: by <= sz_mul_y;                      // by * 0.5

        // z0 correction: subtract z0铏?(2锜絖max铏? to account for fixed base-sampler sigma
        // z0 = sc (CDT index). Uses ba as temporary (free before polynomial eval).
        SZ0: ba <= f64_i64({59'd0, sc});          // ba = int_to_float(sc)
        SZ1: ba <= sz_mul_y;                      // ba = ba * ba = z0^2
        SZ2: sz_prod <= sz_mul_y;                 // first half of local FMA
        SZ2A: by <= sz_add_y;                     // by = by - z0^2/(2*sigma_max^2)

        // BerExp range reduction
        SYR: begin
          x_orig <= by;
          if (ber_too_large) begin
            // x > 2.0: rejection probability < exp(-2) 閳?0.135 閳?instant reject
            rsp_accept <= 0;
          end else begin
            bn <= |ber_k; bs <= exp_neg_uint(ber_k);
            if (1'b0) begin
              // x 閳?[2.0, 4.0): subtract 2 or 3 depending on by[51]
              bn <= by[51] ? 3 : 2;
              bs <= by[51] ? EN3 : EN2;
            end else if (by[62:52] == 11'd1023) begin
              // x 閳?[1.0, 2.0): subtract 1
              bn <= 1;
              bs <= EN1;
            end
          end
        end
        SYT: by <= sz_add_y;                      // latch range-reduced by

        // Horner evaluation of exp(-x) Taylor polynomial
        SYP0: sz_prod <= sz_mul_y;
        SYP0A: ba <= sz_add_y;                    // C4*by + C3
        SYP1: sz_prod <= sz_mul_y;
        SYP1A: ba <= sz_add_y;                    // ba*by + C2
        SYP2: sz_prod <= sz_mul_y;
        SYP2A: ba <= sz_add_y;                    // ba*by + C1
        SYP3: sz_prod <= sz_mul_y;
        SYP3A: ba <= sz_add_y;                    // ba*by + 1.0
        SYS:  ba <= sz_mul_y;                     // ba * bs
        SYS2: ba <= sz_mul_y;                     // ba * ccs

        // Generate uniform random float ru 閳?[0,1) from RNG buffer
        SYC: begin
          ru <= f64_uniform01(rb[51:0]);
          rb <= {52'd0, rb[255:52]}; ra <= ra - 8'd52;
        end

        // Bernoulli comparison: accept if ba > ru
        SY2: begin
          if (sim_exact_berexp) begin
`ifndef SYNTHESIS
            rsp_accept <= (($exp(-$bitstoreal(x_orig)) * $bitstoreal(ccs)) > $bitstoreal(ru));
`else
            rsp_accept <= 1'b0;
`endif
          end else begin
            rsp_accept <= f64_ge(ba, ru);
          end
        end

        // Result dispatch
        SCH: begin
          if (rsp_accept) begin
            // Accepted: store z_out to rsp_z0 (first) or rsp_z1 (second in pair)
            debug_cmd_accepts <= debug_cmd_accepts + 1'b1;
            if (sp == 0) rsp_z0 <= z_out; else rsp_z1 <= z_out;
          end else begin
            debug_cmd_rejects <= debug_cmd_rejects + 1'b1;
            rc <= rc + 1;
            if (rc == 99) begin fail <= 1; rsp_status <= 8'hFF; end
          end
        end
        SNS: begin sp <= 1; rc <= 0; end            // pair mode: start second sample
        SDO: begin
          rsp_valid <= 1; done <= 1;                 // normal completion
`ifndef SYNTHESIS
          if (debug_trace_sampler && (debug_sample_count < 64)) begin
            $display("  SZ_DONE[%0d] mu=%016x si=%016x z0=%016x diff=%0.6f rc=%0d",
                     debug_sample_count, mu, si, rsp_z0,
                     ($bitstoreal(rsp_z0) - $bitstoreal(mu)), rc);
            debug_sample_count = debug_sample_count + 1;
          end
`endif
          if (rsp_valid && rsp_ready) rsp_valid <= 0;
        end
        SFA: begin
          rsp_valid <= 1; done <= 1; fail <= 1;      // failure (99 retries exhausted)
          if (rsp_valid && rsp_ready) rsp_valid <= 0;
        end
        default: st <= SI;
      endcase
    end
  end

  //===================================================================
  // Combinational next-state logic
  //===================================================================
  always @(*) begin
    sn = st;
    case (st)
      SI: if (cmd_valid) begin
`ifndef SYNTHESIS
        if (debug_return_mu) sn = SDO; else
`endif
        sn = SRR;
      end
      SRR: if (sr)   sn = SRW;
      SRW: if (!rng_req) sn = SRF;
      // Setup pipeline
      SRF: sn = SRS;
      SRI: sn = SRS;
      SRS: sn = SRC;
      SRC: if (r_frac == 64'd0 || !r_frac[63]) begin
             sn = ccs_cache_hit ? SSB : SCM;  // r_frac >= 0: normal path
           end else sn = SR1;                 // r_frac < 0: safety path
      SR1: sn = SCM;
      SCM: sn = SSB;
      // Base sampler
      SSB: sn = SSS;
      SSS: sn = SSI;
      // Rejection pipeline
      SSI: sn = SZR;
      SZA: sn = SZR;
      SZR: sn = SDQ;
      SDQ: sn = SYM;
      SYM: sn = SYH;
      SYH: sn = SYQ;
      SYQ: sn = SZ0;
      // z0 correction: subtract z0铏?(2锜絖max铏? from rejection exponent
      SZ0: sn = SZ1;
      SZ1: sn = SZ2;
      SZ2: sn = SZ2A;
      SZ2A: sn = SYR;
      // BerExp: range-reduction check
      SYR: if (ber_too_large) sn = SCH;              // instant reject
           else if (by[62:52] < 11'd1023) sn = SYP0;        // x < 1.0 閳?poly directly
           else sn = SYT;                                    // x 閳?[1,4) 閳?range-reduce
      SYT: sn = SYP0;
      // Horner evaluation
      SYP0: sn = SYP0A;
      SYP0A: sn = SYP1;
      SYP1: sn = SYP1A;
      SYP1A: sn = SYP2;
      SYP2: sn = SYP2A;
      SYP2A: sn = SYP3;
      SYP3: sn = SYP3A;
      SYP3A: sn = SYS;
      SYS:  sn = SYS2;
      SYS2: sn = SYC;
      SYC:  sn = SY2;                                        // NOP: random float ready
      SY2:  sn = SCH;
      // Result dispatch
      SCH: if (fail) sn = SFA;
           else if (rsp_accept) begin
             if (pm && !sp) sn = SNS;                         // pair mode: do second sample
             else sn = SDO;                                   // done
           end else sn = SSB;                                 // reject: retry
      SNS: sn = SSB;                                          // loop back for second sample
      SDO: if (rsp_valid && rsp_ready) sn = SI;
      SFA: if (rsp_valid && rsp_ready) sn = SI;
      default: sn = SI;
    endcase
  end

  //===================================================================
  // Shared FPU request dispatch
  //===================================================================
  always @(*) begin
    // SamplerZ now uses the local add/mul datapath for its staged arithmetic.
    // The shared FPU port is kept idle so the parent ffSampling scheduler is
    // not blocked by SamplerZ internal pipeline states.
    fpu_req_valid = (st == SI) ? 1'b0 : 1'b0;
    fpu_req_op    = FA;
    fpu_req_a     = 64'd0;
    fpu_req_b     = 64'd0;
    fpu_req_c     = 64'd0;
  end

endmodule
