//=============================================================================
// Falcon ffSampling (SampleZ) — discrete Gaussian sampler over integers.
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
//   cmd_mu        — distribution center µ (FP64)
//   cmd_sigma_inv — 1/σ (FP64)
//   cmd_sigma_min — σ_min for security lower bound (FP64)
//   cmd_pair_mode — 1 = sample two independent z values in one command
//   rsp_z0, rsp_z1 — sampled integers as FP64 (z1 valid only in pair mode)
//   rsp_accept     — 1 = sample accepted, 0 = rejected (retry loop)
//   rsp_status     — 0xFF on failure (99 rejections exhausted)
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
  localparam[3:0] FA  = 0,   // FADD  — a + b
                  FS  = 1,   // FSUB  — a - b
                  FM  = 2,   // FMUL  — a * b
                  FMA = 3,   // FMADD — a * b + c
                  FC  = 9,   // FCVT / compare
                  FF  = 12,  // FFLOOR
                  FI  = 13;  // FINT-TO-FLOAT

  // Taylor coefficients for exp(-x) ≈ 1 - x + x²/2 - x³/6 + x⁴/24
  // Evaluated as Horner: (((C4*x + C3)*x + C2)*x + C1)*x + 1.0
  localparam[63:0] C1 = 64'hBFF0000000000000,  // -1.0
                   C2 = 64'h3FE0000000000000,  //  0.5
                   C3 = 64'hBFC5555555555555,  // -1/6  ≈ -0.1666667
                   C4 = 64'h3FA5555555555555,  //  1/24 ≈  0.0416667
                   F1 = 64'h3FF0000000000000,  //  1.0
                   FH = 64'h3FE0000000000000,  //  0.5
  // Range-reduction constants: ENn = exp(-n)
                   EN1 = 64'h3FD78B56362CEF38, // exp(-1) ≈ 0.3679
                   EN2 = 64'h3FC152AAEE5FEA2E, // exp(-2) ≈ 0.1353
                   EN3 = 64'h3FA982C2EB92860E, // exp(-3) ≈ 0.0498
  // z0 correction: NEG_INV_2SQRSIGMA0 = -1/(2*sigma_max^2) = -1/(2*1.8205^2)
  // Used in FMA to subtract z0²/(2σ_max²) from rejection exponent
                   NEG_INV_2SQRSIGMA0 = 64'hBFC34F8BC183BBC2;

  //-----------------------------------------------------------------
  // FSM state encoding — Algorithm 12 (ffSampling) step-by-step
  //
  // Setup phase:
  //   SI:  idle, wait for command
  //   SRR: request 256-bit RNG word
  //   SRW: wait for RNG acknowledge
  //   SRF: rfp = floor(mu) via hardware function f64_floor_i64
  //   SRI: rfp = int_to_float(rfp)          (FI op)
  //   SRS: r_frac = mu - rfp                (FS op)  fractional part ∈ [0,1)
  //   SRC: branch on r_frac sign (always non-negative in practice)
  //   SR1: r_frac = r_frac - 1.0            (FS op)  unreachable safety path
  //   SCM: ccs = sigma_min * sigma_inv       (FM op)  = σ_min/σ
  //
  // Base sampler phase (CDT scan):
  //   SSB: extract zs=sign-bit(b), ut=16-bit uniform, sc=0 from RNG buffer
  //   SSS: CDT scan loop — find smallest sc where CDT[sc] >= ut, or sc=15
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
  //   SYQ: by  = by  * 0.5                   (FM op)  → (z-r)²/(2σ²)
  //
  // z0 correction — subtracts base-sampler sigma_max bias from rejection:
  //   SZ0: ba = int_to_float(sc)              (FI op)  sc = z0 = |zi|
  //   SZ1: ba = ba * ba                       (FM op)  = z0²
  //   SZ2: by = ba * NEG_INV + by            (FMA op)  by -= z0²/(2σ_max²)
  //
  // BerExp — Bernoulli experiment for exp(-x):
  //   SYR: check by exponent:
  //          > 1024 (by>2.0) → instant reject
  //          ==1024 (by∈[2,4)) → fire FSUB to range-reduce (subtract 2 or 3)
  //          ==1023 (by∈[1,2)) → fire FSUB to range-reduce (subtract 1)
  //          < 1023 (by<1.0)  → skip FSUB, go to polynomial eval
  //        Also sets bs = exp(-n) for the subtracted amount.
  //   SYT: by = FSUB_result  (latch range-reduced by)
  //   SYP0: ba = C4*by + C3                  (FMA op)
  //   SYP1: ba = ba*by + C2                  (FMA op)
  //   SYP2: ba = ba*by + C1                  (FMA op)
  //   SYP3: ba = ba*by + 1.0                 (FMA op)  Horner done: ba ≈ exp(-x)
  //   SYS:  ba = ba * bs                     (FM op)   apply range-reduction scale
  //   SYC:  ru = uniform random float ∈ [0,1) from RNG buffer
  //   SY2:  rsp_accept = (ba > ru)           (FC op)   Bernoulli comparison
  //
  // Result dispatch:
  //   SCH: if accepted → store z_out into rsp_z0/z1
  //        if rejected → rc++, retry from SSB (fail at 99 rejections)
  //   SNS: switch to second sample (sp=1), loop to SSB (pair mode)
  //   SDO: drive rsp_valid, done
  //   SFA: drive rsp_valid, done, fail
  //-----------------------------------------------------------------
  localparam[5:0] SI=0,SRR=1,SRW=2,SRF=3,SRI=4,SRS=5,
    SRC=6,SR1=7,SCM=8,SSB=9,SSS=29,SSI=10,SZR=11,SDQ=12,SZA=30,
    SYM=13,SYH=14,SYR=25,SYP0=15,SYP1=16,SYP2=17,SYP3=18,SYS=28,
    SYC=19,SY2=20,SCH=21,SNS=22,SDO=23,SFA=24,SYQ=31,SYT=32,
    SZ0=26,SZ1=27,SZ2=33;

  //-----------------------------------------------------------------
  // Internal registers
  //-----------------------------------------------------------------
  reg[5:0]  st, sn;             // FSM state, next-state
  // Command inputs (latched at SI)
  reg[63:0] mu,                  // µ: distribution center (FP64)
            si,                  // 1/σ: inverse of standard deviation (FP64)
            sn_min;              // σ_min: security lower bound on sigma (FP64)
  reg       pm;                  // pair-mode flag: 1 = produce two samples
  // Computation pipeline
  reg[63:0] rfp;                 // floor(µ): integer part (from f64_floor_i64), then as FP64
  reg[63:0] r_frac;              // µ - floor(µ): fractional part ∈ [0, 1)
  reg[63:0] ccs;                 // σ_min / σ (computed but unused in current rejection path)
  reg[15:0] zi;                  // sampled integer offset z (before adding floor(µ))
  reg[63:0] z_fp,                // float(zi)
            z_out;               // float(zi) + float(floor(µ)) = accepted sample as FP64
  reg[63:0] zmr,                 // z_fp - r_frac = (z - r_frac)
            dsq;                 // zmr^2 = (z - r_frac)^2
  reg[63:0] by,                  // x = (z-r_frac)²/(2σ²) — input to BerExp
            ba,                  // polynomial result ≈ exp(-x)
            bs;                  // scale factor: exp(-n) from range reduction (or 1.0)
  reg       bn;                  // number of integer range-reduction steps (1..3)
  reg[63:0] ru;                  // uniform random float comparator ∈ [0, 1)
  // Base sampler (CDT scan) state
  reg[15:0] ut;                  // 16-bit uniform random value for CDT comparison
  reg[3:0]  sc;                  // CDT table scan index (doubles as z0 = |zi|)
  reg       zs;                  // sign bit from RNG (b in spec: 1=positive, 0=negative)
  // RNG buffer and control
  reg[255:0] rb;                 // 256-bit random buffer (shift register)
  reg[7:0]  ra;                  // remaining random bits in rb (< 32 triggers auto-refill)
  reg       sr;                  // RNG request pending flag
  // Retry / pair-mode control
  reg[7:0]  rc;                  // consecutive rejection counter (fail at 99)
  reg       sp;                  // second-sample-produced flag (pair mode)

  wire[15:0] cd;                 // CDT table output for current sc
  wire frf = fpu_rsp_valid && fpu_rsp_ready;  // FPU result fire (handshake complete)

  // FPU interface: always ready, format=0 (FP64), round-to-nearest, no fcvt
  assign fpu_rsp_ready = 1'b1;
  assign fpu_req_fmt   = 2'd0;
  assign fpu_req_rm    = 3'd0;
  assign fpu_req_fcvt_op = 2'd0;

  assign cmd_ready = (st == SI);
  assign busy = (st != SI && st != SDO && st != SFA);

  // Auto-refill: request new RNG word when fewer than 32 useful bits remain.
  // Each SSB consumes 17 bits (16 for ut + 1 for zs), SYC consumes 52 bits.
  // Suppressed during SI, SDO, SFA, SRR, SRW states.
  wire rl = (ra < 8'd32) && !rng_req && !sr
         && (st != SI) && (st != SDO) && (st != SFA)
         && (st != SRR) && (st != SRW);

  // CDT ROM: 16-entry CDF table for sigma_max=1.8205 half-Gaussian
  falconsign_bs_cdt_rom #(.ADDR_W(4)) uu(.addr(sc), .data(cd));

`ifndef SYNTHESIS
  // Debug: when +FS_SAMPLE_MU is set, return mu directly as samples
  // (bypasses the Gaussian sampler — useful for testing caller logic)
  reg debug_return_mu;
  initial begin
    debug_return_mu = $test$plusargs("FS_SAMPLE_MU");
  end
`endif

  //===================================================================
  // f64_floor_i64: software-style FP64 floor returning 64-bit integer.
  //
  // Decomposes IEEE-754 FP64 into sign, exponent, fraction, then shifts
  // the mantissa to extract the integer part. Handles denormals (exp==0),
  // fractional-only values (|x|<1 → 0), and overflow (exp >= 1086).
  //
  // For negative non-integer values, returns the floor toward -∞
  // (e.g., floor(-1.5) = -2), matching C's fpr_floor().
  //
  // Uses combinational logic only — no FPU involvement, single-cycle.
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
        // -0.0 → 0 (negation of zero is zero)
        f64_floor_i64 = 64'd0;
      end else begin
        // Negative with fractional part: floor = -(ip + 1)
        // (2's complement negation of ip+has_frac)
        f64_floor_i64 = ~(ip + (has_frac ? 64'd1 : 64'd0)) + 1'b1;
      end
    end
  endfunction

  //===================================================================
  // Sequential block: state register, RNG handshake, datapath writes
  //===================================================================
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      st <= SI; mu <= 0; si <= 0; sn_min <= 0; pm <= 0;
      r_frac <= 0; ccs <= 0; zi <= 0;
      z_fp <= 0; z_out <= 0; zmr <= 0; dsq <= 0;
      by <= 0; ba <= 0; ru <= 0; rb <= 0; ra <= 0;
      sr <= 0; sp <= 0; rc <= 0;
      zs <= 0; ut <= 0; sc <= 0; bs <= 0; bn <= 0; rfp <= 0;
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
        SRF: rfp <= f64_floor_i64(mu);           // floor(mu) — hardware function
        SRI: if (frf) rfp <= fpu_rsp_result;      // int_to_float(floor(mu))
        SRS: if (frf) r_frac <= fpu_rsp_result;   // r_frac = mu - floor(mu)
        SRC: begin end                             // NOP: branch point
        SR1: if (frf) r_frac <= fpu_rsp_result;   // r_frac = r_frac - 1.0
        SCM: if (frf) ccs <= fpu_rsp_result;       // ccs = sigma_min * sigma_inv

        // Base sampler: consume 17 random bits (16 ut + 1 zs)
        SSB: begin
          ut <= rb[15:0]; zs <= rb[16]; sc <= 0;
          rb <= {17'd0, rb[255:17]}; ra <= ra - 8'd17;
        end

        // CDT scan: iterate sc until CDT[sc] >= ut, or sc saturates at 15
        // zi = zs ? (sc+1) : (-sc)  matches C: z = b + ((b<<1)-1)*z0
        SSS: begin
          if (cd >= ut || sc == 15)
            zi <= zs ? ({12'd0, sc} + 16'd1) : (-{12'd0, sc});
          else
            sc <= sc + 1'd1;
        end

        // Rejection pipeline
        SSI: if (frf) z_fp <= fpu_rsp_result;     // int_to_float(zi)
        SZA: if (frf) z_out <= fpu_rsp_result;    // z_fp + floor(mu)
        SZR: if (frf) zmr <= fpu_rsp_result;       // z_fp - r_frac
        SDQ: if (frf) dsq <= fpu_rsp_result;       // zmr * zmr
        SYM: if (frf) by <= fpu_rsp_result;        // dsq * sigma_inv
        SYH: if (frf) by <= fpu_rsp_result;        // by  * sigma_inv
        SYQ: if (frf) by <= fpu_rsp_result;        // by  * 0.5  → (z-r)²/(2σ²)

        // z0 correction: subtract z0²/(2σ_max²) to account for fixed base-sampler sigma
        // z0 = sc (CDT index). Uses ba as temporary (free before polynomial eval).
        SZ0: if (frf) ba <= fpu_rsp_result;        // ba = int_to_float(sc)
        SZ1: if (frf) ba <= fpu_rsp_result;        // ba = ba * ba = z0²
        SZ2: if (frf) by <= fpu_rsp_result;        // by = by - z0²/(2σ_max²)

        // BerExp range reduction
        SYR: begin
          if (by[62:52] > 11'd1024) begin
            // x > 2.0: rejection probability < exp(-2) ≈ 0.135 — instant reject
            rsp_accept <= 0;
          end else begin
            bn <= 0; bs <= F1;   // default: no range reduction
            if (by[62:52] == 11'd1024) begin
              // x ∈ [2.0, 4.0): subtract 2 or 3 depending on by[51]
              bn <= by[51] ? 3 : 2;
              bs <= by[51] ? EN3 : EN2;
            end else if (by[62:52] == 11'd1023) begin
              // x ∈ [1.0, 2.0): subtract 1
              bn <= 1;
              bs <= EN1;
            end
          end
        end
        SYT: if (frf) by <= fpu_rsp_result;        // latch range-reduced by

        // Horner evaluation of exp(-x) Taylor polynomial
        SYP0: if (frf) ba <= fpu_rsp_result;       // C4*by + C3
        SYP1: if (frf) ba <= fpu_rsp_result;       // ba*by + C2
        SYP2: if (frf) ba <= fpu_rsp_result;       // ba*by + C1
        SYP3: if (frf) ba <= fpu_rsp_result;       // ba*by + 1.0
        SYS:  if (frf) ba <= fpu_rsp_result;       // ba * bs (apply range-reduction scale)

        // Generate uniform random float ru ∈ [0,1) from RNG buffer
        // Constructs FP64 with biased exponent 1021 or 1022 and 52 random mantissa bits.
        // Consumes 52 random bits.
        SYC: begin
          ru <= {1'b0, rb[51] ? 11'd1022 : 11'd1021, rb[50:0], 1'b0};
          rb <= {52'd0, rb[255:52]}; ra <= ra - 8'd52;
        end

        // Bernoulli comparison: accept if ba > ru
        SY2: if (frf) rsp_accept <= (fpu_rsp_result != 64'd0);

        // Result dispatch
        SCH: begin
          if (rsp_accept) begin
            // Accepted: store z_out to rsp_z0 (first) or rsp_z1 (second in pair)
            if (sp == 0) rsp_z0 <= z_out; else rsp_z1 <= z_out;
          end else begin
            rc <= rc + 1;
            if (rc == 99) begin fail <= 1; rsp_status <= 8'hFF; end
          end
        end
        SNS: begin sp <= 1; rc <= 0; end            // pair mode: start second sample
        SDO: begin
          rsp_valid <= 1; done <= 1;                 // normal completion
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
      SRF: sn = SRI;
      SRI: if (frf) sn = SRS;
      SRS: if (frf) sn = SRC;
      SRC: if (r_frac == 64'd0 || !r_frac[63]) sn = SCM;  // r_frac >= 0 → normal path
           else sn = SR1;                                   // r_frac < 0 → safety path
      SR1: if (frf) sn = SCM;
      SCM: if (frf) sn = SSB;
      // Base sampler
      SSB: sn = SSS;
      SSS: if (cd >= ut || sc == 15) sn = SSI;              // CDT match found
           else sn = SSS;                                    // continue scan
      // Rejection pipeline
      SSI: if (frf) sn = SZA;
      SZA: if (frf) sn = SZR;
      SZR: if (frf) sn = SDQ;
      SDQ: if (frf) sn = SYM;
      SYM: if (frf) sn = SYH;
      SYH: if (frf) sn = SYQ;
      SYQ: if (frf) sn = SZ0;
      // z0 correction: subtract z0²/(2σ_max²) from rejection exponent
      SZ0: if (frf) sn = SZ1;
      SZ1: if (frf) sn = SZ2;
      SZ2: if (frf) sn = SYR;
      // BerExp: range-reduction check
      SYR: if (by[62:52] > 11'd1024) sn = SCH;              // instant reject
           else if (by[62:52] < 11'd1023) sn = SYP0;        // x < 1.0 → poly directly
           else sn = SYT;                                    // x ∈ [1,4) → range-reduce
      SYT: if (frf) sn = SYP0;
      // Horner evaluation
      SYP0: if (frf) sn = SYP1;
      SYP1: if (frf) sn = SYP2;
      SYP2: if (frf) sn = SYP3;
      SYP3: if (frf) sn = SYS;
      SYS:  if (frf) sn = SYC;
      SYC:  sn = SY2;                                        // NOP: random float ready
      SY2:  if (frf) sn = SCH;
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
  // FPU request dispatch — issues one FPU operation per state
  //===================================================================
  always @(*) begin
    fpu_req_valid = 0; fpu_req_op = FA; fpu_req_a = 0; fpu_req_b = 0; fpu_req_c = 0;
    case (st)
      // Setup
      SRI: begin fpu_req_valid = 1; fpu_req_op = FI; fpu_req_a = rfp; end
      SRS: begin fpu_req_valid = 1; fpu_req_op = FS; fpu_req_a = mu;  fpu_req_b = rfp; end
      SR1: begin fpu_req_valid = 1; fpu_req_op = FS; fpu_req_a = r_frac; fpu_req_b = F1; end
      SCM: begin fpu_req_valid = 1; fpu_req_op = FM; fpu_req_a = sn_min; fpu_req_b = si; end
      // Rejection pipeline
      SSI: begin fpu_req_valid = 1; fpu_req_op = FI; fpu_req_a = {{48{zi[15]}}, zi}; end
      SZA: begin fpu_req_valid = 1; fpu_req_op = FA; fpu_req_a = z_fp; fpu_req_b = rfp; end
      SZR: begin fpu_req_valid = 1; fpu_req_op = FS; fpu_req_a = z_fp; fpu_req_b = r_frac; end
      SDQ: begin fpu_req_valid = 1; fpu_req_op = FM; fpu_req_a = zmr; fpu_req_b = zmr; end
      SYM: begin fpu_req_valid = 1; fpu_req_op = FM; fpu_req_a = dsq; fpu_req_b = si; end
      SYH: begin fpu_req_valid = 1; fpu_req_op = FM; fpu_req_a = by;  fpu_req_b = si; end
      SYQ: begin fpu_req_valid = 1; fpu_req_op = FM; fpu_req_a = by;  fpu_req_b = FH; end
      // z0 correction: subtract z0²/(2σ_max²) from by = (z-r)²/(2σ²)
      // Uses FMA: by + ba * NEG_INV = by - z0² * INV_2SQRSIGMA0
      SZ0: begin fpu_req_valid = 1; fpu_req_op = FI; fpu_req_a = {{60{1'b0}}, sc}; end
      SZ1: begin fpu_req_valid = 1; fpu_req_op = FM; fpu_req_a = ba;  fpu_req_b = ba; end
      SZ2: begin fpu_req_valid = 1; fpu_req_op = FMA; fpu_req_a = ba; fpu_req_b = NEG_INV_2SQRSIGMA0; fpu_req_c = by; end
      // BerExp range reduction:
      //   by ∈ [1.0, 2.0) → subtract 1.0  (bs = exp(-1))
      //   by ∈ [2.0, 3.0] → subtract 2.0  (bs = exp(-2))
      //   by ∈ (3.0, 4.0) → subtract 3.0  (bs = exp(-3))
      SYR: if (by[62:52] >= 11'd1023 && by[62:52] <= 11'd1024) begin
        fpu_req_valid = 1; fpu_req_op = FS; fpu_req_a = by;
        fpu_req_b = (by[62:52] == 11'd1024)
                    ? (by[51] ? 64'h4008000000000000   // 3.0
                              : 64'h4000000000000000)  // 2.0
                    : 64'h3FF0000000000000;             // 1.0
      end
      // Horner evaluation: exp(-x) ≈ (((C4*x + C3)*x + C2)*x + C1)*x + 1.0
      SYP0: begin fpu_req_valid = 1; fpu_req_op = FMA; fpu_req_a = C4; fpu_req_b = by; fpu_req_c = C3; end
      SYP1: begin fpu_req_valid = 1; fpu_req_op = FMA; fpu_req_a = ba; fpu_req_b = by; fpu_req_c = C2; end
      SYP2: begin fpu_req_valid = 1; fpu_req_op = FMA; fpu_req_a = ba; fpu_req_b = by; fpu_req_c = C1; end
      SYP3: begin fpu_req_valid = 1; fpu_req_op = FMA; fpu_req_a = ba; fpu_req_b = by; fpu_req_c = F1; end
      SYS:  begin fpu_req_valid = 1; fpu_req_op = FM;  fpu_req_a = ba; fpu_req_b = bs; end
      // Bernoulli compare: rsp_accept = (ba > ru)
      SY2:  begin fpu_req_valid = 1; fpu_req_op = FC;  fpu_req_a = ba; fpu_req_b = ru; end
      default: ;
    endcase
  end

endmodule
