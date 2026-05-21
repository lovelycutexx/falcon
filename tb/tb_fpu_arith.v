`timescale 1ns/1ps
// Targeted hardware FPU arithmetic test: FMADD, FNMADD, FADD, FSUB, FMUL.

module tb_fpu_arith;
    reg clk, rst_n;

    reg         req_valid;
    wire        req_ready;
    reg  [3:0]  req_op;
    reg  [63:0] req_a, req_b, req_c;
    reg  [1:0]  req_fmt;
    reg  [2:0]  req_rm;

    wire        rsp_valid;
    reg         rsp_ready;
    wire [63:0] rsp_result;
    wire [4:0]  rsp_flags;

    falcon_fp_fpu u_fpu (
        .clk(clk), .rst_n(rst_n),
        .req_valid(req_valid), .req_ready(req_ready),
        .req_op(req_op), .req_a(req_a), .req_b(req_b), .req_c(req_c),
        .req_fmt(req_fmt), .req_rm(req_rm), .req_fcvt_op(2'b00),
        .rsp_valid(rsp_valid), .rsp_ready(rsp_ready),
        .rsp_result(rsp_result), .rsp_flags(rsp_flags),
        .busy()
    );

    always #5 clk = ~clk;

    integer pass, fail;
    reg [63:0] result_q;
    reg [4:0]  flags_q;

    task send_fpu_op;
        input [3:0] op;
        input [63:0] a, b, c;
        begin
            @(posedge clk);
            req_valid <= 1'b1;
            req_op    <= op;
            req_a     <= a;
            req_b     <= b;
            req_c     <= c;
            req_fmt   <= 2'b01;  // FP64
            req_rm    <= 3'b000; // RNE
            rsp_ready <= 1'b1;
            @(posedge clk);
            req_valid <= 1'b0;
            // FPU pipelined: wait for rsp_valid, capture result
            @(posedge clk);
            while (!rsp_valid) @(posedge clk);
            // result is available on this cycle; capture it
            result_q = rsp_result;
            flags_q  = rsp_flags;
            @(posedge clk);
        end
    endtask

    task check_f64;
        input [63:0] got;
        input [63:0] exp;
        input [120*8:1] msg;
        begin
            if (got === exp) begin
                $display("  PASS: %0s", msg);
                pass = pass + 1;
            end else begin
                $display("  FAIL: %0s", msg);
                $display("    got  %016x  (%e)", got, $bitstoreal(got));
                $display("    exp  %016x  (%e)", exp, $bitstoreal(exp));
                fail = fail + 1;
            end
        end
    endtask

    localparam [3:0] FADD   = 4'd0;
    localparam [3:0] FSUB   = 4'd1;
    localparam [3:0] FMUL   = 4'd2;
    localparam [3:0] FMADD  = 4'd3;
    localparam [3:0] FNMADD = 4'd6;

    initial begin
        clk = 0; rst_n = 0; req_valid = 0;
        pass = 0; fail = 0;

        #20 rst_n = 1; #10;

        $display("=== FPU Arithmetic Test ===\n");

        // ─── Test 1: Basic FADD ───
        $display("-- FADD --");
        send_fpu_op(FADD, $realtobits(1.0), $realtobits(2.0), 0);
        check_f64(result_q, $realtobits(3.0), "1 + 2 = 3");

        send_fpu_op(FADD, $realtobits(-1.5), $realtobits(2.5), 0);
        check_f64(result_q, $realtobits(1.0), "-1.5 + 2.5 = 1");

        send_fpu_op(FADD, $realtobits(1e10), $realtobits(1.0), 0);
        check_f64(result_q, $realtobits(1e10 + 1.0), "1e10 + 1 ≈ 1e10+1");

        send_fpu_op(FADD, $realtobits(100.0), $realtobits(-100.0), 0);
        check_f64(result_q, $realtobits(0.0), "100 + (-100) = 0");

        // ─── Test 2: Basic FSUB ───
        $display("\n-- FSUB --");
        send_fpu_op(FSUB, $realtobits(5.0), $realtobits(3.0), 0);
        check_f64(result_q, $realtobits(2.0), "5 - 3 = 2");

        send_fpu_op(FSUB, $realtobits(1.0), $realtobits(1.0), 0);
        check_f64(result_q, $realtobits(0.0), "1 - 1 = 0");

        send_fpu_op(FSUB, $realtobits(3.0), $realtobits(5.0), 0);
        check_f64(result_q, $realtobits(-2.0), "3 - 5 = -2");

        // ─── Test 3: Basic FMUL ───
        $display("\n-- FMUL --");
        send_fpu_op(FMUL, $realtobits(2.0), $realtobits(3.0), 0);
        check_f64(result_q, $realtobits(6.0), "2 * 3 = 6");

        send_fpu_op(FMUL, $realtobits(-2.0), $realtobits(3.0), 0);
        check_f64(result_q, $realtobits(-6.0), "-2 * 3 = -6");

        send_fpu_op(FMUL, $realtobits(1.5), $realtobits(2.0), 0);
        check_f64(result_q, $realtobits(3.0), "1.5 * 2 = 3");

        send_fpu_op(FMUL, $realtobits(0.0), $realtobits(100.0), 0);
        check_f64(result_q, $realtobits(0.0), "0 * 100 = 0");

        // ─── Test 4: FMADD (fused multiply-add: a*b + c) ───
        $display("\n-- FMADD (a*b + c) --");
        send_fpu_op(FMADD, $realtobits(2.0), $realtobits(3.0), $realtobits(4.0));
        check_f64(result_q, $realtobits(10.0), "2*3 + 4 = 10");

        send_fpu_op(FMADD, $realtobits(1.5), $realtobits(2.0), $realtobits(-1.0));
        check_f64(result_q, $realtobits(2.0), "1.5*2 + (-1) = 2");

        // Falcon-style: dr*wr + di*wi (real part of complex mul with conj)
        send_fpu_op(FMADD, $realtobits(3.0), $realtobits(0.5), $realtobits(2.0));
        check_f64(result_q, $realtobits(3.5), "3*0.5 + 2 = 3.5 (conj-style rep)");

        // Large values (~Falcon FFT range)
        send_fpu_op(FMADD, $realtobits(1234.5), $realtobits(0.7071), $realtobits(-567.8));
        check_f64(result_q, $realtobits(1234.5*0.7071 - 567.8),
                  "1234.5*0.7071 + (-567.8) (Falcon range)");

        // Small values (~Falcon leaf range)
        send_fpu_op(FMADD, $realtobits(0.123), $realtobits(-0.456), $realtobits(0.789));
        check_f64(result_q, $realtobits(0.123*(-0.456) + 0.789),
                  "0.123*(-0.456) + 0.789 (leaf range)");

        // ─── Test 5: FNMADD (negated fused multiply-add: -a*b + c) ───
        $display("\n-- FNMADD (-a*b + c) --");
        send_fpu_op(FNMADD, $realtobits(2.0), $realtobits(3.0), $realtobits(10.0));
        check_f64(result_q, $realtobits(4.0), "-2*3 + 10 = 4");

        send_fpu_op(FNMADD, $realtobits(1.0), $realtobits(1.0), $realtobits(0.0));
        check_f64(result_q, $realtobits(-1.0), "-1*1 + 0 = -1");

        // Falcon-style: -(dr*wi) + di*wr (imag part of complex mul with conj)
        send_fpu_op(FNMADD, $realtobits(3.0), $realtobits(0.5), $realtobits(4.0));
        check_f64(result_q, $realtobits(4.0 - 3.0*0.5), "-3*0.5 + 4 = 2.5 (conj-style imp)");

        // ─── Test 6: FMADD that cancels (a*b ≈ -c) ───
        $display("\n-- FMADD near-cancellation --");
        send_fpu_op(FMADD, $realtobits(1e5), $realtobits(1e-5), $realtobits(-1.0));
        check_f64(result_q, $realtobits(1e5 * 1e-5 - 1.0), "1e5*1e-5 + (-1) ≈ 0");

        // ─── Test 7: Half-value operations (SPLIT divide by 2) ───
        $display("\n-- Half-value (SPLIT scaling) --");
        // Simulating what SPLIT does: first computes a+b, then halves
        send_fpu_op(FADD, $realtobits(1000.0), $realtobits(2000.0), 0);
        check_f64(result_q, $realtobits(3000.0), "pre-half: 1000+2000=3000");

        send_fpu_op(FMUL, result_q, $realtobits(0.5), 0);
        check_f64(result_q, $realtobits(1500.0), "half: 3000*0.5=1500");

        // ─── Summary ───
        $display("\n=== Results: %0d pass, %0d fail ===", pass, fail);
        if (fail == 0)
            $display("FPU_ARITH PASSED");
        else
            $display("FPU_ARITH FAILED");
        $finish;
    end
endmodule
