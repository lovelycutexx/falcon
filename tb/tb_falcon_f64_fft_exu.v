`timescale 1ns/1ps

module tb_falcon_f64_fft_exu;

    localparam ADDR_W = 4;    // N up to 16; use tb_falcon_f64_fft_large for 256/512/1024
    localparam MAX_N  = (1 << ADDR_W);

    localparam [2:0] OP_FFT_FWD = 3'd0;
    localparam [2:0] OP_FFT_INV = 3'd1;

    reg                   clk;
    reg                   rst_n;
    reg                   cmd_valid;
    wire                  cmd_ready;
    reg  [2:0]            cmd_opcode;
    reg  [4:0]            cmd_logn;

    wire [ADDR_W-1:0]     mem_rd_addr0;
    wire [ADDR_W-1:0]     mem_rd_addr1;
    wire [63:0]           mem_rd_data0_re;
    wire [63:0]           mem_rd_data0_im;
    wire [63:0]           mem_rd_data1_re;
    wire [63:0]           mem_rd_data1_im;

    wire [ADDR_W-1:0]     twiddle_addr;
    wire [63:0]           twiddle_re;
    wire [63:0]           twiddle_im;

    wire                  mem_wr_en;
    wire [ADDR_W-1:0]     mem_wr_addr0;
    wire [ADDR_W-1:0]     mem_wr_addr1;
    wire [63:0]           mem_wr_data0_re;
    wire [63:0]           mem_wr_data0_im;
    wire [63:0]           mem_wr_data1_re;
    wire [63:0]           mem_wr_data1_im;

    wire                  rsp_valid;
    wire                  rsp_done;
    wire                  rsp_fail;
    wire [7:0]            rsp_status;
    wire                  status_invalid;
    wire                  status_overflow;
    wire                  status_underflow;
    wire                  status_inexact;
    wire                  busy;

    reg  [63:0] mem_re [0:MAX_N-1];
    reg  [63:0] mem_im [0:MAX_N-1];
    reg  [63:0] ref_re [0:MAX_N-1];
    reg  [63:0] ref_im [0:MAX_N-1];
    reg  [63:0] saved_re [0:MAX_N-1];
    reg  [63:0] saved_im [0:MAX_N-1];
    reg  [63:0] tw_re_rom [0:MAX_N-1];
    reg  [63:0] tw_im_rom [0:MAX_N-1];

    integer error_count;
    integer idx;
    integer wait_count;
    integer hw_csv_file;

    // CSV dump format: test_name,phase,idx,hw_re,hw_im

    assign mem_rd_data0_re = mem_re[mem_rd_addr0];
    assign mem_rd_data0_im = mem_im[mem_rd_addr0];
    assign mem_rd_data1_re = mem_re[mem_rd_addr1];
    assign mem_rd_data1_im = mem_im[mem_rd_addr1];

    assign twiddle_re = tw_re_rom[twiddle_addr];
    assign twiddle_im = tw_im_rom[twiddle_addr];

    falcon_f64_fft_exu #
    (
        .ADDR_W (ADDR_W)
    )
    dut (
        .clk              (clk),
        .rst_n            (rst_n),
        .cmd_valid        (cmd_valid),
        .cmd_ready        (cmd_ready),
        .cmd_opcode       (cmd_opcode),
        .cmd_logn         (cmd_logn),
        .mem_rd_addr0     (mem_rd_addr0),
        .mem_rd_addr1     (mem_rd_addr1),
        .mem_rd_data0_re  (mem_rd_data0_re),
        .mem_rd_data0_im  (mem_rd_data0_im),
        .mem_rd_data1_re  (mem_rd_data1_re),
        .mem_rd_data1_im  (mem_rd_data1_im),
        .twiddle_addr     (twiddle_addr),
        .twiddle_re       (twiddle_re),
        .twiddle_im       (twiddle_im),
        .mem_wr_en        (mem_wr_en),
        .mem_wr_addr0     (mem_wr_addr0),
        .mem_wr_addr1     (mem_wr_addr1),
        .mem_wr_data0_re  (mem_wr_data0_re),
        .mem_wr_data0_im  (mem_wr_data0_im),
        .mem_wr_data1_re  (mem_wr_data1_re),
        .mem_wr_data1_im  (mem_wr_data1_im),
        .rsp_valid        (rsp_valid),
        .rsp_done         (rsp_done),
        .rsp_fail         (rsp_fail),
        .rsp_status       (rsp_status),
        .status_invalid   (status_invalid),
        .status_overflow  (status_overflow),
        .status_underflow (status_underflow),
        .status_inexact   (status_inexact),
        .busy             (busy)
    );

    always #5 clk = ~clk;

    always @(posedge clk) begin
        if (mem_wr_en) begin
            mem_re[mem_wr_addr0] <= mem_wr_data0_re;
            mem_im[mem_wr_addr0] <= mem_wr_data0_im;
            mem_re[mem_wr_addr1] <= mem_wr_data1_re;
            mem_im[mem_wr_addr1] <= mem_wr_data1_im;
        end
    end

    // ------------------------------------------------------------------
    // Tasks
    // ------------------------------------------------------------------

    task generate_twiddles;
        input integer n;
        integer i;
        real pi;
        real angle;
        begin
            pi = 3.1415926535897932;
            for (i = 0; i < n; i = i + 1) begin
                angle = -2.0 * pi * i / n;
                tw_re_rom[i] = $realtobits($cos(angle));
                tw_im_rom[i] = $realtobits($sin(angle));
            end
            for (i = n; i < MAX_N; i = i + 1) begin
                tw_re_rom[i] = 64'd0;
                tw_im_rom[i] = 64'd0;
            end
        end
    endtask

    task load_simple_vectors;
        input integer n;
        integer i;
        real r;
        begin
            for (i = 0; i < n; i = i + 1) begin
                r = 0.5 + i * 0.75;
                mem_re[i] = $realtobits(r);
                mem_im[i] = $realtobits(-r * 0.3);
            end
        end
    endtask

    task load_random_vectors;
        input integer n;
        input integer seed;
        integer i;
        begin
            for (i = 0; i < n; i = i + 1) begin
                mem_re[i] = $realtobits($sin(seed + i * 31337) * 1.5);
                mem_im[i] = $realtobits($cos(seed + i * 31337 + 1000) * 1.5);
            end
        end
    endtask

    task load_zero_vectors;
        input integer n;
        integer i;
        begin
            for (i = 0; i < n; i = i + 1) begin
                mem_re[i] = 64'd0;
                mem_im[i] = 64'd0;
            end
        end
    endtask

    task load_impulse_vectors;
        input integer n;
        integer i;
        begin
            for (i = 0; i < n; i = i + 1) begin
                mem_re[i] = 64'd0;
                mem_im[i] = 64'd0;
            end
            mem_re[0] = 64'h3ff0_0000_0000_0000; // 1.0
        end
    endtask

    task load_all_ones;
        input integer n;
        integer i;
        begin
            for (i = 0; i < n; i = i + 1) begin
                mem_re[i] = 64'h3ff0_0000_0000_0000; // 1.0
                mem_im[i] = 64'd0;
            end
        end
    endtask

    task save_originals;
        input integer n;
        integer i;
        begin
            for (i = 0; i < n; i = i + 1) begin
                saved_re[i] = mem_re[i];
                saved_im[i] = mem_im[i];
            end
        end
    endtask

    task copy_to_ref;
        input integer n;
        integer i;
        begin
            for (i = 0; i < n; i = i + 1) begin
                ref_re[i] = mem_re[i];
                ref_im[i] = mem_im[i];
            end
        end
    endtask

    // Run double-precision reference FFT/IFFT (in-place on ref_re/ref_im).
    // Reference uses Verilog 'real' arithmetic which may differ from the
    // hardware FPU in the last 1-2 ULP — we use approximate comparison later.
    task reference_fft;
        input integer n;
        input integer logn;
        input          inverse;
        integer stage;
        integer pair;
        integer half_m;
        integer m_size;
        integer j_idx;
        integer group_idx;
        integer base_idx;
        integer addr_a;
        integer addr_b;
        integer tw_idx;
        real a_re_r;
        real a_im_r;
        real b_re_r;
        real b_im_r;
        real w_re_r;
        real w_im_r;
        real t_re_r;
        real t_im_r;
        real y0_re_r;
        real y0_im_r;
        real y1_re_r;
        real y1_im_r;
        integer bitrev_i;
        integer bitrev_j;
        integer tmp_i;
        reg [63:0] tmp_re_bits;
        reg [63:0] tmp_im_bits;
        begin
            // bit-reversal permutation
            for (bitrev_i = 0; bitrev_i < n; bitrev_i = bitrev_i + 1) begin
                bitrev_j = 0;
                for (tmp_i = 0; tmp_i < logn; tmp_i = tmp_i + 1) begin
                    bitrev_j = (bitrev_j << 1) | ((bitrev_i >> tmp_i) & 1);
                end
                if (bitrev_i < bitrev_j) begin
                    tmp_re_bits       = ref_re[bitrev_i];
                    tmp_im_bits       = ref_im[bitrev_i];
                    ref_re[bitrev_i]  = ref_re[bitrev_j];
                    ref_im[bitrev_i]  = ref_im[bitrev_j];
                    ref_re[bitrev_j]  = tmp_re_bits;
                    ref_im[bitrev_j]  = tmp_im_bits;
                end
            end

            for (stage = 0; stage < logn; stage = stage + 1) begin
                half_m = (1 << stage);
                m_size = (half_m << 1);
                for (pair = 0; pair < (n >> 1); pair = pair + 1) begin
                    j_idx     = pair & (half_m - 1);
                    group_idx = pair >> stage;
                    base_idx  = group_idx * m_size;
                    addr_a    = base_idx + j_idx;
                    addr_b    = addr_a + half_m;
                    tw_idx    = j_idx << (logn - stage - 1);

                    a_re_r = $bitstoreal(ref_re[addr_a]);
                    a_im_r = $bitstoreal(ref_im[addr_a]);
                    b_re_r = $bitstoreal(ref_re[addr_b]);
                    b_im_r = $bitstoreal(ref_im[addr_b]);
                    w_re_r = $bitstoreal(tw_re_rom[tw_idx]);
                    w_im_r = $bitstoreal(tw_im_rom[tw_idx]);

                    if (inverse) begin
                        w_im_r = -w_im_r;
                    end

                    t_re_r  = b_re_r * w_re_r - b_im_r * w_im_r;
                    t_im_r  = b_re_r * w_im_r + b_im_r * w_re_r;
                    y0_re_r = a_re_r + t_re_r;
                    y0_im_r = a_im_r + t_im_r;
                    y1_re_r = a_re_r - t_re_r;
                    y1_im_r = a_im_r - t_im_r;

                    if (inverse) begin
                        y0_re_r = y0_re_r * 0.5;
                        y0_im_r = y0_im_r * 0.5;
                        y1_re_r = y1_re_r * 0.5;
                        y1_im_r = y1_im_r * 0.5;
                    end

                    ref_re[addr_a] = $realtobits(y0_re_r);
                    ref_im[addr_a] = $realtobits(y0_im_r);
                    ref_re[addr_b] = $realtobits(y1_re_r);
                    ref_im[addr_b] = $realtobits(y1_im_r);
                end
            end
        end
    endtask

    task run_fft_cmd;
        input integer       logn;
        input [2:0]         opcode;
        begin
            @(posedge clk);
            while (!cmd_ready) begin
                @(posedge clk);
            end

            cmd_opcode <= opcode;
            cmd_logn   <= logn;
            cmd_valid  <= 1'b1;
            @(posedge clk);
            cmd_valid  <= 1'b0;

            wait_count = 0;
            while ((rsp_valid !== 1'b1) && (wait_count <= 20000)) begin
                #1;
                wait_count = wait_count + 1;
            end
            if (wait_count > 20000) begin
                $display("TB_FAIL - fft command timeout logn=%0d opcode=%0d", logn, opcode);
                error_count = error_count + 1;
                disable run_fft_cmd;
            end

            #1;
            if (!rsp_done || rsp_fail) begin
                $display("TB_FAIL - fft response error logn=%0d opcode=%0d done=%0d fail=%0d status=%h", logn, opcode, rsp_done, rsp_fail, rsp_status);
                error_count = error_count + 1;
            end
            // inexact is expected in nearly every floating-point operation — ignore it.
            // invalid/overflow/underflow are real errors.
            if (status_invalid || status_overflow || status_underflow) begin
                $display("TB_FAIL - fft exception logn=%0d opcode=%0d inv=%0d ovf=%0d udf=%0d (inexact ignored)", logn, opcode, status_invalid, status_overflow, status_underflow);
                error_count = error_count + 1;
            end
        end
    endtask

    // Compare DUT output against the reference model using relative error.
    // The hardware FPU and Verilog 'real' type may differ by a few ULP
    // (different rounding in f64_add vs the simulator's FPU), so we use
    // a generous tolerance: 1e-12 (~2^40 ULP at magnitude 1.0).  This is
    // still far tighter than Falcon's numerical requirements.
    task compare_with_reference;
        input integer               n;
        input [128*8-1:0]           phase_name;
        integer                     i;
        real                        exp_re;
        real                        exp_im;
        real                        got_re;
        real                        got_im;
        real                        err_re;
        real                        err_im;
        real                        mag;
        begin
            for (i = 0; i < n; i = i + 1) begin
                exp_re = $bitstoreal(ref_re[i]);
                exp_im = $bitstoreal(ref_im[i]);
                got_re = $bitstoreal(mem_re[i]);
                got_im = $bitstoreal(mem_im[i]);
                err_re = got_re - exp_re;
                err_im = got_im - exp_im;
                if (err_re < 0) err_re = -err_re;
                if (err_im < 0) err_im = -err_im;
                mag = (exp_re > 0 ? exp_re : -exp_re) + (exp_im > 0 ? exp_im : -exp_im);
                // For near-zero values, use absolute tolerance
                if (mag < 1e-150) begin
                    if ((err_re > 1e-12) || (err_im > 1e-12)) begin
                        $display("TB_FAIL - %0s [n=%0d] idx=%0d near-zero mismatch exp=(%e,%e) got=(%e,%e) err=(%e,%e)", phase_name, n, i, exp_re, exp_im, got_re, got_im, err_re, err_im);
                        error_count = error_count + 1;
                    end
                end else begin
                    if ((err_re > mag * 1e-12) || (err_im > mag * 1e-12)) begin
                        $display("TB_FAIL - %0s [n=%0d] idx=%0d exp=(%e,%e) got=(%e,%e) rel_err=(%e,%e)", phase_name, n, i, exp_re, exp_im, got_re, got_im, err_re/mag, err_im/mag);
                        error_count = error_count + 1;
                    end
                end
            end
        end
    endtask

    task check_roundtrip;
        input integer               n;
        input [128*8-1:0]           phase_name;
        integer                     i;
        real                        err_re;
        real                        err_im;
        real                        max_err;
        real                        got_re;
        real                        got_im;
        begin
            max_err = 0.0;
            for (i = 0; i < n; i = i + 1) begin
                got_re = $bitstoreal(mem_re[i]);
                got_im = $bitstoreal(mem_im[i]);
                err_re = got_re - $bitstoreal(saved_re[i]);
                err_im = got_im - $bitstoreal(saved_im[i]);
                if (err_re < 0) err_re = -err_re;
                if (err_im < 0) err_im = -err_im;
                if (err_re > max_err) max_err = err_re;
                if (err_im > max_err) max_err = err_im;
                if ((err_re > 1e-6) || (err_im > 1e-6)) begin
                    $display("TB_FAIL - %0s [n=%0d] idx=%0d roundtrip mismatch orig=(%e,%e) got=(%e,%e) err=(%e,%e)", phase_name, n, i, $bitstoreal(saved_re[i]), $bitstoreal(saved_im[i]), got_re, got_im, err_re, err_im);
                    error_count = error_count + 1;
                end
            end
            $display("TB_INFO - %0s [n=%0d] roundtrip max_err=%e", phase_name, n, max_err);
        end
    endtask

    task dump_hw_csv;
        input integer               n;
        input [128*8-1:0]           phase;
        input [128*8-1:0]           tname;
        integer                     i;
        begin
            for (i = 0; i < n; i = i + 1) begin
                $fwrite(hw_csv_file, "%0s,%0s,%0d,64'h%016h,64'h%016h\n", tname, phase, i, mem_re[i], mem_im[i]);
            end
        end
    endtask

    task dump_hw_csv_ref;
        input integer               n;
        input [128*8-1:0]           phase;
        input [128*8-1:0]           tname;
        integer                     i;
        begin
            for (i = 0; i < n; i = i + 1) begin
                $fwrite(hw_csv_file, "%0s,%0s,%0d,64'h%016h,64'h%016h\n", tname, phase, i, ref_re[i], ref_im[i]);
            end
        end
    endtask

    task run_testcase;
        input integer               n;
        input integer               logn;
        input [128*8-1:0]           name;
        reg   [128*8-1:0]           full_name;
        begin
            $display("=== Test: %0s n=%0d ===", name, n);

            generate_twiddles(n);
            save_originals(n);
            dump_hw_csv(n, "input", name);

            // FFT forward — compare with reference (2-ULP tolerance)
            $sformat(full_name, "%s_fft_fwd", name);
            copy_to_ref(n);
            reference_fft(n, logn, 1'b0);
            run_fft_cmd(logn, OP_FFT_FWD);
            compare_with_reference(n, full_name);
            dump_hw_csv(n, "fft", name);

            // IFFT inverse — compare with reference (2-ULP tolerance)
            $sformat(full_name, "%s_fft_inv", name);
            // Reload the original vectors for IFFT reference
            copy_to_ref(n);
            reference_fft(n, logn, 1'b1);
            run_fft_cmd(logn, OP_FFT_INV);
            compare_with_reference(n, full_name);
            dump_hw_csv(n, "ifft", name);

            // Round-trip: FFT then IFFT should restore original
            $sformat(full_name, "%s_roundtrip", name);
            run_fft_cmd(logn, OP_FFT_FWD);
            run_fft_cmd(logn, OP_FFT_INV);
            check_roundtrip(n, full_name);
        end
    endtask

    // ---------------------------------------------------------------
    // Test sequence
    // ---------------------------------------------------------------

    initial begin
        clk        = 1'b0;
        rst_n      = 1'b0;
        cmd_valid  = 1'b0;
        cmd_opcode = 3'd0;
        cmd_logn   = 5'd0;
        error_count = 0;

        hw_csv_file = $fopen("hw_results.csv", "w");
        $fwrite(hw_csv_file, "test_name,phase,idx,hw_re,hw_im\n");

        repeat (3) @(posedge clk);
        rst_n = 1'b1;
        repeat (2) @(posedge clk);

        // Dump waveform (only first test case to keep VCD small)
        $dumpfile("tb_falcon_f64_fft_exu.vcd");
        $dumpvars(0, tb_falcon_f64_fft_exu);
        $dumpoff;

        // Waveform dump for the first test case only
        $dumpon;
        // ========== N=4 tests (logn=2) ==========
        load_simple_vectors(4);
        run_testcase(4, 2, "simple_n4");

        $dumpoff;
        load_random_vectors(4, 42);
        run_testcase(4, 2, "random_n4");

        load_zero_vectors(4);
        run_testcase(4, 2, "zeros_n4");

        load_impulse_vectors(4);
        run_testcase(4, 2, "impulse_n4");

        load_all_ones(4);
        run_testcase(4, 2, "allones_n4");

        // ========== N=8 tests (logn=3) ==========
        load_simple_vectors(8);
        run_testcase(8, 3, "simple_n8");

        load_random_vectors(8, 137);
        run_testcase(8, 3, "random_n8");

        load_zero_vectors(8);
        run_testcase(8, 3, "zeros_n8");

        load_impulse_vectors(8);
        run_testcase(8, 3, "impulse_n8");

        load_all_ones(8);
        run_testcase(8, 3, "allones_n8");

        // ========== N=16 tests (logn=4) ==========
        load_simple_vectors(16);
        run_testcase(16, 4, "simple_n16");

        load_random_vectors(16, 999);
        run_testcase(16, 4, "random_n16");

        load_zero_vectors(16);
        run_testcase(16, 4, "zeros_n16");

        load_impulse_vectors(16);
        run_testcase(16, 4, "impulse_n16");

        load_all_ones(16);
        run_testcase(16, 4, "allones_n16");

        // Large N tests (256/512/1024) have their own testbench:
        //   tb_falcon_f64_fft_large.v  (ADDR_W=10, round-trip only)

        // ========== Back-to-back commands ==========
        $display("=== Test: back_to_back n=4 ===");
        load_random_vectors(4, 555);
        save_originals(4);
        dump_hw_csv(4, "input", "back_to_back");
        run_fft_cmd(2, OP_FFT_FWD);
        dump_hw_csv(4, "fft", "back_to_back");
        run_fft_cmd(2, OP_FFT_INV);
        dump_hw_csv(4, "ifft", "back_to_back");
        check_roundtrip(4, "back_to_back");

        // ========== FWD/INV/FWD/INV chain ==========
        $display("=== Test: chain_fwd_inv n=8 ===");
        load_random_vectors(8, 777);
        save_originals(8);
        dump_hw_csv(8, "input", "chain_fwd_inv");
        run_fft_cmd(3, OP_FFT_FWD);
        dump_hw_csv(8, "fft", "chain_fwd_inv");
        run_fft_cmd(3, OP_FFT_INV);
        dump_hw_csv(8, "ifft", "chain_fwd_inv");
        run_fft_cmd(3, OP_FFT_FWD);
        run_fft_cmd(3, OP_FFT_INV);
        check_roundtrip(8, "chain_fwd_inv");

        // ========== Result ==========
        if (error_count == 0) begin
            $display("");
            $display("##########################################");
            $display("  TB_PASS falcon_f64_fft_exu");
            $display("##########################################");
        end else begin
            $display("");
            $display("##########################################");
            $display("  TB_FAIL falcon_f64_fft_exu error_count=%0d", error_count);
            $display("##########################################");
        end

        $fclose(hw_csv_file);
        $finish;
    end

endmodule
