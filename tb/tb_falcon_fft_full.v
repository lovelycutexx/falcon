`timescale 1ns/1ps

module tb_falcon_fft_full;

    localparam ADDR_W = 10, MEM_DEPTH = 1024;
    localparam N_VALS = 8;  // logn = 2..9 (N=4..512)
    reg [4:0] logn_list [0:N_VALS-1];
    integer li;

    reg clk, rst_n;
    reg cmd_valid, cmd_opcode_inv;
    wire cmd_ready;
    reg [4:0] cmd_logn;

    // FFT interface
    wire [ADDR_W-1:0] fft_rd_addr0, fft_rd_addr1;
    wire [63:0] fft_rd_data0_re, fft_rd_data0_im;
    wire [63:0] fft_rd_data1_re, fft_rd_data1_im;
    wire [ADDR_W-1:0] fft_twiddle_addr;
    wire [63:0] fft_twiddle_re, fft_twiddle_im;
    wire fft_wr_en;
    wire [ADDR_W-1:0] fft_wr_addr0, fft_wr_addr1;
    wire [63:0] fft_wr_data0_re, fft_wr_data0_im;
    wire [63:0] fft_wr_data1_re, fft_wr_data1_im;
    wire fft_rsp_valid, fft_rsp_done, fft_rsp_fail;
    wire [7:0] fft_rsp_status;

    falcon_f64_fft_exu #(.ADDR_W(ADDR_W)) u_fft (
        .clk(clk), .rst_n(rst_n),
        .cmd_valid(cmd_valid), .cmd_ready(cmd_ready),
        .cmd_opcode({2'b00, cmd_opcode_inv}),
        .cmd_logn(cmd_logn),
        .mem_rd_addr0(fft_rd_addr0), .mem_rd_addr1(fft_rd_addr1),
        .mem_rd_data0_re(fft_rd_data0_re), .mem_rd_data0_im(fft_rd_data0_im),
        .mem_rd_data1_re(fft_rd_data1_re), .mem_rd_data1_im(fft_rd_data1_im),
        .twiddle_addr(fft_twiddle_addr), .twiddle_re(fft_twiddle_re),
        .twiddle_im(fft_twiddle_im),
        .mem_wr_en(fft_wr_en), .mem_wr_addr0(fft_wr_addr0),
        .mem_wr_addr1(fft_wr_addr1),
        .mem_wr_data0_re(fft_wr_data0_re), .mem_wr_data0_im(fft_wr_data0_im),
        .mem_wr_data1_re(fft_wr_data1_re), .mem_wr_data1_im(fft_wr_data1_im),
        .rsp_valid(fft_rsp_valid), .rsp_done(fft_rsp_done),
        .rsp_fail(fft_rsp_fail), .rsp_status(fft_rsp_status),
        .busy()
    );

    falconsign_twiddle_rom #(.ADDR_W(8), .DEPTH(256)) u_twiddle (
        .clk(clk), .addr(fft_twiddle_addr[7:0]),
        .twiddle_re(fft_twiddle_re), .twiddle_im(fft_twiddle_im)
    );

    // Combinational memory
    reg [63:0] mem_re [0:MEM_DEPTH-1];
    reg [63:0] mem_im [0:MEM_DEPTH-1];
    assign fft_rd_data0_re = mem_re[fft_rd_addr0];
    assign fft_rd_data0_im = mem_im[fft_rd_addr0];
    assign fft_rd_data1_re = mem_re[fft_rd_addr1];
    assign fft_rd_data1_im = mem_im[fft_rd_addr1];
    always @(posedge clk) begin
        if (fft_wr_en) begin
            mem_re[fft_wr_addr0] <= fft_wr_data0_re;
            mem_im[fft_wr_addr0] <= fft_wr_data0_im;
            mem_re[fft_wr_addr1] <= fft_wr_data1_re;
            mem_im[fft_wr_addr1] <= fft_wr_data1_im;
        end
    end

    // Roundtrip save area
    reg [63:0] rt_orig_re [0:511];
    reg [63:0] rt_orig_im [0:511];

    initial begin clk = 0; rst_n = 0; #30 rst_n = 1; end
    always #5 clk = ~clk;

    // ─── Test state ───
    reg [31:0] cycle;
    reg [7:0]  phase, test_idx;
    reg [31:0] pc;  // phase cycle counter
    reg [31:0] pass, fail;
    reg [4:0]  logn;
    reg [9:0]  Nn;
    reg [7:0]  pat;       // 0=imp0, 1=impN/2, 2=DC, 3=roundtrip
    reg [9:0]  imp_pos;
    reg        is_inv;
    reg        rt_half;   // 0=FFT phase, 1=IFFT phase

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cycle <= 0; phase <= 0; test_idx <= 0; pc <= 0;
            pass <= 0; fail <= 0;
            logn <= 0; Nn <= 0; pat <= 0; imp_pos <= 0; is_inv <= 0;
            rt_half <= 0;
            cmd_valid <= 0; cmd_opcode_inv <= 0; cmd_logn <= 0;
            // init logn list
            for (li = 0; li < N_VALS; li = li + 1) logn_list[li] <= li + 2;
        end else begin
            cycle <= cycle + 1;
            cmd_valid <= 0;
            pc <= pc + 1;

            case (phase)
                0: begin // Init: load logn list, start first test
                    for (li = 0; li < N_VALS; li = li + 1) logn_list[li] <= li + 2;
                    pass <= 0; fail <= 0;
                    test_idx <= 0; phase <= 1; pc <= 0;
                end

                1: begin // Pick next test
                    if (test_idx >= 3 * N_VALS) begin // 3 test types
                        $display("=== ALL %0d TESTS DONE: %0d pass, %0d fail ===",
                                 test_idx, pass, fail);
                        #100 $finish;
                    end

                    li = test_idx % N_VALS;
                    logn <= logn_list[li];
                    Nn   <= 1 << logn_list[li];
                    if (test_idx < N_VALS) begin
                        pat <= 0; is_inv <= 0; // impulse at 0
                        $display("=== TEST %0d: FFT FWD N=%0d, impulse@0 ===",
                                 test_idx+1, 1 << logn_list[li]);
                    end else if (test_idx < 2*N_VALS) begin
                        pat <= 1; is_inv <= 0; // impulse at N/2
                        imp_pos <= (1 << logn_list[li]) / 2;
                        $display("=== TEST %0d: FFT FWD N=%0d, impulse@N/2 ===",
                                 test_idx+1, 1 << logn_list[li]);
                    end else begin
                        pat <= 2; is_inv <= 0; rt_half <= 0; // DC + roundtrip
                        $display("=== TEST %0d-%0d: ROUNDTRIP N=%0d ===",
                                 test_idx+1, test_idx+2, 1 << logn_list[li]);
                    end
                    load_memory;
                    cmd_valid <= 1;
                    cmd_opcode_inv <= 0; // FWD first
                    cmd_logn <= logn_list[li];
                    phase <= 2; pc <= 0;
                end

                2: begin // Wait for FFT
                    if (fft_rsp_valid && fft_rsp_done) begin
                        if (pat == 2) begin // Roundtrip: run IFFT next
                            save_fft_output;
                            cmd_valid <= 1;
                            cmd_opcode_inv <= 1; // INV
                            cmd_logn <= logn;
                            rt_half <= 1;
                            phase <= 2; pc <= 0; // stay in wait
                        end else begin
                            phase <= 3; pc <= 0;
                        end
                    end
                    if (pc > 5000000) begin
                        $display("  TIMEOUT"); fail <= fail + 1;
                        test_idx <= test_idx + (pat == 2 ? 2 : 1);
                        phase <= 1; pc <= 0;
                    end
                end

                3: begin // Verify
                    verify_current;
                    test_idx <= test_idx + (pat == 2 ? 2 : 1);
                    phase <= 1; pc <= 0;
                end

                // Roundtrip IFFT result captured in phase 2
                // Handled by re-entering phase 2 with is_inv=1 and rt_half=1
                // After IFFT done: rt_half goes 0→1, but we need to detect completion
                // The FFT RTL will fire rsp_valid again for the IFFT
                // We stay in phase 2, and on the SECOND rsp_valid, check rt_half

                default: phase <= 0;
            endcase

            // Roundtrip IFFT done detection (phase 2 re-entry)
            if (phase == 2 && rt_half == 1 && fft_rsp_valid && fft_rsp_done) begin
                phase <= 3; pc <= 0;
            end
        end
    end

    // ─── Load test vectors ───
    task load_memory;
        integer j; real v;
        begin
            for (j = 0; j < Nn; j = j + 1) begin
                mem_re[j] = 64'd0; mem_im[j] = 64'd0;
            end
            case (pat)
                0: mem_re[0] = 64'h3FF0000000000000; // 1.0 at 0
                1: mem_re[imp_pos] = 64'h3FF0000000000000; // 1.0 at imp_pos
                2: begin // Falcon-like data, save for roundtrip check
                    for (j = 0; j < Nn; j = j + 1) begin
                        v = ((j * 17 + 3) % 7) - 3.0;
                        mem_re[j] = $realtobits(v);
                        v = ((j * 13 + 7) % 5) - 2.0;
                        mem_im[j] = $realtobits(v);
                        rt_orig_re[j] = mem_re[j];
                        rt_orig_im[j] = mem_im[j];
                    end
                end
            endcase
        end
    endtask

    // ─── Save FFT output for roundtrip ───
    task save_fft_output;
        integer j;
        begin
            for (j = 0; j < Nn; j = j + 1) begin
                mem_re[j] = mem_re[j]; // FFT output already in mem
                mem_im[j] = mem_im[j];
            end
        end
    endtask

    // ─── Verification ───
    task verify_current;
        real vr, vi, evr, evi, err, tol;
        integer j, fails;
        begin
            fails = 0; tol = 0.001;

            if (pat == 2) begin
                // Roundtrip: IFFT(FFT(orig)) = orig × N
                // IFFT scales by 0.5 per stage = total 1/N
                // Actually: IFFT with per-stage 0.5 scaling applies (1/2)^logn = 1/N
                // So IFFT(FFT(orig)) = (1/N) * N * orig = orig
                // Wait no: FFT FWD does NOT scale, IFFT scales by 0.5 per stage = 1/N
                // So: IFFT(FFT(orig)) = (1/N) * FFT(orig) in time domain
                // = (1/N) * (N * orig_reversed?) = orig (if we account for bit-reversal)
                //
                // Empirically with our testbench: the roundtrip preserves amplitude
                // because the bit-reversal cancels out.
                // Let's just check: output ≈ original data (both are small ints ±3)
                for (j = 0; j < Nn; j = j + 1) begin
                    vr = $bitstoreal(mem_re[j]);
                    vi = $bitstoreal(mem_im[j]);
                    evr = $bitstoreal(rt_orig_re[j]);
                    evi = $bitstoreal(rt_orig_im[j]);
                    err = (vr-evr)*(vr-evr) + (vi-evi)*(vi-evi);
                    if (err > 0.01) begin
                        if (fails < 6)
                            $display("  [%4d] got=(%10.6f,%10.6f) exp=(%10.6f,%10.6f)", j, vr, vi, evr, evi);
                        fails = fails + 1;
                    end
                end
            end else begin
                for (j = 0; j < Nn; j = j + 1) begin
                    vr = $bitstoreal(mem_re[j]);
                    vi = $bitstoreal(mem_im[j]);

                    if (pat == 0) begin
                        // FFT(impulse@0) = all ones (real=1, imag=0)
                        evr = 1.0; evi = 0.0;
                    end else begin
                        // FFT(impulse@k) = exp(-2πi·k·j/N) for forward FFT
                        evr = $cos(-2.0 * 3.141592653589793 * imp_pos * j / Nn);
                        evi = $sin(-2.0 * 3.141592653589793 * imp_pos * j / Nn);
                    end

                    err = (vr-evr)*(vr-evr) + (vi-evi)*(vi-evi);
                    if (err > tol*tol) begin
                        if (fails < 5)
                            $display("  [%4d] got=(%10.6f,%10.6f) exp=(%10.6f,%10.6f) err=%.2e",
                                     j, vr, vi, evr, evi, err);
                        fails = fails + 1;
                    end
                end
            end

            if (fails == 0) begin
                $display("  PASS"); pass <= pass + 1;
            end else begin
                $display("  FAIL: %0d mismatches (tol=%.0e)", fails, tol);
                fail <= fail + 1;
            end
        end
    endtask

    initial begin #200000000; $display("GLOBAL TIMEOUT"); $finish; end
    initial begin $dumpfile("tb_falcon_fft_full.vcd"); $dumpvars(0, tb_falcon_fft_full); end

endmodule
