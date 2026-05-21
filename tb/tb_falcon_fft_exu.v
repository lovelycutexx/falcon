`timescale 1ns/1ps

module tb_falcon_fft_exu;

    localparam ADDR_W = 10;
    localparam N      = 16;
    localparam LOGN   = 4;

    reg         clk;
    reg         rst_n;

    // FFT command
    reg         cmd_valid;
    wire        cmd_ready;
    reg  [2:0]  cmd_opcode;
    reg  [4:0]  cmd_logn;

    // Memory signals (from FFT EXU)
    wire [ADDR_W-1:0] fft_rd_addr0, fft_rd_addr1;
    wire [63:0]       fft_rd_data0_re, fft_rd_data0_im;
    wire [63:0]       fft_rd_data1_re, fft_rd_data1_im;
    wire [ADDR_W-1:0] fft_twiddle_addr;
    wire [63:0]       fft_twiddle_re, fft_twiddle_im;
    wire              fft_wr_en;
    wire [ADDR_W-1:0] fft_wr_addr0, fft_wr_addr1;
    wire [63:0]       fft_wr_data0_re, fft_wr_data0_im;
    wire [63:0]       fft_wr_data1_re, fft_wr_data1_im;
    wire              fft_rsp_valid, fft_rsp_done, fft_rsp_fail;
    wire [7:0]        fft_rsp_status;
    wire              fft_busy;

    // ─── DUT: FFT EXU ───
    falcon_f64_fft_exu #(.ADDR_W(ADDR_W)) u_fft (
        .clk(clk), .rst_n(rst_n),
        .cmd_valid(cmd_valid), .cmd_ready(cmd_ready),
        .cmd_opcode(cmd_opcode), .cmd_logn(cmd_logn),
        .mem_rd_addr0(fft_rd_addr0), .mem_rd_addr1(fft_rd_addr1),
        .mem_rd_data0_re(fft_rd_data0_re), .mem_rd_data0_im(fft_rd_data0_im),
        .mem_rd_data1_re(fft_rd_data1_re), .mem_rd_data1_im(fft_rd_data1_im),
        .twiddle_addr(fft_twiddle_addr), .twiddle_re(fft_twiddle_re),
        .twiddle_im(fft_twiddle_im),
        .mem_wr_en(fft_wr_en), .mem_wr_addr0(fft_wr_addr0),
        .mem_wr_addr1(fft_wr_addr1),
        .mem_wr_data0_re(fft_wr_data0_re), .mem_wr_data0_im(fft_wr_data0_im),
        .mem_wr_data1_re(fft_wr_data1_re), .mem_wr_data1_im(fft_wr_data1_im),
        .rsp_valid(fft_rsp_valid), .rsp_done(fft_rsp_done), .rsp_fail(fft_rsp_fail),
        .rsp_status(fft_rsp_status), .busy(fft_busy)
    );

    // ─── Twiddle ROM ───
    falconsign_twiddle_rom #(.ADDR_W(8), .DEPTH(256)) u_twiddle (
        .clk(clk),
        .addr(fft_twiddle_addr[7:0]),
        .twiddle_re(fft_twiddle_re),
        .twiddle_im(fft_twiddle_im)
    );

    // ─── Behavioral Memory ───
    localparam MEM_DEPTH = 1024;
    reg [63:0] mem_re [0:MEM_DEPTH-1];
    reg [63:0] mem_im [0:MEM_DEPTH-1];

    // Read: combinational (0-cycle) to match FFT EXU timing expectation
    assign fft_rd_data0_re = mem_re[fft_rd_addr0];
    assign fft_rd_data0_im = mem_im[fft_rd_addr0];
    assign fft_rd_data1_re = mem_re[fft_rd_addr1];
    assign fft_rd_data1_im = mem_im[fft_rd_addr1];

    // Write
    always @(posedge clk) begin
        if (fft_wr_en) begin
            mem_re[fft_wr_addr0] <= fft_wr_data0_re;
            mem_im[fft_wr_addr0] <= fft_wr_data0_im;
            mem_re[fft_wr_addr1] <= fft_wr_data1_re;
            mem_im[fft_wr_addr1] <= fft_wr_data1_im;
        end
    end

    // ─── Clock & Reset ───
    initial begin
        clk   = 1'b0;
        rst_n = 1'b0;
        #30 rst_n = 1'b1;
    end
    always #5 clk = ~clk;

    // ─── Load test vector into memory ───
    // Test 1: Unit impulse → FFT output should be all 1.0 + 0.0j
    integer i;
    initial begin
        // Initialize all memory to 0
        for (i = 0; i < MEM_DEPTH; i = i + 1) begin
            mem_re[i] = 64'd0;
            mem_im[i] = 64'd0;
        end
        // Impulse at index 0: x[0] = 1.0 + 0.0j
        mem_re[0] = 64'h3FF0000000000000; // 1.0
        mem_im[0] = 64'h0000000000000000; // 0.0
    end

    // ─── Test sequence (multi-test) ───
    reg [31:0]  cycle;
    reg [2:0]   tb_test_phase;
    reg [7:0]   tb_test_num;
    reg [31:0]  tb_test_start_cycle;
    reg [63:0]  mem_snapshot_re [0:1023];
    reg [63:0]  mem_snapshot_im [0:1023];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cycle              <= 32'd0;
            cmd_valid          <= 1'b0;
            cmd_opcode         <= 3'd0;
            cmd_logn           <= 5'd0;
            tb_test_phase      <= 3'd0;
            tb_test_num        <= 8'd0;
            tb_test_start_cycle <= 32'd0;
        end else begin
            cycle     <= cycle + 32'd1;
            cmd_valid <= 1'b0;

            case (tb_test_phase)
                3'd0: begin
                    tb_test_num   <= 8'd1;
                    tb_test_phase <= 3'd1;
                end

                3'd1: begin
                    if (cycle - tb_test_start_cycle > 32'd10) begin
                        $display("=== TEST %0d: FFT FWD, N=%0d ===", tb_test_num,
                                 (tb_test_num <= 2) ? 16 : 512);
                        cmd_valid  <= 1'b1;
                        cmd_opcode <= 3'd0;
                        cmd_logn   <= (tb_test_num <= 2) ? 5'd4 : 5'd9;
                        tb_test_start_cycle <= cycle;
                        tb_test_phase <= 3'd2;
                    end
                end

                3'd2: begin
                    if (fft_rsp_valid && fft_rsp_done) begin
                        $display("[%0t] FFT done, status=%h", $time, fft_rsp_status);
                        snapshot_memory;
                        tb_test_phase <= 3'd3;
                    end
                end

                3'd3: begin
                    verify_current_test;
                    tb_test_start_cycle <= cycle;
                    if (tb_test_num == 1) begin
                        $display("=== TEST %0d: IFFT, N=%0d ===", tb_test_num + 1, 16);
                        tb_test_num <= 8'd2;
                        cmd_valid  <= 1'b1;
                        cmd_opcode <= 3'd1;
                        cmd_logn   <= 5'd4;
                        tb_test_phase <= 3'd2;
                    end else if (tb_test_num == 2) begin
                        $display("=== TEST %0d: FFT FWD, N=%0d ===", tb_test_num + 1, 512);
                        load_impulse(512);
                        tb_test_num <= 8'd3;
                        cmd_valid  <= 1'b1;
                        cmd_opcode <= 3'd0;
                        cmd_logn   <= 5'd9;
                        tb_test_phase <= 3'd2;
                    end else if (tb_test_num == 3) begin
                        $display("=== TEST %0d: IFFT, N=%0d ===", tb_test_num + 1, 512);
                        tb_test_num <= 8'd4;
                        cmd_valid  <= 1'b1;
                        cmd_opcode <= 3'd1;
                        cmd_logn   <= 5'd9;
                        tb_test_phase <= 3'd2;
                    end else begin
                        $display("=== ALL 4 TESTS COMPLETE ===");
                        $finish;
                    end
                end

                default: tb_test_phase <= 3'd0;
            endcase
        end
    end

    // ─── Helper tasks ───
    task load_impulse;
        input integer nn;
        integer j;
        begin
            for (j = 0; j < nn; j = j + 1) begin
                mem_re[j] = 64'd0;
                mem_im[j] = 64'd0;
            end
            mem_re[0] = 64'h3FF0000000000000; // 1.0
            mem_im[0] = 64'h0000000000000000; // 0.0
        end
    endtask

    task snapshot_memory;
        integer j;
        begin
            for (j = 0; j < 1024; j = j + 1) begin
                mem_snapshot_re[j] = mem_re[j];
                mem_snapshot_im[j] = mem_im[j];
            end
        end
    endtask

    task verify_current_test;
        real      val_re, val_im, err, expected_re;
        integer   j, Nn, fail_cnt;
        begin
            fail_cnt = 0;
            Nn = (tb_test_num <= 2) ? 16 : 512;
            $display("--- Verification for test %0d (N=%0d) ---", tb_test_num, Nn);

            if (tb_test_num == 1 || tb_test_num == 3) begin
                // FWD FFT of impulse → all 1.0 + 0.0j
                expected_re = 1.0;
                for (j = 0; j < Nn; j = j + 1) begin
                    val_re = $bitstoreal(mem_re[j]);
                    val_im = $bitstoreal(mem_im[j]);
                    err = (val_re - expected_re)*(val_re - expected_re) + val_im*val_im;
                    if (err > 0.001) begin
                        if (fail_cnt < 8)
                            $display("  [%4d] RE=%12.9f IM=%12.9f  MISMATCH", j, val_re, val_im);
                        fail_cnt = fail_cnt + 1;
                    end
                end
            end else begin
                // IFFT of all-ones → impulse at 0
                for (j = 0; j < Nn; j = j + 1) begin
                    val_re = $bitstoreal(mem_re[j]);
                    val_im = $bitstoreal(mem_im[j]);
                    if (j == 0) begin
                        err = (val_re - 1.0)*(val_re - 1.0) + val_im*val_im;
                    end else begin
                        err = val_re*val_re + val_im*val_im;
                    end
                    if (err > 0.005) begin
                        if (fail_cnt < 8)
                            $display("  [%4d] RE=%12.9f IM=%12.9f  MISMATCH", j, val_re, val_im);
                        fail_cnt = fail_cnt + 1;
                    end
                end
            end

            if (fail_cnt == 0)
                $display("  ALL %0d OUTPUTS MATCH", Nn);
            else
                $display("  FAILED: %0d mismatches", fail_cnt);
        end
    endtask

    // ─── Watchdog ───
    initial begin
        #50000000;
        $display("[%0t] WATCHDOG TIMEOUT", $time);
        $finish;
    end

    // ─── Dump ───
    initial begin
        $dumpfile("tb_falcon_fft_exu.vcd");
        $dumpvars(0, tb_falcon_fft_exu);
    end

endmodule
