`timescale 1ns/1ps

// Large-N FFT/IFFT testbench — verifies N=256, 512, 1024 round-trip.
// ADDR_W=10 supports N up to 1024.  We keep the test minimal to avoid
// excessive simulation time: load vectors, run FFT→IFFT, check round-trip.

module tb_falcon_f64_fft_large;

    localparam ADDR_W = 10;
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

    reg  [63:0] mem_re  [0:MAX_N-1];
    reg  [63:0] mem_im  [0:MAX_N-1];
    reg  [63:0] saved_re [0:MAX_N-1];
    reg  [63:0] saved_im [0:MAX_N-1];
    reg  [63:0] tw_re_rom [0:MAX_N-1];
    reg  [63:0] tw_im_rom [0:MAX_N-1];

    integer error_count;
    integer idx;
    integer wait_count;
    integer hw_csv_file;

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

    // ----------------------------------------------------------------
    // Tasks
    // ----------------------------------------------------------------

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

    task load_zeros;
        input integer n;
        integer i;
        begin
            for (i = 0; i < n; i = i + 1) begin
                mem_re[i] = 64'd0;
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
            while ((rsp_valid !== 1'b1) && (wait_count <= 2000000)) begin
                #1;
                wait_count = wait_count + 1;
            end
            if (wait_count > 2000000) begin
                $display("TB_FAIL - timeout logn=%0d op=%0d", logn, opcode);
                error_count = error_count + 1;
                disable run_fft_cmd;
            end
            #1;
            if (!rsp_done || rsp_fail) begin
                $display("TB_FAIL - response error logn=%0d op=%0d done=%0d fail=%0d status=%h", logn, opcode, rsp_done, rsp_fail, rsp_status);
                error_count = error_count + 1;
            end
        end
    endtask

    task check_roundtrip;
        input integer               n;
        input [128*8-1:0]           name;
        integer                     i;
        real                        err;
        real                        max_err;
        real                        got_re;
        real                        got_im;
        begin
            max_err = 0.0;
            for (i = 0; i < n; i = i + 1) begin
                got_re = $bitstoreal(mem_re[i]);
                got_im = $bitstoreal(mem_im[i]);
                err = (got_re - $bitstoreal(saved_re[i])) * (got_re - $bitstoreal(saved_re[i]))
                    + (got_im - $bitstoreal(saved_im[i])) * (got_im - $bitstoreal(saved_im[i]));
                if (err > max_err) max_err = err;
            end
            max_err = $sqrt(max_err);
            $display("TB_INFO - %0s [n=%0d] roundtrip max_err=%e", name, n, max_err);
            if (max_err > 1e-6) begin
                $display("TB_FAIL - %0s [n=%0d] roundtrip exceeded tolerance: %e", name, n, max_err);
                error_count = error_count + 1;
            end
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

    task run_large_case;
        input integer               n;
        input integer               logn;
        input [128*8-1:0]           name;
        begin
            save_originals(n);
            dump_hw_csv(n, "input", name);
            run_fft_cmd(logn, OP_FFT_FWD);
            dump_hw_csv(n, "fft", name);
            run_fft_cmd(logn, OP_FFT_INV);
            dump_hw_csv(n, "ifft", name);
            check_roundtrip(n, name);
        end
    endtask

    // ----------------------------------------------------------------
    // Test sequence
    // ----------------------------------------------------------------

    initial begin
        clk        = 1'b0;
        rst_n      = 1'b0;
        cmd_valid  = 1'b0;
        cmd_opcode = 3'd0;
        cmd_logn   = 5'd0;
        error_count = 0;
        hw_csv_file = $fopen("hw_results_large.csv", "w");
        $fwrite(hw_csv_file, "test_name,phase,idx,hw_re,hw_im\n");

        repeat (3) @(posedge clk);
        rst_n = 1'b1;
        repeat (2) @(posedge clk);

        // ----- N=256 tests (logn=8) -----
        $display("=== large N=256 simple ===");
        generate_twiddles(256);
        load_simple_vectors(256);
        run_large_case(256, 8, "simple_n256");

        $display("=== large N=256 random ===");
        generate_twiddles(256);
        load_random_vectors(256, 42);
        run_large_case(256, 8, "random_n256");

        $display("=== large N=256 zeros ===");
        generate_twiddles(256);
        load_zeros(256);
        run_large_case(256, 8, "zeros_n256");

        // ----- N=512 tests (logn=9) -----
        $display("=== large N=512 random ===");
        generate_twiddles(512);
        load_random_vectors(512, 137);
        run_large_case(512, 9, "random_n512");

        $display("=== large N=512 zeros ===");
        generate_twiddles(512);
        load_zeros(512);
        run_large_case(512, 9, "zeros_n512");

        // ----- N=1024 tests (logn=10) -----
        $display("=== large N=1024 random ===");
        generate_twiddles(1024);
        load_random_vectors(1024, 999);
        run_large_case(1024, 10, "random_n1024");

        $display("=== large N=1024 zeros ===");
        generate_twiddles(1024);
        load_zeros(1024);
        run_large_case(1024, 10, "zeros_n1024");

        // ----- Result -----
        if (error_count == 0) begin
            $display("");
            $display("##########################################");
            $display("  TB_PASS falcon_f64_fft_large");
            $display("##########################################");
        end else begin
            $display("");
            $display("##########################################");
            $display("  TB_FAIL falcon_f64_fft_large error_count=%0d", error_count);
            $display("##########################################");
        end
        $fclose(hw_csv_file);
        $finish;
    end

endmodule
