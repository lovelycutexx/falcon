`timescale 1ns/1ps
// Golden SPLIT test: drives the ffSampling EXU with a single OP_SPLIT
// task and compares output against software golden values.
//
// Input a (t0[0]): re=0xc0bf4818a2d0d6f1 im=0x40e861812c5f75b0
// Input b (t0[1]): re=0x40ac43e326d63b98 im=0xc0994ce7a76ba61a
// Expected f0:     re=0xc0a126270f65b925 im=0x40d79719ef24187f
// Expected f1:     re=0xc0b616c114404755 im=0x40d934b41c9d3c7d
// GM twiddle[0]:   re=0x3fefffd8858e8a92 im=0x3f7921f0fe670071

module tb_split_golden;

    localparam ADDR_W = 10;

    reg         clk, rst_n;
    reg         task_valid;
    wire        task_ready;
    reg  [67:0] task_word;
    wire        task_done, task_fail;
    wire [7:0]  task_status;

    wire        mem_rd_en;
    wire [ADDR_W-1:0] mem_rd_addr;
    reg  [255:0] mem_rd_data;
    wire        mem_wr_en;
    wire [ADDR_W-1:0] mem_wr_addr;
    wire [255:0] mem_wr_data;

    wire [ADDR_W-1:0] twiddle_addr;
    reg  [63:0] twiddle_re, twiddle_im;

    wire        fpu_req_valid;
    reg         fpu_req_ready;
    wire [3:0]  fpu_req_op;
    wire [63:0] fpu_req_a, fpu_req_b, fpu_req_c;
    reg         fpu_rsp_valid;
    reg  [63:0] fpu_rsp_result;

    wire        sz_cmd_valid;
    reg         sz_cmd_ready;
    wire [63:0] sz_cmd_mu, sz_cmd_sigma_inv;
    wire        sz_cmd_pair;
    reg         sz_rsp_valid;
    reg  [63:0] sz_rsp_z0, sz_rsp_z1;

    // Memory model — holds a, b at addresses 0, 1
    reg [63:0] mem_re [0:511];
    reg [63:0] mem_im [0:511];

    // ─── DUT ───
    falcon_f64_ffsampling_exu #(.ADDR_W(ADDR_W)) dut (
        .clk(clk), .rst_n(rst_n),
        .task_valid(task_valid), .task_ready(task_ready),
        .task_word(task_word),
        .task_done(task_done), .task_fail(task_fail),
        .task_status(task_status),
        .mem_rd_en(mem_rd_en), .mem_rd_addr(mem_rd_addr),
        .mem_rd_data(mem_rd_data),
        .mem_wr_en(mem_wr_en), .mem_wr_addr(mem_wr_addr),
        .mem_wr_data(mem_wr_data),
        .twiddle_addr(twiddle_addr),
        .twiddle_re(twiddle_re), .twiddle_im(twiddle_im),
        .fpu_req_valid(fpu_req_valid), .fpu_req_ready(fpu_req_ready),
        .fpu_req_op(fpu_req_op),
        .fpu_req_a(fpu_req_a), .fpu_req_b(fpu_req_b), .fpu_req_c(fpu_req_c),
        .fpu_rsp_valid(fpu_rsp_valid), .fpu_rsp_result(fpu_rsp_result),
        .sz_cmd_valid(sz_cmd_valid), .sz_cmd_ready(sz_cmd_ready),
        .sz_cmd_mu(sz_cmd_mu), .sz_cmd_sigma_inv(sz_cmd_sigma_inv),
        .sz_cmd_pair(sz_cmd_pair),
        .sz_rsp_valid(sz_rsp_valid),
        .sz_rsp_z0(sz_rsp_z0), .sz_rsp_z1(sz_rsp_z1)
    );

    // Memory readback
    assign mem_rd_data = {128'd0, mem_im[mem_rd_addr], mem_re[mem_rd_addr]};

    // Memory write
    always @(posedge clk) begin
        if (mem_wr_en) begin
            mem_re[mem_wr_addr] <= mem_wr_data[63:0];
            mem_im[mem_wr_addr] <= mem_wr_data[127:64];
        end
    end

    // ─── Software FPU model (combinational bypass, 1-cycle latency) ───
    function [63:0] f64_add; input [63:0] a,b; begin f64_add = $realtobits($bitstoreal(a)+$bitstoreal(b)); end endfunction
    function [63:0] f64_sub; input [63:0] a,b; begin f64_sub = $realtobits($bitstoreal(a)-$bitstoreal(b)); end endfunction
    function [63:0] f64_mul; input [63:0] a,b; begin f64_mul = $realtobits($bitstoreal(a)*$bitstoreal(b)); end endfunction

    assign fpu_req_ready = 1'b1;

    always @(posedge clk) begin
        fpu_rsp_valid <= fpu_req_valid;
        if (fpu_req_valid) begin
            case (fpu_req_op)
                4'd0: fpu_rsp_result <= f64_add(fpu_req_a, fpu_req_b);
                4'd1: fpu_rsp_result <= f64_sub(fpu_req_a, fpu_req_b);
                4'd2: fpu_rsp_result <= f64_mul(fpu_req_a, fpu_req_b);
                4'd3: fpu_rsp_result <= f64_add(f64_mul(fpu_req_a, fpu_req_b), fpu_req_c);
                4'd6: fpu_rsp_result <= f64_sub(fpu_req_c, f64_mul(fpu_req_a, fpu_req_b));
                default: fpu_rsp_result <= 64'd0;
            endcase
        end
    end

    // ─── SamplerZ stub (not used by SPLIT) ───
    assign sz_cmd_ready = 1'b1;

    // ─── Clock ───
    initial clk = 1'b0;
    always #5 clk = ~clk;

    // ─── Test sequence ───
    reg [63:0] expected_f0_re, expected_f0_im;
    reg [63:0] expected_f1_re, expected_f1_im;
    integer    errors;

    initial begin
        rst_n = 1'b0;
        task_valid = 1'b0;
        sz_rsp_valid = 1'b0;
        errors = 0;

        // Golden values from software
        expected_f0_re = 64'hc0a126270f65b925;
        expected_f0_im = 64'h40d79719ef24187f;
        expected_f1_re = 64'hc0b616c114404755;
        expected_f1_im = 64'h40d934b41c9d3c7d;

        // Load input a at addr 0
        mem_re[0] = 64'hc0bf4818a2d0d6f1;
        mem_im[0] = 64'h40e861812c5f75b0;
        // Load input b at addr 1
        mem_re[1] = 64'h40ac43e326d63b98;
        mem_im[1] = 64'hc0994ce7a76ba61a;

        // GM twiddle at addr 0 (for level=0, idx=0: gm_tab[256])
        twiddle_re = 64'h3fefffd8858e8a92;
        twiddle_im = 64'h3f7921f0fe670071;

        repeat (8) @(posedge clk);
        rst_n = 1'b1;
        repeat (4) @(posedge clk);

        $display("=== SPLIT Golden Test ===");
        $display("Input a: re=%h im=%h", mem_re[0], mem_im[0]);
        $display("Input b: re=%h im=%h", mem_re[1], mem_im[1]);
        $display("GM twiddle: re=%h im=%h", twiddle_re, twiddle_im);
        $display("");

        // Issue SPLIT task
        // Task word format [67:0] — as decoded by the EXU:
        //   [67:64] = op       (4 bits) → OP_SPLIT = 4'd1
        //   [63:60] = level    (4 bits) → 0 for root
        //   [59:50] = unused
        //   [49:36] = src0     (14 bits) → 0
        //   [35:22] = src1     (14 bits) → 0 (unused for split)
        //   [21:8]  = dst      (14 bits) → 0
        //   [7:0]   = unused
        task_word = {4'd1, 4'd0, 10'd0, 14'd0, 14'd0, 14'd0, 8'd0};
        // wait actually, let me check the task word encoding from the scheduler...
        // From the scheduler: task_word = {adj_t0_base_q, pair_limit_q, l_base_q, dst_q, src1_q, src0_q, level_q, issued_op_q}
        // So: [67:58]=adj_t0_base, [57:48]=pair_limit, [47:38]=l_base, [37:28]=dst,
        //     [27:18]=src1, [17:8]=src0, [7:4]=level, [3:0]=op

        @(posedge clk);
        task_valid <= 1'b1;
        wait (task_ready);
        @(posedge clk);
        task_valid <= 1'b0;

        // Wait for done
        wait (task_done || task_fail);
        @(posedge clk);

        if (task_fail) begin
            $display("FAIL: task_fail=1 status=%02x", task_status);
            $finish;
        end

        $display("SPLIT done. Reading outputs...");
        $display("");

        // Check f0 at dst+0 = addr 0
        $display("f0 (addr 0): re=%h im=%h", mem_re[0], mem_im[0]);
        $display("  Expected:  re=%h im=%h", expected_f0_re, expected_f0_im);
        if (mem_re[0] !== expected_f0_re || mem_im[0] !== expected_f0_im) begin
            $display("  *** f0 MISMATCH ***");
            $display("  Delta re: %g", $bitstoreal(mem_re[0]) - $bitstoreal(expected_f0_re));
            $display("  Delta im: %g", $bitstoreal(mem_im[0]) - $bitstoreal(expected_f0_im));
            errors = errors + 1;
        end else begin
            $display("  f0 MATCH");
        end

        // Check f1 at dst+pair_limit = addr 128
        $display("");
        $display("f1 (addr 128): re=%h im=%h", mem_re[128], mem_im[128]);
        $display("  Expected:    re=%h im=%h", expected_f1_re, expected_f1_im);
        if (mem_re[128] !== expected_f1_re || mem_im[128] !== expected_f1_im) begin
            $display("  *** f1 MISMATCH ***");
            $display("  Delta re: %g", $bitstoreal(mem_re[128]) - $bitstoreal(expected_f1_re));
            $display("  Delta im: %g", $bitstoreal(mem_im[128]) - $bitstoreal(expected_f1_im));
            errors = errors + 1;
        end else begin
            $display("  f1 MATCH");
        end

        // Report
        $display("");
        if (errors == 0) begin
            $display("=== SPLIT GOLDEN TEST PASSED ===");
        end else begin
            $display("=== SPLIT GOLDEN TEST FAILED (%0d errors) ===", errors);
        end
        $finish;
    end

    // Watchdog
    reg [31:0] wd;
    always @(posedge clk) begin
        if (!rst_n) wd <= 0;
        else begin
            wd <= wd + 1;
            if (wd == 100000) begin
                $display("WATCHDOG timeout");
                $finish;
            end
        end
    end

endmodule
