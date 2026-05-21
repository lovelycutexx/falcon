`timescale 1ns/1ps
// SamplerZ isolation test with REAL FPU.
// Feeds known mu/sigma_inv and dumps every accepted z_out value.
// Also monitors state to detect rejection-loop stalls.

module tb_samplerz_fpu;

    reg         clk, rst_n;
    reg         cmd_valid, cmd_pair_mode;
    reg  [63:0] cmd_mu, cmd_sigma_inv, cmd_sigma_min;
    wire        cmd_ready;
    wire        rsp_valid, rsp_accept;
    reg         rsp_ready;
    wire [63:0] rsp_z0, rsp_z1;
    wire        done, fail;
    wire [7:0]  rsp_status;

    wire        fpu_req_valid, fpu_req_ready;
    wire [3:0]  fpu_req_op;
    wire [63:0] fpu_req_a, fpu_req_b, fpu_req_c;
    wire        fpu_rsp_valid, fpu_rsp_ready;
    wire [63:0] fpu_rsp_result;
    wire [1:0]  fpu_req_fmt;
    wire [2:0]  fpu_req_rm;
    wire [1:0]  fpu_req_fcvt_op;

    wire        rng_req, rng_ack;
    wire [255:0] rng_data;
    wire        busy;

    falconsign_samplerz_top #(.RNG_DATA_W(256)) u_sz (
        .clk(clk), .rst_n(rst_n),
        .cmd_valid(cmd_valid), .cmd_pair_mode(cmd_pair_mode),
        .cmd_mu(cmd_mu), .cmd_sigma_inv(cmd_sigma_inv),
        .cmd_sigma_min(cmd_sigma_min), .cmd_ready(cmd_ready),
        .rsp_valid(rsp_valid), .rsp_accept(rsp_accept),
        .rsp_ready(rsp_ready), .rsp_z0(rsp_z0), .rsp_z1(rsp_z1),
        .rsp_status(rsp_status), .done(done), .fail(fail),
        .fpu_req_valid(fpu_req_valid), .fpu_req_ready(fpu_req_ready),
        .fpu_req_op(fpu_req_op),
        .fpu_req_a(fpu_req_a), .fpu_req_b(fpu_req_b), .fpu_req_c(fpu_req_c),
        .fpu_req_fmt(fpu_req_fmt), .fpu_req_rm(fpu_req_rm),
        .fpu_req_fcvt_op(fpu_req_fcvt_op),
        .fpu_rsp_valid(fpu_rsp_valid), .fpu_rsp_ready(fpu_rsp_ready),
        .fpu_rsp_result(fpu_rsp_result),
        .rng_req(rng_req), .rng_ack(rng_ack),
        .rng_data(rng_data), .busy(busy)
    );

    // ─── Real FPU ───
    falcon_fp_fpu u_fpu (
        .clk(clk), .rst_n(rst_n),
        .req_valid(fpu_req_valid), .req_ready(fpu_req_ready),
        .req_op(fpu_req_op), .req_a(fpu_req_a), .req_b(fpu_req_b), .req_c(fpu_req_c),
        .req_fmt(fpu_req_fmt), .req_rm(fpu_req_rm), .req_fcvt_op(fpu_req_fcvt_op),
        .rsp_valid(fpu_rsp_valid), .rsp_ready(fpu_rsp_ready),
        .rsp_result(fpu_rsp_result), .rsp_flags(), .busy()
    );

    // ─── Simple LFSR RNG ───
    reg [255:0] rng_state;
    reg         rng_valid;
    assign rng_data = rng_state;
    assign rng_ack  = rng_req && rng_valid;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rng_state <= 256'hDEADBEEF01234567CAFEBABE89ABCDEF0123456789ABCDEFFEEDFACE12345678;
            rng_valid <= 1'b0;
        end else begin
            if (rng_req) begin
                rng_state <= {rng_state[254:0], rng_state[255] ^ rng_state[251] ^ rng_state[246] ^ rng_state[241]};
                rng_valid <= 1'b1;
            end else begin
                rng_valid <= 1'b0;
            end
        end
    end

    // ─── Clock ───
    initial clk = 1'b0;
    always #5 clk = ~clk;

    // ─── Test ───
    integer i, accepted, rejected, cycles;
    real    mu_r, z_r, sigma_r;

    initial begin
        rst_n = 1'b0;
        cmd_valid = 1'b0;
        rsp_ready = 1'b1;
        accepted = 0;
        rejected = 0;

        repeat (10) @(posedge clk);
        rst_n = 1'b1;
        repeat (5) @(posedge clk);

        $display("=== SamplerZ + Real FPU Test ===");

        // Test 1: mu=5.0, sigma_inv=0.7
        $display("\n--- Test 1: mu=5.0, sigma_inv=0.7 ---");
        cmd_mu        = 64'h4014000000000000;
        cmd_sigma_inv = 64'h3fe6666666666666;
        cmd_sigma_min = 64'h3ff3333333333333;
        cmd_pair_mode = 1'b0;

        for (i = 0; i < 10; i = i + 1) begin
            @(posedge clk);
            cmd_valid <= 1'b1;
            wait (cmd_ready);
            @(posedge clk);
            cmd_valid <= 1'b0;

            cycles = 0;
            while (!rsp_valid && cycles < 50000) begin
                @(posedge clk);
                cycles = cycles + 1;
            end

            if (rsp_valid) begin
                mu_r = $bitstoreal(cmd_mu);
                z_r  = $bitstoreal(rsp_z0);
                sigma_r = 1.0 / $bitstoreal(cmd_sigma_inv);
                if (rsp_accept) begin
                    $display("  [%0d] z=%0.1f mu=%0.1f |z-mu|=%0.1f sigma=%.2f cycles=%0d ACCEPT",
                        i, z_r, mu_r, (z_r>mu_r ? z_r-mu_r : mu_r-z_r), sigma_r, cycles);
                    accepted = accepted + 1;
                end else begin
                    rejected = rejected + 1;
                end
            end else begin
                $display("  [%0d] TIMEOUT (state=%0d)", i, u_sz.st);
                rejected = rejected + 1;
            end
            @(posedge clk);
        end

        // Test 2: mu from realistic tree leaf
        $display("\n--- Test 2: mu=120.0 (typical leaf), sigma_inv=0.77 (tree value) ---");
        cmd_mu        = 64'h405e000000000000;  // 120.0
        cmd_sigma_inv = 64'h3fe8a58ba9052ab4;  // ~0.77 (from real tree)
        cmd_sigma_min = 64'h3ff3333333333333;

        for (i = 0; i < 10; i = i + 1) begin
            @(posedge clk);
            cmd_valid <= 1'b1;
            wait (cmd_ready);
            @(posedge clk);
            cmd_valid <= 1'b0;

            cycles = 0;
            while (!rsp_valid && cycles < 50000) begin
                @(posedge clk);
                cycles = cycles + 1;
            end

            if (rsp_valid && rsp_accept) begin
                mu_r = $bitstoreal(cmd_mu);
                z_r  = $bitstoreal(rsp_z0);
                sigma_r = 1.0 / $bitstoreal(cmd_sigma_inv);
                $display("  [%0d] z=%0.1f mu=%0.1f diff=%0.1f sigma=%.2f cycles=%0d ACCEPT",
                    i, z_r, mu_r, (z_r>mu_r ? z_r-mu_r : mu_r-z_r), sigma_r, cycles);
                accepted = accepted + 1;
            end else if (!rsp_valid) begin
                $display("  [%0d] TIMEOUT (state=%0d)", i, u_sz.st);
                rejected = rejected + 1;
            end
            @(posedge clk);
        end

        // Test 3: mu=2500 (larger leaf value)
        $display("\n--- Test 3: mu=2500.0, sigma_inv=0.77 ---");
        cmd_mu        = 64'h40a3880000000000;  // 2500.0
        cmd_sigma_inv = 64'h3fe8a58ba9052ab4;
        cmd_sigma_min = 64'h3ff3333333333333;

        for (i = 0; i < 10; i = i + 1) begin
            @(posedge clk);
            cmd_valid <= 1'b1;
            wait (cmd_ready);
            @(posedge clk);
            cmd_valid <= 1'b0;

            cycles = 0;
            while (!rsp_valid && cycles < 50000) begin
                @(posedge clk);
                cycles = cycles + 1;
            end

            if (rsp_valid && rsp_accept) begin
                mu_r = $bitstoreal(cmd_mu);
                z_r  = $bitstoreal(rsp_z0);
                sigma_r = 1.0 / $bitstoreal(cmd_sigma_inv);
                $display("  [%0d] z=%0.1f mu=%0.1f diff=%0.1f sigma=%.2f cycles=%0d ACCEPT",
                    i, z_r, mu_r, (z_r>mu_r ? z_r-mu_r : mu_r-z_r), sigma_r, cycles);
                accepted = accepted + 1;
            end else if (!rsp_valid) begin
                $display("  [%0d] TIMEOUT (state=%0d)", i, u_sz.st);
                rejected = rejected + 1;
            end
            @(posedge clk);
        end

        $display("\n=== Results: %0d accepted, %0d rejected ===", accepted, rejected);
        if (rejected == 0)
            $display("SamplerZ+FPU PASSED");
        else
            $display("SamplerZ+FPU FAILED (%0d timeouts)", rejected);
        $finish;
    end

    // Watchdog
    reg [31:0] wd;
    always @(posedge clk) begin
        if (!rst_n) wd <= 0;
        else begin
            wd <= wd + 1;
            if (wd == 2000000) begin
                $display("GLOBAL WATCHDOG state=%0d", u_sz.st);
                $finish;
            end
        end
    end

endmodule
