`timescale 1ns/1ps
// Standalone SamplerZ test — verify it produces reasonable samples
// with known mu and sigma_inv values.

module tb_samplerz_standalone;

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
    reg         fpu_rsp_valid;
    wire        fpu_rsp_ready;
    reg  [63:0] fpu_rsp_result;

    wire        rng_req, rng_ack;
    wire [255:0] rng_data;
    wire        busy;

    falconsign_samplerz_top #(.RNG_DATA_W(256)) dut (
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
        .fpu_req_fmt(), .fpu_req_rm(), .fpu_req_fcvt_op(),
        .fpu_rsp_valid(fpu_rsp_valid), .fpu_rsp_ready(fpu_rsp_ready),
        .fpu_rsp_result(fpu_rsp_result),
        .rng_req(rng_req), .rng_ack(rng_ack),
        .rng_data(rng_data), .busy(busy)
    );

    // ─── Software FPU model ───
    function [63:0] f64_add; input [63:0] a,b; begin f64_add = $realtobits($bitstoreal(a)+$bitstoreal(b)); end endfunction
    function [63:0] f64_sub; input [63:0] a,b; begin f64_sub = $realtobits($bitstoreal(a)-$bitstoreal(b)); end endfunction
    function [63:0] f64_mul; input [63:0] a,b; begin f64_mul = $realtobits($bitstoreal(a)*$bitstoreal(b)); end endfunction

    assign fpu_req_ready = 1'b1;

    always @(posedge clk) begin
        fpu_rsp_valid <= fpu_req_valid;
        if (fpu_req_valid) begin
            case (fpu_req_op)
                4'd0: fpu_rsp_result <= f64_add(fpu_req_a, fpu_req_b);  // FADD
                4'd1: fpu_rsp_result <= f64_sub(fpu_req_a, fpu_req_b);  // FSUB
                4'd2: fpu_rsp_result <= f64_mul(fpu_req_a, fpu_req_b);  // FMUL
                4'd3: fpu_rsp_result <= f64_add(f64_mul(fpu_req_a, fpu_req_b), fpu_req_c); // FMADD
                4'd9: fpu_rsp_result <= fpu_req_a;   // FC: convert
                4'd12: begin  // FF: floor
                    real val;
                    val = $bitstoreal(fpu_req_a);
                    fpu_rsp_result <= $realtobits(val < 0.0 ? ($rtoi(val)-1.0) : $rtoi(val));
                end
                4'd13: fpu_rsp_result <= $realtobits($itor($signed(fpu_req_a))); // FI: int-to-float
                default: fpu_rsp_result <= 64'd0;
            endcase
        end
    end
    // ─── Fake RNG (simple LFSR) ───
    reg [255:0] rng_state;
    reg         rng_data_valid;
    assign rng_data = rng_state;
    assign rng_ack  = rng_req && rng_data_valid;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rng_state <= 256'h0123456789ABCDEF0123456789ABCDEF0123456789ABCDEF0123456789ABCDEF;
            rng_data_valid <= 1'b0;
        end else begin
            if (rng_req) begin
                rng_state <= {rng_state[254:0], rng_state[255] ^ rng_state[251] ^ rng_state[246] ^ rng_state[242]};
                rng_data_valid <= 1'b1;
            end else begin
                rng_data_valid <= 1'b0;
            end
        end
    end

    // ─── Clock ───
    initial clk = 1'b0;
    always #5 clk = ~clk;

    // ─── Test ───
    integer i, pass_count, fail_count;
    reg [63:0] z_val;
    real       mu_real, z_real, sigma_eff;

    initial begin
        rst_n = 1'b0;
        cmd_valid = 1'b0;
        rsp_ready = 1'b1;
        pass_count = 0;
        fail_count = 0;

        repeat (10) @(posedge clk);
        rst_n = 1'b1;
        repeat (5) @(posedge clk);

        $display("=== SamplerZ Standalone Test ===");

        // Test 1: Simple sample with mu=5.0, sigma_inv=0.7 (~sigma=1.43)
        cmd_mu        = 64'h4014000000000000;  // 5.0
        cmd_sigma_inv = 64'h3fe6666666666666;  // ~0.7
        cmd_sigma_min = 64'h3ff3333333333333;  // ~1.2
        cmd_pair_mode = 1'b0;

        for (i = 0; i < 20; i = i + 1) begin
            @(posedge clk);
            cmd_valid <= 1'b1;
            wait (cmd_ready);
            @(posedge clk);
            cmd_valid <= 1'b0;

            wait (rsp_valid);
            z_val = rsp_z0;
            mu_real = $bitstoreal(cmd_mu);
            z_real  = $bitstoreal(z_val);
            sigma_eff = 1.0 / $bitstoreal(cmd_sigma_inv);

            if (rsp_accept) begin
                $display("Sample[%0d]: z=%0.1f mu=%0.1f |z-mu|=%0.1f sigma_eff=%0.2f ACCEPT",
                    i, z_real, mu_real,
                    z_real > mu_real ? z_real-mu_real : mu_real-z_real,
                    sigma_eff);
                if (z_real > mu_real ? (z_real-mu_real) < 10.0*sigma_eff : (mu_real-z_real) < 10.0*sigma_eff)
                    pass_count = pass_count + 1;
                else begin
                    $display("  *** z too far from mu! ***");
                    fail_count = fail_count + 1;
                end
            end else begin
                $display("Sample[%0d]: REJECT", i);
            end

            @(posedge clk);
        end

        // Test 2: mu=0 (center at zero)
        cmd_mu        = 64'h0000000000000000;  // 0.0
        cmd_sigma_inv = 64'h3fe6666666666666;  // ~0.7
        cmd_sigma_min = 64'h3ff3333333333333;  // ~1.2

        for (i = 0; i < 20; i = i + 1) begin
            @(posedge clk);
            cmd_valid <= 1'b1;
            wait (cmd_ready);
            @(posedge clk);
            cmd_valid <= 1'b0;

            wait (rsp_valid);
            z_val = rsp_z0;
            mu_real = 0.0;
            z_real  = $bitstoreal(z_val);
            sigma_eff = 1.0 / $bitstoreal(cmd_sigma_inv);

            if (rsp_accept) begin
                $display("Sample[%0d]: z=%0.1f |z|=%0.1f sigma=%0.2f ACCEPT",
                    i, z_real, z_real < 0 ? -z_real : z_real, sigma_eff);
                if ((z_real < 0 ? -z_real : z_real) < 10.0*sigma_eff)
                    pass_count = pass_count + 1;
                else begin
                    $display("  *** z too far from 0! ***");
                    fail_count = fail_count + 1;
                end
            end else begin
                $display("Sample[%0d]: REJECT", i);
            end

            @(posedge clk);
        end

        // Test 3: Larger mu (typical t-value at leaf)
        cmd_mu        = 64'h40b0000000000000;  // ~4096
        cmd_sigma_inv = 64'h3fe8a58ba9052ab4;  // ~0.77 (real tree value)
        cmd_sigma_min = 64'h3ff3333333333333;  // ~1.2

        for (i = 0; i < 20; i = i + 1) begin
            @(posedge clk);
            cmd_valid <= 1'b1;
            wait (cmd_ready);
            @(posedge clk);
            cmd_valid <= 1'b0;

            wait (rsp_valid);
            z_val = rsp_z0;
            mu_real = $bitstoreal(cmd_mu);
            z_real  = $bitstoreal(z_val);
            sigma_eff = 1.0 / $bitstoreal(cmd_sigma_inv);

            if (rsp_accept) begin
                $display("Sample[%0d]: z=%0.1f mu=%0.1f diff=%0.1f sigma=%0.2f ACCEPT",
                    i, z_real, mu_real,
                    z_real > mu_real ? z_real-mu_real : mu_real-z_real,
                    sigma_eff);
                if ((z_real > mu_real ? z_real-mu_real : mu_real-z_real) < 10.0*sigma_eff)
                    pass_count = pass_count + 1;
                else begin
                    $display("  *** z too far! ***");
                    fail_count = fail_count + 1;
                end
            end else begin
                $display("Sample[%0d]: REJECT", i);
            end

            @(posedge clk);
        end

        $display("");
        $display("=== Results: %0d pass, %0d fail ===", pass_count, fail_count);
        if (fail_count == 0)
            $display("SamplerZ PASSED");
        else
            $display("SamplerZ FAILED (%0d bad samples)", fail_count);
        $finish;
    end

    // Watchdog
    reg [31:0] wd;
    always @(posedge clk) begin
        if (!rst_n) wd <= 0;
        else begin
            wd <= wd + 1;
            if (wd == 500000) begin
                $display("WATCHDOG at state=%0d", dut.st);
                $finish;
            end
        end
    end

endmodule
