`timescale 1ns/1ps

// Standalone FPU test — verify FMADD / FCMP / FCVT correctness
module tb_falcon_fpu_direct;

    reg        clk, rst_n;
    reg        req_valid;
    wire       req_ready;
    reg [3:0]  req_op;
    reg [63:0] req_a, req_b, req_c;
    wire       rsp_valid;
    reg        rsp_ready;
    wire [63:0] rsp_result;
    wire [4:0]  rsp_flags;

    falcon_fp_fpu u_fpu (
        .clk(clk), .rst_n(rst_n),
        .req_valid(req_valid), .req_ready(req_ready),
        .req_op(req_op), .req_a(req_a), .req_b(req_b), .req_c(req_c),
        .req_fmt(2'b01), .req_rm(3'b000), .req_fcvt_op(2'b00),
        .rsp_valid(rsp_valid), .rsp_ready(rsp_ready),
        .rsp_result(rsp_result), .rsp_flags(rsp_flags), .busy()
    );

    always #5 clk = ~clk;

    task fpu_cmd;
        input [3:0]  op;
        input [63:0] a, b, c;
        begin
            @(posedge clk);
            while (!req_ready) @(posedge clk);
            req_valid <= 1'b1; req_op <= op; req_a <= a; req_b <= b; req_c <= c;
            @(posedge clk);
            req_valid <= 1'b0;
            while (!rsp_valid) @(posedge clk);
            @(posedge clk);  // sample after posedge
        end
    endtask

    integer fails = 0;
    reg [63:0] exp;

    initial begin
        clk = 0; rst_n = 0; req_valid = 0; rsp_ready = 1;
        #20 rst_n = 1;
        #10;

        $display("=== FMADD(1/3, 2.0, -1.0) ===");
        fpu_cmd(4'd3, 64'h3FD5555555555555, 64'h4000000000000000, 64'hBFF0000000000000);
        if (rsp_result !== 64'hBFD5555555555555 && rsp_result !== 64'hBFD5555555555556) begin
            $display("FAIL: got %h exp bfd5555555555555/bfd5555555555556", rsp_result); fails = fails + 1;
        end else $display("PASS: %h", rsp_result);

        // 2. FMADD(1/24, 2.0, -1/6) = 1/24*2 - 1/6 = -1/12
        $display("=== FMADD(1/24, 2.0, -1/6) ===");
        fpu_cmd(4'd3, 64'h3FA5555555555555, 64'h4000000000000000, 64'hBFC5555555555555);
        exp = 64'hBFB5555555555555;  // -1/12
        if (rsp_result !== exp) begin
            $display("FAIL: got %h exp %h", rsp_result, exp); fails = fails + 1;
        end else $display("PASS: %h", rsp_result);

        // 3. FCVT_F2I(5.0) = 5
        $display("=== FCVT_F2I(5.0) ===");
        fpu_cmd(4'd12, 64'h4014000000000000, 64'd0, 64'd0);
        exp = 64'd5;
        if (rsp_result !== exp) begin
            $display("FAIL: got %h exp %h", rsp_result, exp); fails = fails + 1;
        end else $display("PASS: %h", rsp_result);

        // 4. FCVT_I2F(5) = 5.0
        $display("=== FCVT_I2F(5) ===");
        fpu_cmd(4'd13, 64'd5, 64'd0, 64'd0);
        exp = 64'h4014000000000000;
        if (rsp_result !== exp) begin
            $display("FAIL: got %h exp %h", rsp_result, exp); fails = fails + 1;
        end else $display("PASS: %h", rsp_result);

        // 5. FCMP(0.5, 0.3) = 1.0 (0.5 >= 0.3)
        $display("=== FCMP(0.5, 0.3) ===");
        fpu_cmd(4'd9, 64'h3FE0000000000000, 64'h3FD3333333333333, 64'd0);
        exp = 64'h3FF0000000000000;  // 1.0
        if (rsp_result !== exp) begin
            $display("FAIL: got %h exp %h", rsp_result, exp); fails = fails + 1;
        end else $display("PASS: %h", rsp_result);

        // 6. FCMP(0.3, 0.5) = 0.0 (0.3 < 0.5)
        $display("=== FCMP(0.3, 0.5) ===");
        fpu_cmd(4'd9, 64'h3FD3333333333333, 64'h3FE0000000000000, 64'd0);
        exp = 64'd0;
        if (rsp_result !== exp) begin
            $display("FAIL: got %h exp %h", rsp_result, exp); fails = fails + 1;
        end else $display("PASS: %h", rsp_result);

        // 7. FADD(2/3, -1.0) = -1/3
        $display("=== FADD(2/3, -1.0) ===");
        fpu_cmd(4'd0, 64'h3FE5555555555555, 64'hBFF0000000000000, 64'd0);
        if (rsp_result !== 64'hBFD5555555555555 && rsp_result !== 64'hBFD5555555555556) begin
            $display("FAIL: got %h exp bfd5555555555555 or bfd5555555555556", rsp_result); fails = fails + 1;
        end else $display("PASS: %h", rsp_result);

        // 8. FMUL(1/3, 2.0) = 2/3  (the multiply step of the broken FMADD)
        $display("=== FMUL(1/3, 2.0) ===");
        fpu_cmd(4'd2, 64'h3FD5555555555555, 64'h4000000000000000, 64'd0);
        exp = 64'h3FE5555555555555;
        if (rsp_result !== exp) begin
            $display("FAIL: got %h exp %h", rsp_result, exp); fails = fails + 1;
        end else $display("PASS: %h", rsp_result);

        // Report
        if (fails) $display("FAIL: %0d errors", fails);
        else       $display("PASS: all tests passed");
        $finish;
    end

endmodule
