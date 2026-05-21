`timescale 1ns/1ps
// Comprehensive ffSampling EXU tests: SPLIT, MERGE, ADJUST, READ_L10.

module tb_ffsampling;
    reg clk, rst_n;
    reg [255:0] mem [0:1023];

    // EXU signals
    reg  task_valid; wire task_ready; reg [67:0] task_word;
    wire task_done, task_fail; wire [7:0] task_status;
    wire mem_rd_en; wire [9:0] mem_rd_addr; wire [255:0] mem_rd_data;
    wire mem_wr_en; wire [9:0] mem_wr_addr; wire [255:0] mem_wr_data;
    wire [9:0] tw_addr; reg [63:0] tw_re, tw_im;
    wire fpu_req_valid; wire fpu_req_ready = 1;
    wire [3:0] fpu_req_op; wire [63:0] fpu_req_a, fpu_req_b, fpu_req_c;
    wire fpu_rsp_valid; wire [63:0] fpu_rsp_result;
    wire sz_cmd_valid; wire sz_cmd_ready;
    reg [63:0] sz_cmd_mu, sz_cmd_sigma_inv; reg sz_cmd_pair;
    reg sz_rsp_valid; reg [63:0] sz_rsp_z0, sz_rsp_z1;

    falcon_f64_ffsampling_exu u_fe(
        .clk(clk),.rst_n(rst_n),
        .task_valid(task_valid),.task_ready(task_ready),
        .task_word(task_word),.task_done(task_done),.task_fail(task_fail),
        .task_status(task_status),
        .mem_rd_en(mem_rd_en),.mem_rd_addr(mem_rd_addr),
        .mem_rd_data(mem_rd_data),.mem_wr_en(mem_wr_en),
        .mem_wr_addr(mem_wr_addr),.mem_wr_data(mem_wr_data),
        .twiddle_addr(tw_addr),.twiddle_re(tw_re),.twiddle_im(tw_im),
        .fpu_req_valid(fpu_req_valid),.fpu_req_ready(fpu_req_ready),
        .fpu_req_op(fpu_req_op),.fpu_req_a(fpu_req_a),
        .fpu_req_b(fpu_req_b),.fpu_req_c(fpu_req_c),
        .fpu_rsp_valid(fpu_rsp_valid),.fpu_rsp_result(fpu_rsp_result),
        .sz_cmd_valid(sz_cmd_valid),.sz_cmd_ready(sz_cmd_ready),
        .sz_cmd_mu(sz_cmd_mu),.sz_cmd_sigma_inv(sz_cmd_sigma_inv),
        .sz_cmd_pair(sz_cmd_pair),.sz_rsp_valid(sz_rsp_valid),
        .sz_rsp_z0(sz_rsp_z0),.sz_rsp_z1(sz_rsp_z1));

    always #5 clk = ~clk;
    // Combinational read + registered write
    assign mem_rd_data = mem_rd_en ? mem[mem_rd_addr] : 256'd0;
    always @(posedge clk) if (mem_wr_en) mem[mem_wr_addr] <= mem_wr_data;

    // FPU model: 1-cycle delay
    reg fpu_val_q; reg [63:0] fpu_res_q;
    always @(posedge clk) begin
        if (fpu_req_valid) begin
            case (fpu_req_op)
                0: fpu_res_q <= $realtobits($bitstoreal(fpu_req_a) + $bitstoreal(fpu_req_b));
                1: fpu_res_q <= $realtobits($bitstoreal(fpu_req_a) - $bitstoreal(fpu_req_b));
                2: fpu_res_q <= $realtobits($bitstoreal(fpu_req_a) * $bitstoreal(fpu_req_b));
                3: fpu_res_q <= $realtobits($bitstoreal(fpu_req_a) * $bitstoreal(fpu_req_b) + $bitstoreal(fpu_req_c));
                4: fpu_res_q <= $realtobits($bitstoreal(fpu_req_a) * $bitstoreal(fpu_req_b) - $bitstoreal(fpu_req_c));
                6: fpu_res_q <= $realtobits(-$bitstoreal(fpu_req_a) * $bitstoreal(fpu_req_b) + $bitstoreal(fpu_req_c));
                default: fpu_res_q <= 0;
            endcase
            fpu_val_q <= 1;
        end else fpu_val_q <= 0;
    end
    assign fpu_rsp_valid = fpu_val_q;
    assign fpu_rsp_result = fpu_res_q;

    // SamplerZ stub
    assign sz_cmd_ready = 1;
    always @(posedge clk) if (sz_cmd_valid) sz_rsp_valid <= 1; else sz_rsp_valid <= 0;

    integer pass, fail;
    reg [63:0] a_re, a_im, b_re, b_im;
    reg [63:0] r0, r1;

    // Send task and wait for completion
    task send_task;
        input [67:0] tw;
        begin
            @(posedge clk); task_valid=1; task_word=tw;
            @(posedge clk); task_valid=0;
            while (!task_done) @(posedge clk);
            @(posedge clk);  // settle
        end
    endtask

    // Check with tolerance (default 1e-9)
    task check;
        input [63:0] got; input [63:0] exp; input [80*8:0] msg;
        real g, e, err;
        begin
            g = $bitstoreal(got); e = $bitstoreal(exp);
            err = (g - e); if (err < 0) err = -err;
            if (err < 1e-9 || (e != 0 && err/e < 1e-9)) begin
                $display("  PASS: %0s (%f)", msg, g);
                pass = pass + 1;
            end else begin
                $display("  FAIL: %0s (got %f exp %f, err=%e)", msg, g, e, err);
                fail = fail + 1;
            end
        end
    endtask

    initial begin
        clk=0; rst_n=0; task_valid=0; task_word=0;
        tw_re=0; tw_im=0; sz_rsp_valid=0;
        pass=0; fail=0;
        #20 rst_n=1; #10;

        // ============================================================
        // TEST 1: SPLIT with w=1 (a=1+2i, b=3+4i)
        // ============================================================
        $display("=== TEST 1: SPLIT w=1 ===");
        mem[0] = {64'd0,64'd0,$realtobits(2.0),$realtobits(1.0)};  // a
        mem[1] = {64'd0,64'd0,$realtobits(4.0),$realtobits(3.0)};  // b
        tw_re = $realtobits(1.0); tw_im = $realtobits(0.0);
        send_task({4'd1,4'd8,10'd0,14'd0,14'd0,14'd0,8'd0});  // SPLIT, level=8
        r0 = mem[0][127:64]; r1 = mem[0][63:0];  // y0_im, y0_re
        check(r0, $realtobits(3.0), "y0_re=2, y0_im=3");
        // Actually y0_im is at [127:64], y0_re at [63:0]
        // wait, mem[0] = {128'd0, y0_im, y0_re} = {64'd0,64'd0,y0_im,y0_re}
        // So [127:64] = y0_im = 3.0, [63:0] = y0_re = 2.0
        r0 = mem[0][63:0]; r1 = mem[0][127:64];
        check(r0, $realtobits(2.0), "y0_re=2");
        check(r1, $realtobits(3.0), "y0_im=3");
        r0 = mem[1][63:0]; r1 = mem[1][127:64];
        check(r0, $realtobits(-1.0), "y1_re=-1");
        check(r1, $realtobits(-1.0), "y1_im=-1");

        // ============================================================
        // TEST 2: SPLIT with w = exp(pi/4)  (45-degree rotation)
        // a=1+0i, b=0+1i, w=cos(pi/4)+i*sin(pi/4)=0.707+0.707i
        // ============================================================
        $display("=== TEST 2: SPLIT w=exp(pi/4) ===");
        mem[0] = {64'd0,64'd0,64'd0,$realtobits(1.0)};  // a=1+0i
        mem[1] = {64'd0,64'd0,$realtobits(1.0),64'd0};  // b=0+1i
        tw_re = $realtobits(0.7071067811865476);
        tw_im = $realtobits(0.7071067811865476);
        send_task({4'd1,4'd8,10'd0,14'd0,14'd0,14'd0,8'd0});  // SPLIT

        // Expected: y0=(a+b)/2 = (1+0i+0+1i)/2 = 0.5+0.5i
        // y1=((a-b)*conj(w))/2
        // a-b = 1+0i-(0+1i) = 1-1i
        // conj(w) = 0.707-0.707i
        // (1-1i)*(0.707-0.707i) = 0.707-0.707i-0.707i+0.707i^2
        // = 0.707-1.414i-0.707 = 0-1.414i
        // y1 = (0-1.414i)/2 = 0-0.707i
        r0 = mem[0][63:0]; r1 = mem[0][127:64];
        check(r0, $realtobits(0.5), "y0_re=0.5");
        check(r1, $realtobits(0.5), "y0_im=0.5");
        r0 = mem[1][63:0]; r1 = mem[1][127:64];
        check(r0, $realtobits(0.0), "y1_re=0");
        check(r1, $realtobits(-0.70710678), "y1_im=-0.707");

        // ============================================================
        // TEST 3: MERGE with w=1 (inverse of TEST 1)
        // ============================================================
        $display("=== TEST 3: MERGE w=1 ===");
        // mergefft output = a + b*w, a - b*w  (no /2)
        // Input: a=y0=(2+3i), b=y1=(-1-i), w=1
        // Expected: a+b*w = 2+3i+(-1-i) = 1+2i
        //           a-b*w = 2+3i-(-1-i) = 3+4i
        mem[0] = {64'd0,64'd0,$realtobits(3.0),$realtobits(2.0)};  // a
        mem[1] = {64'd0,64'd0,$realtobits(-1.0),$realtobits(-1.0)};  // b
        tw_re = $realtobits(1.0); tw_im = $realtobits(0.0);
        send_task({4'd4,4'd8,10'd0,14'd0,14'd0,14'd0,8'd0});  // MERGE (op=4)
        r0 = mem[0][63:0]; r1 = mem[0][127:64];
        check(r0, $realtobits(1.0), "m0_re=1");
        check(r1, $realtobits(2.0), "m0_im=2");
        r0 = mem[1][63:0]; r1 = mem[1][127:64];
        check(r0, $realtobits(3.0), "m1_re=3");
        check(r1, $realtobits(4.0), "m1_im=4");

        // ============================================================
        // TEST 4: ADJUST_T0  t0 += (t1 - z1) * conj(L10_re)
        // t0=5+0i, t1=1+1i, z1=0+0i, L10=2+0i
        // ============================================================
        $display("=== TEST 4: ADJUST_T0 ===");
        // ADJUST reads t1 word at src0+(ci>>2), z1 at src1+(ci>>2), t0 at dst+(ci>>2)
        // For level=8, pair_limit=1, ci starts at 0
        // t1 at addr src0+0 = s0, z1 at addr src1+0 = s1, t0 at dst+0 = dt
        // ADJUST: t0 += (t1-z1) * conj(L10)
        // Fill ALL lanes and words so every coefficient gets correct data.
        // ADJUST for level=8 processes 2 words × 2 lanes = 4 coefficients.
        mem[50] = {64'd0,64'd0,$realtobits(0.0),$realtobits(2.0)};  // L10=2+0i
        mem[100] = {$realtobits(1.0),$realtobits(1.0),$realtobits(1.0),$realtobits(1.0)}; // t1
        mem[101] = {$realtobits(1.0),$realtobits(1.0),$realtobits(1.0),$realtobits(1.0)}; // t1
        mem[200] = {64'd0,64'd0,64'd0,64'd0};  // z1
        mem[201] = {64'd0,64'd0,64'd0,64'd0};  // z1
        mem[300] = {$realtobits(0.0),$realtobits(5.0),$realtobits(0.0),$realtobits(5.0)}; // t0
        mem[301] = {$realtobits(0.0),$realtobits(5.0),$realtobits(0.0),$realtobits(5.0)}; // t0
        // Load L10 from addr 50 via READ_L10 (src1=50)
        send_task({4'd0,4'd8,10'd0,14'd0,14'd50,14'd0,8'd0});  // READ_L10(src1=50)
        $display("  L10 loaded: re=%f im=%f", $bitstoreal(u_fe.l_re_q), $bitstoreal(u_fe.l_im_q));
        // Now send ADJUST: op=2, level=8, src0=100, src1=200, dst=300
        send_task({4'd2,4'd8,10'd0,14'd100,14'd200,14'd300,8'd0});
        // Expected: t0_new = t0 + (t1-z1)*conj(L10)
        // conj(L10) = 2-0i = 2
        // (t1-z1) = (1+1i)-(0+0i) = 1+1i
        // (t1-z1)*conj(L10) = (1+1i)*2 = 2+2i
        // t0_new = 5+0i + 2+2i = 7+2i
        // Read back from mem[300]
        r0 = mem[300][63:0]; r1 = mem[300][127:64];
        check(r0, $realtobits(7.0), "adj_re=7");
        check(r1, $realtobits(2.0), "adj_im=2");

        // ============================================================
        // Summary
        // ============================================================
        $display("");
        if (fail == 0)
            $display("PASS: all %0d tests passed", pass);
        else
            $display("FAIL: %0d passed, %0d failed", pass, fail);
        $finish;
    end
endmodule
