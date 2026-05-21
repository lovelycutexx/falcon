`timescale 1ns/1ps

module tb_falcon_f64_complex_bfly;

    localparam [3:0] OP_FADD    = 4'd0;
    localparam [3:0] OP_FMUL    = 4'd2;
    localparam [3:0] OP_FMADD   = 4'd3;
    localparam [3:0] OP_FNMSUB  = 4'd5;
    localparam [3:0] OP_FNMADD  = 4'd6;

    reg         clk;
    reg         rst_n;
    reg         in_valid;
    wire        in_ready;
    reg  [63:0] a_re;
    reg  [63:0] a_im;
    reg  [63:0] b_re;
    reg  [63:0] b_im;
    reg  [63:0] w_re;
    reg  [63:0] w_im;
    wire        out_valid;
    reg         out_ready;
    wire [63:0] y0_re;
    wire [63:0] y0_im;
    wire [63:0] y1_re;
    wire [63:0] y1_im;
    wire        status_invalid;
    wire        status_overflow;
    wire        status_underflow;
    wire        status_inexact;
    wire        busy;

    reg         fpu_req_valid;
    wire        fpu_req_ready;
    reg  [3:0]  fpu_req_op;
    reg  [63:0] fpu_req_a;
    reg  [63:0] fpu_req_b;
    reg  [63:0] fpu_req_c;
    wire        fpu_rsp_valid;
    wire [63:0] fpu_rsp_result;
    wire [4:0]  fpu_rsp_flags;
    wire        fpu_busy;

    integer error_count;
    integer cycle_count;

    real ra_re;
    real ra_im;
    real rb_re;
    real rb_im;
    real rw_re;
    real rw_im;
    real exp_y0_re_r;
    real exp_y0_im_r;
    real exp_y1_re_r;
    real exp_y1_im_r;
    reg [63:0] exp_y0_re;
    reg [63:0] exp_y0_im;
    reg [63:0] exp_y1_re;
    reg [63:0] exp_y1_im;

    falcon_fp_fpu u_fpu (
        .clk         (clk),
        .rst_n       (rst_n),
        .req_valid   (fpu_req_valid),
        .req_ready   (fpu_req_ready),
        .req_op      (fpu_req_op),
        .req_a       (fpu_req_a),
        .req_b       (fpu_req_b),
        .req_c       (fpu_req_c),
        .req_fmt     (2'b01),
        .req_rm      (3'b000),
        .req_fcvt_op (2'b00),
        .rsp_valid   (fpu_rsp_valid),
        .rsp_ready   (1'b1),
        .rsp_result  (fpu_rsp_result),
        .rsp_flags   (fpu_rsp_flags),
        .busy        (fpu_busy)
    );

    falcon_f64_complex_bfly dut (
        .clk              (clk),
        .rst_n            (rst_n),
        .in_valid         (in_valid),
        .in_ready         (in_ready),
        .a_re             (a_re),
        .a_im             (a_im),
        .b_re             (b_re),
        .b_im             (b_im),
        .w_re             (w_re),
        .w_im             (w_im),
        .out_valid        (out_valid),
        .out_ready        (out_ready),
        .y0_re            (y0_re),
        .y0_im            (y0_im),
        .y1_re            (y1_re),
        .y1_im            (y1_im),
        .status_invalid   (status_invalid),
        .status_overflow  (status_overflow),
        .status_underflow (status_underflow),
        .status_inexact   (status_inexact),
        .busy             (busy)
    );

    always #5 clk = ~clk;

    task check_equal64;
        input [127:0] name;
        input [63:0] actual;
        input [63:0] expected;
        begin
            if (actual !== expected) begin
                $display("TB_FAIL %0s expected=%h actual=%h", name, expected, actual);
                error_count = error_count + 1;
            end
        end
    endtask

    task run_fpu_case;
        input [127:0] name;
        input [3:0]   top;
        input [63:0]  ta;
        input [63:0]  tb;
        input [63:0]  tc;
        input [63:0]  expected;
        begin
            @(posedge clk);
            while (!fpu_req_ready) begin
                @(posedge clk);
            end

            fpu_req_op    <= top;
            fpu_req_a     <= ta;
            fpu_req_b     <= tb;
            fpu_req_c     <= tc;
            fpu_req_valid <= 1'b1;
            @(posedge clk);
            fpu_req_valid <= 1'b0;

            cycle_count = 0;
            while (!fpu_rsp_valid) begin
                @(posedge clk);
                cycle_count = cycle_count + 1;
                if (cycle_count > 20) begin
                    $display("TB_FAIL %0s timeout", name);
                    error_count = error_count + 1;
                    disable run_fpu_case;
                end
            end

            #1;
            check_equal64(name, fpu_rsp_result, expected);
            if (fpu_rsp_flags[4] || fpu_rsp_flags[2] || fpu_rsp_flags[1]) begin
                $display("TB_FAIL %0s unexpected flags=%b", name, fpu_rsp_flags);
                error_count = error_count + 1;
            end

            @(posedge clk);
        end
    endtask

    task run_bfly_case;
        input [63:0] ta_re;
        input [63:0] ta_im;
        input [63:0] tb_re;
        input [63:0] tb_im;
        input [63:0] tw_re;
        input [63:0] tw_im;
        begin
            ra_re = $bitstoreal(ta_re);
            ra_im = $bitstoreal(ta_im);
            rb_re = $bitstoreal(tb_re);
            rb_im = $bitstoreal(tb_im);
            rw_re = $bitstoreal(tw_re);
            rw_im = $bitstoreal(tw_im);

            exp_y0_re_r = ra_re + (rb_re * rw_re - rb_im * rw_im);
            exp_y0_im_r = ra_im + (rb_re * rw_im + rb_im * rw_re);
            exp_y1_re_r = ra_re - (rb_re * rw_re - rb_im * rw_im);
            exp_y1_im_r = ra_im - (rb_re * rw_im + rb_im * rw_re);

            exp_y0_re = $realtobits(exp_y0_re_r);
            exp_y0_im = $realtobits(exp_y0_im_r);
            exp_y1_re = $realtobits(exp_y1_re_r);
            exp_y1_im = $realtobits(exp_y1_im_r);

            @(posedge clk);
            while (!in_ready) begin
                @(posedge clk);
            end

            a_re     <= ta_re;
            a_im     <= ta_im;
            b_re     <= tb_re;
            b_im     <= tb_im;
            w_re     <= tw_re;
            w_im     <= tw_im;
            in_valid <= 1'b1;
            @(posedge clk);
            in_valid <= 1'b0;

            cycle_count = 0;
            while (!out_valid) begin
                @(posedge clk);
                cycle_count = cycle_count + 1;
                if (cycle_count > 80) begin
                    $display("TB_FAIL butterfly timeout");
                    error_count = error_count + 1;
                    disable run_bfly_case;
                end
            end

            #1;
            check_equal64("bfly_y0_re", y0_re, exp_y0_re);
            check_equal64("bfly_y0_im", y0_im, exp_y0_im);
            check_equal64("bfly_y1_re", y1_re, exp_y1_re);
            check_equal64("bfly_y1_im", y1_im, exp_y1_im);

            if (status_invalid || status_overflow || status_underflow) begin
                $display("TB_FAIL butterfly unexpected status inv=%0d ovf=%0d udf=%0d",
                    status_invalid, status_overflow, status_underflow);
                error_count = error_count + 1;
            end

            @(posedge clk);
        end
    endtask

    initial begin
        clk         = 1'b0;
        rst_n       = 1'b0;
        in_valid    = 1'b0;
        out_ready   = 1'b1;
        a_re        = 64'd0;
        a_im        = 64'd0;
        b_re        = 64'd0;
        b_im        = 64'd0;
        w_re        = 64'd0;
        w_im        = 64'd0;
        fpu_req_valid = 1'b0;
        fpu_req_op    = 4'd0;
        fpu_req_a     = 64'd0;
        fpu_req_b     = 64'd0;
        fpu_req_c     = 64'd0;
        error_count = 0;

        repeat (3) @(posedge clk);
        rst_n = 1'b1;
        repeat (2) @(posedge clk);

        run_fpu_case("fpu_add",    OP_FADD,   64'h3ff8_0000_0000_0000, 64'h4004_0000_0000_0000, 64'd0, 64'h4010_0000_0000_0000);
        run_fpu_case("fpu_mul",    OP_FMUL,   64'h4004_0000_0000_0000, 64'h3fe0_0000_0000_0000, 64'd0, 64'h3ff4_0000_0000_0000);
        run_fpu_case("fpu_fmadd",  OP_FMADD,  64'h4000_0000_0000_0000, 64'h3fe0_0000_0000_0000, 64'h3ff8_0000_0000_0000, 64'h4004_0000_0000_0000);
        run_fpu_case("fpu_fnmsub", OP_FNMSUB, 64'h3ff0_0000_0000_0000, 64'h4000_0000_0000_0000, 64'h4008_0000_0000_0000, 64'hc014_0000_0000_0000);
        run_fpu_case("fpu_fnmadd", OP_FNMADD, 64'h3ff0_0000_0000_0000, 64'h4000_0000_0000_0000, 64'h4008_0000_0000_0000, 64'h3ff0_0000_0000_0000);

        run_bfly_case(
            64'h3ff8_0000_0000_0000,
            64'hbfe0_0000_0000_0000,
            64'h4000_0000_0000_0000,
            64'h3fd0_0000_0000_0000,
            64'h3fe0_0000_0000_0000,
            64'hbff0_0000_0000_0000
        );

        run_bfly_case(
            64'hc000_0000_0000_0000,
            64'h3ff0_0000_0000_0000,
            64'h3fe0_0000_0000_0000,
            64'hbfe0_0000_0000_0000,
            64'h3ff0_0000_0000_0000,
            64'h0000_0000_0000_0000
        );

        if (error_count == 0) begin
            $display("TB_PASS falcon_f64_complex_bfly");
        end else begin
            $display("TB_FAIL falcon_f64_complex_bfly error_count=%0d", error_count);
        end

        $finish;
    end

endmodule
