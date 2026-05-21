`timescale 1ns/1ps

module tb_falcon_fft_complex_bfly;

    localparam COEFF_W   = 32;
    localparam PROD_W    = 64;
    localparam FRAC_BITS = 16;

    reg signed [COEFF_W-1:0] a_re;
    reg signed [COEFF_W-1:0] a_im;
    reg signed [COEFF_W-1:0] b_re;
    reg signed [COEFF_W-1:0] b_im;
    reg signed [COEFF_W-1:0] w_re;
    reg signed [COEFF_W-1:0] w_im;

    wire signed [COEFF_W-1:0] y0_re;
    wire signed [COEFF_W-1:0] y0_im;
    wire signed [COEFF_W-1:0] y1_re;
    wire signed [COEFF_W-1:0] y1_im;

    integer error_count;

    falcon_fft_complex_bfly #
    (
        .COEFF_W   (COEFF_W),
        .PROD_W    (PROD_W),
        .FRAC_BITS (FRAC_BITS)
    )
    dut (
        .a_re  (a_re),
        .a_im  (a_im),
        .b_re  (b_re),
        .b_im  (b_im),
        .w_re  (w_re),
        .w_im  (w_im),
        .y0_re (y0_re),
        .y0_im (y0_im),
        .y1_re (y1_re),
        .y1_im (y1_im)
    );

    task expect_value;
        input signed [COEFF_W-1:0] exp_y0_re;
        input signed [COEFF_W-1:0] exp_y0_im;
        input signed [COEFF_W-1:0] exp_y1_re;
        input signed [COEFF_W-1:0] exp_y1_im;
        begin
            #1;
            if ((y0_re !== exp_y0_re) ||
                (y0_im !== exp_y0_im) ||
                (y1_re !== exp_y1_re) ||
                (y1_im !== exp_y1_im)) begin
                $display("TB_FAIL butterfly mismatch exp=(%0d,%0d,%0d,%0d) got=(%0d,%0d,%0d,%0d)",
                    exp_y0_re, exp_y0_im, exp_y1_re, exp_y1_im,
                    y0_re, y0_im, y1_re, y1_im);
                error_count = error_count + 1;
            end
        end
    endtask

    initial begin
        error_count = 0;

        a_re = 32'sd10 <<< FRAC_BITS;
        a_im = 32'sd2  <<< FRAC_BITS;
        b_re = 32'sd4  <<< FRAC_BITS;
        b_im = 32'sd1  <<< FRAC_BITS;
        w_re = 32'sd1  <<< FRAC_BITS;
        w_im = 32'sd0;
        expect_value(32'sd14 <<< FRAC_BITS,
                     32'sd3  <<< FRAC_BITS,
                     32'sd6  <<< FRAC_BITS,
                     32'sd1  <<< FRAC_BITS);

        a_re = 32'sd8  <<< FRAC_BITS;
        a_im = 32'sd0;
        b_re = 32'sd2  <<< FRAC_BITS;
        b_im = 32'sd0;
        w_re = 32'sd0;
        w_im = 32'sd1  <<< FRAC_BITS;
        expect_value(32'sd8  <<< FRAC_BITS,
                     32'sd2  <<< FRAC_BITS,
                     32'sd8  <<< FRAC_BITS,
                     -32'sd2 <<< FRAC_BITS);

        if (error_count == 0) begin
            $display("TB_PASS falcon_fft_complex_bfly");
        end else begin
            $display("TB_FAIL falcon_fft_complex_bfly error_count=%0d", error_count);
        end
        $finish;
    end

endmodule
