`timescale 1ns/1ps

module tb_pqc_common_modules;

  reg clk;
  reg rst_n;
  reg start;
  reg sub;
  reg [7:0] a_coeff;
  reg [7:0] b_coeff;
  reg [31:0] a_poly;
  reg [31:0] b_poly;
  reg [47:0] vote_coeffs;
  reg [31:0] e8_coords;
  reg [31:0] norm_coeffs;

  wire [7:0] mod_add_sub_y;
  wire [7:0] mod_mul_y;
  wire       poly_busy;
  wire       poly_done;
  wire [31:0] poly_result;
  wire [1:0] vote_bits;
  wire       e8_accept;
  wire [31:0] e8_norm_sq;
  wire       norm_accept;
  wire [31:0] norm_sq;

  integer error_count;

  pqc_mod_add_sub #(
    .COEFF_WIDTH(8),
    .MODULUS(17)
  ) u_mod_add_sub (
    .a(a_coeff),
    .b(b_coeff),
    .sub(sub),
    .y(mod_add_sub_y)
  );

  pqc_mod_mul_reduce #(
    .COEFF_WIDTH(8),
    .MODULUS(17)
  ) u_mod_mul_reduce (
    .a(a_coeff),
    .b(b_coeff),
    .y(mod_mul_y)
  );

  pqc_poly_mul_schoolbook #(
    .N(4),
    .COEFF_WIDTH(8),
    .MODULUS(17),
    .RING_MODE(0)
  ) u_poly_mul (
    .clk(clk),
    .rst_n(rst_n),
    .start(start),
    .a_poly(a_poly),
    .b_poly(b_poly),
    .busy(poly_busy),
    .done(poly_done),
    .result_poly(poly_result)
  );

  pqc_vector_vote_decode #(
    .BIT_COUNT(2),
    .REPEAT_COUNT(3),
    .COEFF_WIDTH(8),
    .THRESHOLD(5),
    .MAJORITY_COUNT(2)
  ) u_vector_vote (
    .coeffs(vote_coeffs),
    .msg_bits(vote_bits)
  );

  pqc_e8_radius_check #(
    .COEFF_WIDTH(4),
    .ACC_WIDTH(32),
    .RADIUS_SQ(4)
  ) u_e8_radius_check (
    .coords(e8_coords),
    .accept(e8_accept),
    .norm_sq(e8_norm_sq)
  );

  pqc_norm_check #(
    .COEFF_COUNT(4),
    .COEFF_WIDTH(8),
    .ACC_WIDTH(32),
    .BOUND_SQ(30)
  ) u_norm_check (
    .coeffs(norm_coeffs),
    .accept(norm_accept),
    .norm_sq(norm_sq)
  );

  always #5 clk = ~clk;

  task check8;
    input [255:0] name;
    input [7:0]   got;
    input [7:0]   exp;
    begin
      if (got !== exp) begin
        $display("FAIL %0s: got=%0d expected=%0d", name, got, exp);
        error_count = error_count + 1;
      end else begin
        $display("PASS %0s: %0d", name, got);
      end
    end
  endtask

  task check1;
    input [255:0] name;
    input         got;
    input         exp;
    begin
      if (got !== exp) begin
        $display("FAIL %0s: got=%0b expected=%0b", name, got, exp);
        error_count = error_count + 1;
      end else begin
        $display("PASS %0s: %0b", name, got);
      end
    end
  endtask

  initial begin
    clk         = 1'b0;
    rst_n       = 1'b0;
    start       = 1'b0;
    sub         = 1'b0;
    a_coeff     = 8'd0;
    b_coeff     = 8'd0;
    a_poly      = 32'd0;
    b_poly      = 32'd0;
    vote_coeffs = 48'd0;
    e8_coords   = 32'd0;
    norm_coeffs = 32'd0;
    error_count = 0;

    $dumpfile("tb_pqc_common_modules.vcd");
    $dumpvars(0, tb_pqc_common_modules);

    repeat (3) @(posedge clk);
    rst_n = 1'b1;
    @(posedge clk);

    a_coeff = 8'd15;
    b_coeff = 8'd5;
    sub     = 1'b0;
    #1;
    check8("mod_add", mod_add_sub_y, 8'd3);

    a_coeff = 8'd5;
    b_coeff = 8'd15;
    sub     = 1'b1;
    #1;
    check8("mod_sub", mod_add_sub_y, 8'd7);

    a_coeff = 8'd4;
    b_coeff = 8'd5;
    #1;
    check8("mod_mul", mod_mul_y, 8'd3);

    vote_coeffs = {
      8'd9, 8'd3, 8'd2,
      8'd9, 8'd8, 8'd1
    };
    #1;
    check8("vector_vote", {6'd0, vote_bits}, 8'b00000001);

    e8_coords = {4'd0, 4'd0, 4'd0, 4'd0, 4'd1, 4'd1, 4'd1, 4'd1};
    #1;
    check1("e8_radius_accept", e8_accept, 1'b1);

    norm_coeffs = {8'd4, 8'hFD, 8'd2, 8'd1};
    #1;
    check1("norm_accept", norm_accept, 1'b1);

    a_poly = {8'd4, 8'd3, 8'd2, 8'd1};
    b_poly = {8'd8, 8'd7, 8'd6, 8'd5};
    @(posedge clk);
    start = 1'b1;
    @(posedge clk);
    start = 1'b0;

    wait (poly_done == 1'b1);
    #1;
    check8("poly_c0", poly_result[0*8 +: 8], 8'd12);
    check8("poly_c1", poly_result[1*8 +: 8], 8'd15);
    check8("poly_c2", poly_result[2*8 +: 8], 8'd2);
    check8("poly_c3", poly_result[3*8 +: 8], 8'd9);

    if (error_count == 0) begin
      $display("TB_PASS pqc_common_modules");
    end else begin
      $display("TB_FAIL pqc_common_modules errors=%0d", error_count);
    end

    $finish;
  end

  initial begin
    #5000;
    $display("TB_TIMEOUT pqc_common_modules");
    $finish;
  end

endmodule
