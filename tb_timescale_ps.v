`timescale 1ps/1ps
module tb_timescale_ps;
    reg clk;
    initial begin
        clk = 0;
        $display("[%d] clk=%b", $time, clk);
        #5000;
        clk = 1;
        $display("[%d] clk=%b", $time, clk);
        #5000;
        clk = 0;
        $display("[%d] clk=%b", $time, clk);
        $finish;
    end
endmodule
