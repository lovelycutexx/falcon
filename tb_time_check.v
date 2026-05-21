`timescale 1ns/1ps
module tb_time_check;
    reg clk, rst_n;
    reg [31:0] cycle;
    initial begin
        clk = 0;
        rst_n = 0;
        #30 rst_n = 1;
    end
    always #5 clk = ~clk;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cycle <= 0;
        end else begin
            cycle <= cycle + 1;
            if (cycle < 20)
                $display("[%0t] cycle=%0d rst_n=%0b", $time, cycle, rst_n);
        end
    end
    initial #200 $finish;
endmodule
