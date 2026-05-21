`timescale 1ns/1ps
module tb_raw_time;
    initial begin
        $display("[%0t] time=0", $time);
        #1;
        $display("[%0t] after #1", $time);
        #10;
        $display("[%0t] after #10", $time);
        #100;
        $display("[%0t] after #100", $time);
        $finish;
    end
endmodule
