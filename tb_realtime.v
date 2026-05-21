`timescale 1ns/1ps
module tb_realtime;
    initial begin
        $display("[%0t] $time=%-0t  $realtime=%f", $time, $time, $realtime);
        #1;
        $display("[%0t] $time=%-0t  $realtime=%f", $time, $time, $realtime);
        #10;
        $display("[%0t] $time=%-0t  $realtime=%f", $time, $time, $realtime);
        $finish;
    end
endmodule
