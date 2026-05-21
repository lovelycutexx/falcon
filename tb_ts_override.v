module tb_ts_override;
    `timescale 1ns/1ps
    reg clk;
    initial begin
        clk = 0;
        $display("[%d] after 0ns", $time);
        #1;
        $display("[%d] after #1", $time);
        #10;
        $display("[%d] after #10", $time);
        $finish;
    end
endmodule
