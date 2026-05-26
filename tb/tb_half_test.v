`timescale 1ns/1ps
module tb_half_test;
    function [63:0] f64_half;
        input [63:0] v;
        begin
            f64_half = (v[62:52] == 11'd0) ? v : {v[63], v[62:52] - 1'b1, v[51:0]};
        end
    endfunction
    
    initial begin
        $display("half(0.0)  = %016x", f64_half($realtobits(0.0)));
        $display("half(1.0)  = %016x = %f", f64_half($realtobits(1.0)), $bitstoreal(f64_half($realtobits(1.0))));
        $display("half(1024) = %016x = %f", f64_half($realtobits(1024.0)), $bitstoreal(f64_half($realtobits(1024.0))));
        $display("half(-1.0) = %016x = %f", f64_half($realtobits(-1.0)), $bitstoreal(f64_half($realtobits(-1.0))));
        $display("half(2.0)  = %016x = %f", f64_half($realtobits(2.0)), $bitstoreal(f64_half($realtobits(2.0))));
    end
endmodule
