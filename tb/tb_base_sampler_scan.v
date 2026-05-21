`timescale 1ns/1ps
// Verify BaseSampler CDT scan finds correct z for various u_thresh
module tb_base_sampler_scan;
    reg  [3:0] addr;
    wire [15:0] data;
    falconsign_bs_cdt_rom uu(.addr(addr),.data(data));
    integer u, z;
    initial begin
        $display("CDT ROM verification:");
        for(z=0;z<=8;z++) begin
            addr=z; #1; $display("  CDT[%0d] = %0d", z, data);
        end
        $display("");
        $display("Scan: find z with cdt[z] >= u for various u:");
        for(u=0;u<65536;u+=5000) begin
            for(z=0;z<=15;z++) begin
                addr=z; #1;
                if(data>=u) begin
                    $display("  u=%5d -> z=%2d  (cdt[%2d]=%5d)", u, z, z, data);
                    break;
                end
            end
        end
        $finish;
    end
endmodule
