// BaseSampler CDT ROM (combinational)  sigma_max=1.8205
module falconsign_bs_cdt_rom #(parameter ADDR_W=4) (
    input  wire [ADDR_W-1:0] addr,
    output reg  [15:0] data
);
    always @(*) begin
        case (addr)
            4'd0: data = 16'd14361;
            4'd1: data = 16'd39062;
            4'd2: data = 16'd54771;
            4'd3: data = 16'd62160;
            4'd4: data = 16'd64730;
            4'd5: data = 16'd65391;
            4'd6: data = 16'd65516;
            4'd7: data = 16'd65534;
            4'd8: data = 16'd65535;
            default: data = 16'd65535;
        endcase
    end
endmodule
