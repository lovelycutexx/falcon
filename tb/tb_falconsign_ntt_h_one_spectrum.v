`timescale 1ns/1ps

module tb_falconsign_ntt_h_one_spectrum;
    localparam ADDR_W  = 11;
    localparam N_WORDS = 32;
    localparam H_BASE  = 0;
    localparam S2_BASE = 32;
    localparam C_BASE  = 64;
    localparam DST_BASE= 96;

    reg clk;
    reg rst_n;
    reg start;
    wire start_ready;
    wire done;
    wire fail;
    wire [7:0] status;

    wire              mem_rd_en;
    wire [ADDR_W-1:0] mem_rd_addr;
    reg  [255:0]      mem_rd_data;
    wire              mem_wr_en;
    wire [ADDR_W-1:0] mem_wr_addr;
    wire [255:0]      mem_wr_data;
    wire [8:0]        ntt_tw_rom_addr;
    wire [13:0]       ntt_tw_rom_data;
    wire [9:0]        ntt_psi_rom_addr;
    wire [13:0]       ntt_psi_rom_data;

    reg [255:0] mem [0:127];
    integer i;
    integer lane;
    integer errors;
    reg inspected;
    reg [13:0] got;

    falconsign_ntt_twiddle_rom #(.ADDR_W(9)) u_tw (
        .clk(clk), .addr(ntt_tw_rom_addr), .data(ntt_tw_rom_data)
    );

    falconsign_ntt_psi_rom #(.ADDR_W(10)) u_psi (
        .clk(clk), .addr(ntt_psi_rom_addr), .data(ntt_psi_rom_data)
    );

    falconsign_ntt_exu #(.ADDR_W(ADDR_W)) dut (
        .clk(clk), .rst_n(rst_n),
        .start(start), .start_ready(start_ready),
        .done(done), .fail(fail), .status(status),
        .cfg_h_base(H_BASE[ADDR_W-1:0]),
        .cfg_s2_base(S2_BASE[ADDR_W-1:0]),
        .cfg_c_base(C_BASE[ADDR_W-1:0]),
        .cfg_dst_base(DST_BASE[ADDR_W-1:0]),
        .mem_rd_en(mem_rd_en), .mem_rd_addr(mem_rd_addr), .mem_rd_data(mem_rd_data),
        .mem_wr_en(mem_wr_en), .mem_wr_addr(mem_wr_addr), .mem_wr_data(mem_wr_data),
        .twiddle_rom_addr(ntt_tw_rom_addr), .twiddle_rom_data(ntt_tw_rom_data),
        .psi_rom_addr(ntt_psi_rom_addr), .psi_rom_data(ntt_psi_rom_data)
    );

    always #5 clk = ~clk;

    always @(posedge clk) begin
        if (mem_rd_en)
            mem_rd_data <= mem[mem_rd_addr];
        if (mem_wr_en)
            mem[mem_wr_addr] <= mem_wr_data;
    end

    initial begin
        clk = 1'b0;
        rst_n = 1'b0;
        start = 1'b0;
        mem_rd_data = 256'd0;
        errors = 0;
        inspected = 1'b0;

        for (i = 0; i < 128; i = i + 1)
            mem[i] = 256'd0;
        mem[H_BASE][15:0] = 16'd1;

        repeat (5) @(posedge clk);
        rst_n = 1'b1;
        repeat (5) @(posedge clk);

        start <= 1'b1;
        @(posedge clk);
        start <= 1'b0;
    end

    always @(posedge clk) begin
        if (!inspected && dut.op_state == 4'd1 && dut.op_target == 2'd1) begin
            inspected = 1'b1;
            for (i = 0; i < N_WORDS; i = i + 1) begin
                for (lane = 0; lane < 16; lane = lane + 1) begin
                    got = mem[H_BASE + i][lane*16 +: 14];
                    if (got !== 14'd1) begin
                        errors = errors + 1;
                        if (errors <= 16)
                            $display("H=1 spectrum mismatch coeff[%0d]: got=%0d expected=1",
                                     i*16 + lane, got);
                    end
                end
            end
            if (errors == 0)
                $display("H=1 SPECTRUM PASSED: forward NTT produced all ones");
            else
                $display("H=1 SPECTRUM FAILED: %0d mismatches", errors);
            #20;
            $finish;
        end
        if (done || fail) begin
            $display("H=1 SPECTRUM ended before inspection: done=%0d fail=%0d status=%02x",
                     done, fail, status);
            $finish;
        end
    end

    initial begin
        #20000000;
        $display("H=1 SPECTRUM TIMEOUT op=%0d target=%0d", dut.op_state, dut.op_target);
        $finish;
    end
endmodule
