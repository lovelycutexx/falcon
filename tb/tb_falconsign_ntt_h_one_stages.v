`timescale 1ns/1ps

module tb_falconsign_ntt_h_one_stages;
    localparam ADDR_W=11, H_BASE=0, S2_BASE=32, C_BASE=64, DST_BASE=96;
    reg clk, rst_n, start;
    wire start_ready, done, fail;
    wire [7:0] status;
    wire mem_rd_en, mem_wr_en;
    wire [ADDR_W-1:0] mem_rd_addr, mem_wr_addr;
    reg [255:0] mem_rd_data;
    wire [255:0] mem_wr_data;
    wire [8:0] tw_addr;
    wire [13:0] tw_data;
    wire [9:0] psi_addr;
    wire [13:0] psi_data;
    reg [255:0] mem [0:127];
    integer i, lane, nz, bad;
    reg seen_pre, seen_bitrev;

    falconsign_ntt_twiddle_rom #(.ADDR_W(9)) u_tw(.clk(clk), .addr(tw_addr), .data(tw_data));
    falconsign_ntt_psi_rom #(.ADDR_W(10)) u_psi(.clk(clk), .addr(psi_addr), .data(psi_data));
    falconsign_ntt_exu #(.ADDR_W(ADDR_W)) dut(
        .clk(clk), .rst_n(rst_n), .start(start), .start_ready(start_ready),
        .done(done), .fail(fail), .status(status),
        .cfg_h_base(H_BASE[ADDR_W-1:0]), .cfg_s2_base(S2_BASE[ADDR_W-1:0]),
        .cfg_c_base(C_BASE[ADDR_W-1:0]), .cfg_dst_base(DST_BASE[ADDR_W-1:0]),
        .mem_rd_en(mem_rd_en), .mem_rd_addr(mem_rd_addr), .mem_rd_data(mem_rd_data),
        .mem_wr_en(mem_wr_en), .mem_wr_addr(mem_wr_addr), .mem_wr_data(mem_wr_data),
        .twiddle_rom_addr(tw_addr), .twiddle_rom_data(tw_data),
        .psi_rom_addr(psi_addr), .psi_rom_data(psi_data));

    always #5 clk = ~clk;
    always @(posedge clk) begin
        if (mem_rd_en) mem_rd_data <= mem[mem_rd_addr];
        if (mem_wr_en) mem[mem_wr_addr] <= mem_wr_data;
    end

    task check_delta;
        input [127:0] name;
        begin
            nz = 0; bad = 0;
            for (i = 0; i < 32; i = i + 1) begin
                for (lane = 0; lane < 16; lane = lane + 1) begin
                    if (mem[H_BASE+i][lane*16 +: 14] != 0) nz = nz + 1;
                    if ((i == 0 && lane == 0 && mem[H_BASE+i][lane*16 +: 14] != 1) ||
                        (!(i == 0 && lane == 0) && mem[H_BASE+i][lane*16 +: 14] != 0)) begin
                        bad = bad + 1;
                        if (bad <= 8)
                            $display("%s delta bad coeff[%0d]=%0d", name, i*16+lane,
                                     mem[H_BASE+i][lane*16 +: 14]);
                    end
                end
            end
            if (bad == 0) $display("%s delta OK nz=%0d", name, nz);
            else $display("%s delta FAILED bad=%0d nz=%0d", name, bad, nz);
        end
    endtask

    initial begin
        clk=0; rst_n=0; start=0; mem_rd_data=0; seen_pre=0; seen_bitrev=0;
        for (i=0; i<128; i=i+1) mem[i]=0;
        mem[0][15:0]=16'd1;
        repeat(5) @(posedge clk); rst_n=1; repeat(5) @(posedge clk);
        start<=1; @(posedge clk); start<=0;
    end

    always @(posedge clk) begin
        if (!seen_pre && dut.op_state == 4'd6 && dut.op_target == 2'd0) begin
            seen_pre = 1;
            check_delta("after PRE");
        end
        if (!seen_bitrev && dut.op_state == 4'd2 && dut.op_target == 2'd0) begin
            seen_bitrev = 1;
            check_delta("after BITREV");
            #20; $finish;
        end
        if (done || fail) begin
            $display("ended early done=%0d fail=%0d status=%02x", done, fail, status);
            $finish;
        end
    end

    initial begin
        #20000000;
        $display("stage diag timeout op=%0d target=%0d", dut.op_state, dut.op_target);
        $finish;
    end
endmodule
