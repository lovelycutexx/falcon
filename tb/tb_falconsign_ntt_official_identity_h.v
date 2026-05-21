`timescale 1ns/1ps

// Official-vector diagnostic for non-zero NTT multiplication.
// With h(x)=1 and s2=c, the EXU should return s1=c-c=0.
module tb_falconsign_ntt_official_identity_h;
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

    wire [8:0]  ntt_tw_rom_addr;
    wire [13:0] ntt_tw_rom_data;
    wire [9:0]  ntt_psi_rom_addr;
    wire [13:0] ntt_psi_rom_data;

    reg [255:0] mem [0:127];
    reg [15:0]  coeff [0:511];
    integer i;
    integer lane;
    integer errors;
    reg [13:0] got;

    falconsign_ntt_twiddle_rom #(.ADDR_W(9)) u_tw (
        .clk(clk),
        .addr(ntt_tw_rom_addr),
        .data(ntt_tw_rom_data)
    );

    falconsign_ntt_psi_rom #(.ADDR_W(10)) u_psi (
        .clk(clk),
        .addr(ntt_psi_rom_addr),
        .data(ntt_psi_rom_data)
    );

    falconsign_ntt_exu #(.ADDR_W(ADDR_W)) dut (
        .clk(clk),
        .rst_n(rst_n),
        .start(start),
        .start_ready(start_ready),
        .done(done),
        .fail(fail),
        .status(status),
        .cfg_h_base(H_BASE[ADDR_W-1:0]),
        .cfg_s2_base(S2_BASE[ADDR_W-1:0]),
        .cfg_c_base(C_BASE[ADDR_W-1:0]),
        .cfg_dst_base(DST_BASE[ADDR_W-1:0]),
        .mem_rd_en(mem_rd_en),
        .mem_rd_addr(mem_rd_addr),
        .mem_rd_data(mem_rd_data),
        .mem_wr_en(mem_wr_en),
        .mem_wr_addr(mem_wr_addr),
        .mem_wr_data(mem_wr_data),
        .twiddle_rom_addr(ntt_tw_rom_addr),
        .twiddle_rom_data(ntt_tw_rom_data),
        .psi_rom_addr(ntt_psi_rom_addr),
        .psi_rom_data(ntt_psi_rom_data)
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

        $readmemh("SRC/tb/falcon512_kat0_htp_expected.hex", coeff);

        for (i = 0; i < 128; i = i + 1)
            mem[i] = 256'd0;

        mem[H_BASE][15:0] = 16'd1;

        for (i = 0; i < N_WORDS; i = i + 1) begin
            mem[C_BASE + i] = 256'd0;
            mem[S2_BASE + i] = 256'd0;
            mem[DST_BASE + i] = {16{16'hDEAD}};
            for (lane = 0; lane < 16; lane = lane + 1) begin
                mem[C_BASE + i][lane*16 +: 16] = coeff[i*16 + lane];
                mem[S2_BASE + i][lane*16 +: 16] = coeff[i*16 + lane];
            end
        end

        repeat (5) @(posedge clk);
        rst_n = 1'b1;
        repeat (5) @(posedge clk);

        if (!start_ready) begin
            $display("NTT OFFICIAL ID-H FAILED: start_ready=0");
            $finish;
        end

        start <= 1'b1;
        @(posedge clk);
        start <= 1'b0;

        wait(done || fail);

        if (fail) begin
            $display("NTT OFFICIAL ID-H FAILED: status=%02x", status);
            $finish;
        end

        for (i = 0; i < N_WORDS; i = i + 1) begin
            for (lane = 0; lane < 16; lane = lane + 1) begin
                got = mem[DST_BASE + i][lane*16 +: 14];
                if (got !== 14'd0) begin
                    errors = errors + 1;
                    if (errors <= 12)
                        $display("NTT OFFICIAL ID-H mismatch coeff[%0d]: got=%0d expected=0",
                                 i*16 + lane, got);
                end
            end
        end

        if (errors == 0)
            $display("NTT OFFICIAL ID-H PASSED: h=1,s2=c gives zero s1 for all 512 official c coefficients");
        else
            $display("NTT OFFICIAL ID-H FAILED: %0d mismatches", errors);

        #20;
        $finish;
    end

    initial begin
        #60000000;
        $display("NTT OFFICIAL ID-H TIMEOUT: op=%0d ls=%0d bs=%0d bt=%0d",
                 dut.op_state, dut.ls, dut.bs, dut.bt);
        $finish;
    end
endmodule
