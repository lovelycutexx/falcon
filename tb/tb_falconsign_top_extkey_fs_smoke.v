`timescale 1ns/1ps

module tb_falconsign_top_extkey_fs_smoke;
    localparam [15:0] REG_CR     = 16'h0000;
    localparam [15:0] REG_SR     = 16'h0004;
    localparam [15:0] REG_CFG    = 16'h0008;
    localparam [15:0] REG_MEM_HI = 16'h000C;

    localparam integer LAYOUT_TREE_BASE = 1024;
    localparam integer TREE_LEAF_OFS    = 255;
    localparam integer FALCON_N         = 512;

    reg         clk;
    reg         rst_n;
    reg         bus_cs;
    reg         bus_wr;
    reg  [15:0] bus_addr;
    reg  [31:0] bus_wdata;
    wire [31:0] bus_rdata;
    wire        bus_ready;
    wire        bus_irq;
    wire        busy;
    wire        done;
    wire        fail;
    wire [7:0]  status;

    falconsign_top #(.ADDR_W(13), .LEVEL_W(4), .INDEX_W(10)) dut (
        .clk(clk),
        .rst_n(rst_n),
        .bus_cs(bus_cs),
        .bus_wr(bus_wr),
        .bus_addr(bus_addr),
        .bus_wdata(bus_wdata),
        .bus_rdata(bus_rdata),
        .bus_ready(bus_ready),
        .bus_irq(bus_irq),
        .busy(busy),
        .done(done),
        .fail(fail),
        .status(status)
    );

    initial begin
        clk = 1'b0;
        forever #5 clk = ~clk;
    end

    task bus_write;
        input [15:0] addr;
        input [31:0] data;
        begin
            @(posedge clk);
            bus_cs    <= 1'b1;
            bus_wr    <= 1'b1;
            bus_addr  <= addr;
            bus_wdata <= data;
            @(posedge clk);
            bus_cs    <= 1'b0;
            bus_wr    <= 1'b0;
            bus_addr  <= 16'd0;
            bus_wdata <= 32'd0;
            wait (bus_ready);
            @(posedge clk);
        end
    endtask

    task bus_read;
        input  [15:0] addr;
        output [31:0] data;
        begin
            @(posedge clk);
            bus_cs    <= 1'b1;
            bus_wr    <= 1'b0;
            bus_addr  <= addr;
            bus_wdata <= 32'd0;
            @(posedge clk);
            bus_cs    <= 1'b0;
            bus_addr  <= 16'd0;
            wait (bus_ready);
            data = bus_rdata;
            @(posedge clk);
        end
    endtask

    task write_f64_pair_ones;
        input integer word_addr;
        integer byte_addr;
        begin
            byte_addr = word_addr * 32;
            bus_write(REG_MEM_HI, byte_addr[17:16]);
            bus_write(byte_addr[15:0] + 16'd0,  32'h00000000);
            bus_write(byte_addr[15:0] + 16'd4,  32'h3FF00000);
            bus_write(byte_addr[15:0] + 16'd8,  32'h00000000);
            bus_write(byte_addr[15:0] + 16'd12, 32'h3FF00000);
        end
    endtask

    reg [31:0] sr;
    integer i;
    integer timeout;
    integer saw_fs;
    integer saw_vd;
    integer sample_cmds;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            saw_fs = 0;
            saw_vd = 0;
            sample_cmds = 0;
        end else begin
            if (dut.st == 4'd4) saw_fs = 1;
            if (dut.st == 4'd5) saw_vd = 1;
            if (dut.fe_sz_cmd_valid && dut.fe_sz_cmd_ready) sample_cmds = sample_cmds + 1;
        end
    end

    initial begin
        rst_n = 1'b0;
        bus_cs = 1'b0;
        bus_wr = 1'b0;
        bus_addr = 16'd0;
        bus_wdata = 32'd0;
        timeout = 0;

        repeat (8) @(posedge clk);
        rst_n = 1'b1;
        repeat (4) @(posedge clk);

        // Software-provided tree material for leaf sampling. Internal l10
        // nodes are left as zero for this bring-up smoke.
        for (i = 0; i < FALCON_N; i = i + 1) begin
            write_f64_pair_ones(LAYOUT_TREE_BASE + TREE_LEAF_OFS + i);
        end

        // Force RC accept, but do not bypass ffSampling.
        bus_write(REG_CFG, 32'h00000002);
        bus_write(REG_MEM_HI, 32'd0);
        bus_write(REG_CR, 32'h00000001);

        while (!done && !fail && timeout < 12000000) begin
            @(posedge clk);
            timeout = timeout + 1;
        end

        bus_read(REG_SR, sr);
        if (done && !fail && bus_irq && saw_fs && saw_vd && (sample_cmds >= FALCON_N)) begin
            $display("TB_PASS falconsign_top_extkey_fs_smoke cycles=%0d sample_cmds=%0d sr=%08x",
                     timeout, sample_cmds, sr);
        end else begin
            $display("TB_FAIL falconsign_top_extkey_fs_smoke done=%0d fail=%0d irq=%0d status=%02x st=%0d sn=%0d saw_fs=%0d saw_vd=%0d sample_cmds=%0d sr=%08x",
                     done, fail, bus_irq, status, dut.st, dut.sn, saw_fs, saw_vd, sample_cmds, sr);
            $finish;
        end
        $finish;
    end
endmodule
