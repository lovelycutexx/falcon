`timescale 1ns/1ps

module tb_falconsign_top_extkey_smoke;
    localparam [15:0] REG_CR     = 16'h0000;
    localparam [15:0] REG_SR     = 16'h0004;
    localparam [15:0] REG_CFG    = 16'h0008;
    localparam [15:0] REG_MEM_HI = 16'h000C;

    localparam integer LAYOUT_B11_BASE = 4608;
    localparam integer B11_BYTE_ADDR   = LAYOUT_B11_BASE * 32;

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

    reg [31:0] rd0;
    reg [31:0] rd1;
    reg [31:0] sr;
    integer timeout;

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

        // Prove host-visible high-memory key/material input reaches the B_hat
        // region. Write one FP64 1.0 value into B11[0]. Real key loading will
        // stream many such words through the same MEM_HI window.
        bus_write(REG_MEM_HI, B11_BYTE_ADDR[17:16]);
        bus_write(B11_BYTE_ADDR[15:0] + 16'd0, 32'h00000000);
        bus_write(B11_BYTE_ADDR[15:0] + 16'd4, 32'h3FF00000);
        bus_read(B11_BYTE_ADDR[15:0] + 16'd0, rd0);
        bus_read(B11_BYTE_ADDR[15:0] + 16'd4, rd1);
        if (rd0 !== 32'h00000000 || rd1 !== 32'h3FF00000) begin
            $display("TB_FAIL extkey memory readback got %08x_%08x", rd1, rd0);
            $finish;
        end

        // Bring-up mode: externally supplied key/material, bypass incomplete
        // ffSampling two-component production, and force RC accept so the
        // top-level sign control flow can close.
        bus_write(REG_CFG, 32'h00000003);
        bus_write(REG_MEM_HI, 32'd0);
        bus_write(REG_CR, 32'h00000001);

        while (!done && !fail && timeout < 12000000) begin
            @(posedge clk);
            timeout = timeout + 1;
        end

        bus_read(REG_SR, sr);
        if (done && !fail && bus_irq) begin
            $display("TB_PASS falconsign_top_extkey_smoke cycles=%0d sr=%08x", timeout, sr);
        end else begin
            $display("TB_FAIL falconsign_top_extkey_smoke done=%0d fail=%0d irq=%0d status=%02x st=%0d sn=%0d sr=%08x",
                     done, fail, bus_irq, status, dut.st, dut.sn, sr);
            $finish;
        end
        $finish;
    end
endmodule
