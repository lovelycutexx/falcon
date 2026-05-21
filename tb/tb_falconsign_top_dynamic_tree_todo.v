`timescale 1ns/1ps

module tb_falconsign_top_dynamic_tree_todo;
    localparam [15:0] REG_CR  = 16'h0000;
    localparam [15:0] REG_SR  = 16'h0004;
    localparam [15:0] REG_CFG = 16'h0008;

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

        // cfg[1]=force_accept, cfg[2]=start_at_fs, cfg[3]=dynamic_tree.
        // Dynamic tree currently emits OP_DYNAMIC_LDL, which is intentionally
        // reported as unsupported by the ffSampling EXU until LDL hardware is
        // implemented.
        bus_write(REG_CFG, 32'h0000000E);
        bus_write(REG_CR, 32'h00000001);

        while (!done && !fail && timeout < 10000) begin
            @(posedge clk);
            timeout = timeout + 1;
        end

        bus_read(REG_SR, sr);
        if (fail && (status == 8'hE1)) begin
            $display("TB_PASS falconsign_top_dynamic_tree_todo status=%02x sr=%08x cycles=%0d",
                     status, sr, timeout);
        end else begin
            $display("TB_FAIL falconsign_top_dynamic_tree_todo done=%0d fail=%0d irq=%0d status=%02x st=%0d sn=%0d sr=%08x timeout=%0d",
                     done, fail, bus_irq, status, dut.st, dut.sn, sr, timeout);
            $finish;
        end
        $finish;
    end
endmodule
