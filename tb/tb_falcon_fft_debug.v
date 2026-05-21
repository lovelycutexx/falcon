`timescale 1ns/1ps

// Minimal N=4 IFFT test for debugging
module tb_falcon_fft_debug;

    localparam ADDR_W = 4;
    localparam N = 4;
    localparam LOGN = 2;

    reg clk, rst_n;
    reg cmd_valid, cmd_opcode;
    reg [4:0] cmd_logn;
    wire cmd_ready, fft_busy;
    wire fft_rsp_valid, fft_rsp_done, fft_rsp_fail;
    wire [7:0] fft_rsp_status;

    wire [ADDR_W-1:0] fft_rd_addr0, fft_rd_addr1;
    wire [63:0] fft_rd_data0_re, fft_rd_data0_im;
    wire [63:0] fft_rd_data1_re, fft_rd_data1_im;
    wire [ADDR_W-1:0] fft_twiddle_addr;
    wire [63:0] fft_twiddle_re, fft_twiddle_im;
    wire fft_wr_en;
    wire [ADDR_W-1:0] fft_wr_addr0, fft_wr_addr1;
    wire [63:0] fft_wr_data0_re, fft_wr_data0_im;
    wire [63:0] fft_wr_data1_re, fft_wr_data1_im;

    falcon_f64_fft_exu #(.ADDR_W(ADDR_W)) u_fft (
        .clk(clk), .rst_n(rst_n),
        .cmd_valid(cmd_valid), .cmd_ready(cmd_ready),
        .cmd_opcode(cmd_opcode), .cmd_logn(cmd_logn),
        .mem_rd_addr0(fft_rd_addr0), .mem_rd_addr1(fft_rd_addr1),
        .mem_rd_data0_re(fft_rd_data0_re), .mem_rd_data0_im(fft_rd_data0_im),
        .mem_rd_data1_re(fft_rd_data1_re), .mem_rd_data1_im(fft_rd_data1_im),
        .twiddle_addr(fft_twiddle_addr), .twiddle_re(fft_twiddle_re),
        .twiddle_im(fft_twiddle_im),
        .mem_wr_en(fft_wr_en), .mem_wr_addr0(fft_wr_addr0),
        .mem_wr_addr1(fft_wr_addr1),
        .mem_wr_data0_re(fft_wr_data0_re), .mem_wr_data0_im(fft_wr_data0_im),
        .mem_wr_data1_re(fft_wr_data1_re), .mem_wr_data1_im(fft_wr_data1_im),
        .rsp_valid(fft_rsp_valid), .rsp_done(fft_rsp_done),
        .rsp_fail(fft_rsp_fail), .rsp_status(fft_rsp_status),
        .busy(fft_busy)
    );

    falconsign_twiddle_rom #(.ADDR_W(8), .DEPTH(256)) u_twiddle (
        .clk(clk), .addr({4'd0, fft_twiddle_addr}),
        .twiddle_re(fft_twiddle_re), .twiddle_im(fft_twiddle_im)
    );

    // Combinational memory
    reg [63:0] mem_re [0:15];
    reg [63:0] mem_im [0:15];
    assign fft_rd_data0_re = mem_re[fft_rd_addr0];
    assign fft_rd_data0_im = mem_im[fft_rd_addr0];
    assign fft_rd_data1_re = mem_re[fft_rd_addr1];
    assign fft_rd_data1_im = mem_im[fft_rd_addr1];

    always @(posedge clk) begin
        if (fft_wr_en) begin
            mem_re[fft_wr_addr0] <= fft_wr_data0_re;
            mem_im[fft_wr_addr0] <= fft_wr_data0_im;
            mem_re[fft_wr_addr1] <= fft_wr_data1_re;
            mem_im[fft_wr_addr1] <= fft_wr_data1_im;
        end
    end

    initial begin clk = 0; rst_n = 0; #30 rst_n = 1; end
    always #5 clk = ~clk;

    // Trace FPU operations inside BFU
    always @(posedge clk) begin
        if (u_fft.u_bfly.fpu_req_valid && u_fft.u_bfly.fpu_req_ready)
            $display("[%0t] FPU REQ: op=%0d a=%f b=%f c=%f",
                     $time,
                     u_fft.u_bfly.fpu_req_op,
                     $bitstoreal(u_fft.u_bfly.fpu_req_a),
                     $bitstoreal(u_fft.u_bfly.fpu_req_b),
                     $bitstoreal(u_fft.u_bfly.fpu_req_c));
        if (u_fft.u_bfly.u_fpu.rsp_valid && u_fft.u_bfly.u_fpu.rsp_ready)
            $display("[%0t] FPU RSP: result=%f flags=%b",
                     $time,
                     $bitstoreal(u_fft.u_bfly.u_fpu.rsp_result),
                     u_fft.u_bfly.u_fpu.rsp_flags);
    end

    // Trace signals
    always @(posedge clk) begin
        if (fft_wr_en)
            $display("[%0t] WR addr=%0d data0=(%f,%f) data1=(%f,%f)",
                     $time, fft_wr_addr0,
                     $bitstoreal(fft_wr_data0_re), $bitstoreal(fft_wr_data0_im),
                     $bitstoreal(fft_wr_data1_re), $bitstoreal(fft_wr_data1_im));
    end

    reg [31:0] cycle;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cycle <= 0; cmd_valid <= 0; cmd_opcode <= 0; cmd_logn <= 0;
        end else begin
            cycle <= cycle + 1; cmd_valid <= 0;
            if (cycle == 10) begin
                $display("=== N=4 IFFT: input = [1+0j, 1+0j, 1+0j, 1+0j] ===");
                mem_re[0] = 64'h3FF0000000000000; mem_im[0] = 64'd0;
                mem_re[1] = 64'h3FF0000000000000; mem_im[1] = 64'd0;
                mem_re[2] = 64'h3FF0000000000000; mem_im[2] = 64'd0;
                mem_re[3] = 64'h3FF0000000000000; mem_im[3] = 64'd0;
                cmd_valid <= 1; cmd_opcode <= 1'b1; cmd_logn <= LOGN;
            end
            if (fft_rsp_valid && fft_rsp_done) begin
                $display("Output[0] = (%f, %f)", $bitstoreal(mem_re[0]), $bitstoreal(mem_im[0]));
                $display("Output[1] = (%f, %f)", $bitstoreal(mem_re[1]), $bitstoreal(mem_im[1]));
                $display("Output[2] = (%f, %f)", $bitstoreal(mem_re[2]), $bitstoreal(mem_im[2]));
                $display("Output[3] = (%f, %f)", $bitstoreal(mem_re[3]), $bitstoreal(mem_im[3]));
                $display("Expected: [0]=(1.0, 0.0), [1]=(0.0, 0.0), [2]=(0.0, 0.0), [3]=(0.0, 0.0)");
                #100 $finish;
            end
        end
    end

    initial begin #500000; $display("TIMEOUT"); $finish; end
    initial begin $dumpfile("tb_falcon_fft_debug.vcd"); $dumpvars(0, tb_falcon_fft_debug); end
endmodule
