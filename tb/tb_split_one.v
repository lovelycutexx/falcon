`timescale 1ns/1ps
// Single SPLIT test: compare with C golden output

module tb_split_one;
    reg clk, rst_n;
    reg [255:0] mem [0:255];

    // EXU
    reg  task_valid; wire task_ready; reg [67:0] task_word;
    wire task_done, task_fail; wire [7:0] task_status;
    wire mem_rd_en; wire [9:0] mem_rd_addr; wire [255:0] mem_rd_data;
    wire mem_wr_en; wire [9:0] mem_wr_addr; wire [255:0] mem_wr_data;
    wire [9:0] tw_addr; reg [63:0] tw_re, tw_im;
    wire fpu_req_valid; wire fpu_req_ready = 1'b1;
    wire [3:0] fpu_req_op; wire [63:0] fpu_req_a, fpu_req_b, fpu_req_c;
    wire fpu_rsp_valid; wire [63:0] fpu_rsp_result;
    wire sz_cmd_valid; wire sz_cmd_ready = 1'b1;
    wire [63:0] sz_cmd_mu, sz_cmd_sigma_inv; wire sz_cmd_pair;

    falcon_f64_ffsampling_exu #(.ADDR_W(10)) u_fe(
        .clk(clk),.rst_n(rst_n),
        .task_valid(task_valid),.task_ready(task_ready),
        .task_word(task_word),.task_done(task_done),.task_fail(task_fail),
        .task_status(task_status),
        .mem_rd_en(mem_rd_en),.mem_rd_addr(mem_rd_addr),.mem_rd_data(mem_rd_data),
        .mem_wr_en(mem_wr_en),.mem_wr_addr(mem_wr_addr),.mem_wr_data(mem_wr_data),
        .twiddle_addr(tw_addr),.twiddle_re(tw_re),.twiddle_im(tw_im),
        .fpu_req_valid(fpu_req_valid),.fpu_req_ready(fpu_req_ready),
        .fpu_req_op(fpu_req_op),.fpu_req_a(fpu_req_a),.fpu_req_b(fpu_req_b),.fpu_req_c(fpu_req_c),
        .fpu_rsp_valid(fpu_rsp_valid),.fpu_rsp_result(fpu_rsp_result),
        .sz_cmd_valid(sz_cmd_valid),.sz_cmd_ready(sz_cmd_ready),
        .sz_cmd_mu(sz_cmd_mu),.sz_cmd_sigma_inv(sz_cmd_sigma_inv),.sz_cmd_pair(sz_cmd_pair),
        .sz_rsp_valid(1'b0),.sz_rsp_z0(64'd0),.sz_rsp_z1(64'd0));

    reg fpu_val_q; reg [63:0] fpu_res_q;
    always @(posedge clk) begin
        if (fpu_req_valid) begin
            case (fpu_req_op)
                0: fpu_res_q <= $realtobits($bitstoreal(fpu_req_a) + $bitstoreal(fpu_req_b));
                1: fpu_res_q <= $realtobits($bitstoreal(fpu_req_a) - $bitstoreal(fpu_req_b));
                2: fpu_res_q <= $realtobits($bitstoreal(fpu_req_a) * $bitstoreal(fpu_req_b));
                3: fpu_res_q <= $realtobits($bitstoreal(fpu_req_a) * $bitstoreal(fpu_req_b) + $bitstoreal(fpu_req_c));
                4: fpu_res_q <= $realtobits($bitstoreal(fpu_req_a) * $bitstoreal(fpu_req_b) - $bitstoreal(fpu_req_c));
                6: fpu_res_q <= $realtobits(-$bitstoreal(fpu_req_a) * $bitstoreal(fpu_req_b) + $bitstoreal(fpu_req_c));
                default: fpu_res_q <= 0;
            endcase
            fpu_val_q <= 1;
        end else fpu_val_q <= 0;
    end
    assign fpu_rsp_valid = fpu_val_q;
    assign fpu_rsp_result = fpu_res_q;

    // Twiddle ROM — correct cos/-sin table
    reg [63:0] gm_re[0:255], gm_im[0:255];
    initial begin
        $readmemh("DOC/twiddle_rom_re.hex", gm_re);
        $readmemh("DOC/twiddle_rom_im.hex", gm_im);
        gm_re[255] = $realtobits($cos(-2.0*3.1415926535897932*255.0/512.0));
        gm_im[255] = $realtobits($sin(-2.0*3.1415926535897932*255.0/512.0));
    end
    assign tw_re = gm_re[tw_addr];
    assign tw_im = gm_im[tw_addr];

    assign mem_rd_data = mem[mem_rd_addr];
    always @(posedge clk) if (mem_wr_en) begin
        mem[mem_wr_addr] <= mem_wr_data;
        $display("WR[%0d] = %016x %016x  (re=%016x im=%016x)",
            mem_wr_addr, mem_wr_data[255:128], mem_wr_data[127:0],
            mem_wr_data[63:0], mem_wr_data[127:64]);
    end

    always #5 clk = ~clk;

    task send_task;
        input [67:0] tw;
        begin
            @(posedge clk); task_valid=1; task_word=tw;
            @(posedge clk); task_valid=0;
            while (!task_done) @(posedge clk);
            @(posedge clk);
        end
    endtask

    initial begin
        integer i;
        clk=0; rst_n=0; task_valid=0; fpu_val_q=0;

        for (i=0;i<256;i=i+1) mem[i]=256'd0;

        // Falcon FFT of real constant: DC=1024, rest=0
        // mem layout for SPLIT: a at 2*idx, b at 2*idx+1
        // idx=0: a=mem[0]=(1024,0), b=mem[1]=(0,0)
        // All other pairs: (0,0)
        mem[0] = {128'd0, $realtobits(0.0), $realtobits(1024.0)};
        mem[1] = {128'd0, $realtobits(0.0), $realtobits(0.0)};

        #20 rst_n=1; #10;

        // SPLIT at level 0
        $display("=== Single SPLIT test (L=0, src=0, dst=0) ===");
        $display("Input a=(1024,0) b=(0,0)");

        // Task word: op[67:64] level[63:60] idx[59:50] src0[49:36] src1[35:22] dst[21:8] aux[7:0]
        send_task({4'd1, 4'd0, 10'd0, 14'd0, 14'd0, 14'd0, 8'd0});

        $display("Time=%0t task_done=%b task_fail=%b", $time, task_done, task_fail);
        $display("After SPLIT:");
        $display("  f0[0] re=%016x im=%016x", mem[0][63:0], mem[0][127:64]);
        $display("  f0[1] re=%016x im=%016x", mem[1][63:0], mem[1][127:64]);
        $display("  f1[0] re=%016x im=%016x (at addr 128)", mem[128][63:0], mem[128][127:64]);
        $display("  f1[1] re=%016x im=%016x (at addr 129)", mem[129][63:0], mem[129][127:64]);

        $display("\n=== C golden expected ===");
        $display("  f0[0] = (512, 0) = 4090000000000000 0000000000000000");
        $display("  f1[0] = ((1024-0)*1)/2 = (512, 0)");

        $finish;
    end
endmodule
