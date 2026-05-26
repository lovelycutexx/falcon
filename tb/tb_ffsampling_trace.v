`timescale 1ns/1ps
// Per-node trace: dump SPLIT/ADJUST/MERGE inputs/outputs for L=0 and L=1.
// Compare against C golden per-node output.

module tb_ffsampling_trace;
    reg clk, rst_n;
    reg [255:0] mem [0:8191];

    reg  ts_start; wire ts_start_ready;
    reg  [3:0] ts_cfg_depth; reg ts_cfg_dynamic;
    reg  [13:0] ts_t_base, ts_tree_base, ts_z_base, ts_tmp_base;
    wire ts_task_valid, ts_task_ready, ts_task_done, ts_task_fail;
    wire [67:0] ts_task_word; wire ts_busy, ts_done;

    falconsign_ffsampling_task_update #(.LEVEL_W(4),.INDEX_W(10),.ADDR_W(14)) u_ts(
        .clk(clk),.rst_n(rst_n),.start(ts_start),.start_ready(ts_start_ready),
        .cfg_depth(ts_cfg_depth),.cfg_dynamic_tree(ts_cfg_dynamic),
        .cfg_t_base(ts_t_base),.cfg_tree_base(ts_tree_base),.cfg_z_base(ts_z_base),
        .cfg_tmp_base(ts_tmp_base),
        .task_valid(ts_task_valid),.task_ready(ts_task_ready),
        .task_word(ts_task_word),.task_done(ts_task_done),.task_fail(ts_task_fail),
        .task_status(),.busy(ts_busy),.done(ts_done),.fail(),.status(),
        .dbg_level(),.dbg_index(),.dbg_state());

    wire fe_task_valid,fe_task_ready; wire[67:0] fe_task_word;
    wire fe_task_done,fe_task_fail;
    wire[13:0] fe_rd_addr,fe_wr_addr; wire[255:0] fe_rd_data,fe_wr_data;
    wire fe_rd_en,fe_wr_en; wire[9:0] tw_addr; reg[63:0] tw_re,tw_im;
    wire fe_fpu_req_valid,fe_fpu_req_ready; wire[3:0] fe_fpu_req_op;
    wire[63:0] fe_fpu_req_a,fe_fpu_req_b,fe_fpu_req_c;
    wire fe_fpu_rsp_valid; wire[63:0] fe_fpu_rsp_result;
    wire fe_sz_cmd_valid,fe_sz_cmd_ready; wire[63:0] sz_cmd_mu,sz_cmd_sigma_inv;
    wire sz_cmd_pair;

    falcon_f64_ffsampling_exu #(.ADDR_W(14)) u_fe(
        .clk(clk),.rst_n(rst_n),
        .task_valid(fe_task_valid),.task_ready(fe_task_ready),
        .task_word(fe_task_word),.task_done(fe_task_done),.task_fail(fe_task_fail),
        .task_status(),
        .mem_rd_en(fe_rd_en),.mem_rd_addr(fe_rd_addr),.mem_rd_data(fe_rd_data),
        .mem_wr_en(fe_wr_en),.mem_wr_addr(fe_wr_addr),.mem_wr_data(fe_wr_data),
        .twiddle_addr(tw_addr),.twiddle_re(tw_re),.twiddle_im(tw_im),
        .fpu_req_valid(fe_fpu_req_valid),.fpu_req_ready(fe_fpu_req_ready),
        .fpu_req_op(fe_fpu_req_op),.fpu_req_a(fe_fpu_req_a),
        .fpu_req_b(fe_fpu_req_b),.fpu_req_c(fe_fpu_req_c),
        .fpu_rsp_valid(fe_fpu_rsp_valid),.fpu_rsp_result(fe_fpu_rsp_result),
        .sz_cmd_valid(fe_sz_cmd_valid),.sz_cmd_ready(fe_sz_cmd_ready),
        .sz_cmd_mu(sz_cmd_mu),.sz_cmd_sigma_inv(sz_cmd_sigma_inv),.sz_cmd_pair(sz_cmd_pair),
        .sz_rsp_valid(1'b0),.sz_rsp_z0(64'd0),.sz_rsp_z1(64'd0));

    // FPU model : same as before
    reg fpu_val_q; reg[63:0] fpu_res_q;
    always @(posedge clk) begin
        if (fe_fpu_req_valid) begin
            case (fe_fpu_req_op)
                0: fpu_res_q <= $realtobits($bitstoreal(fe_fpu_req_a) + $bitstoreal(fe_fpu_req_b));
                1: fpu_res_q <= $realtobits($bitstoreal(fe_fpu_req_a) - $bitstoreal(fe_fpu_req_b));
                2: fpu_res_q <= $realtobits($bitstoreal(fe_fpu_req_a) * $bitstoreal(fe_fpu_req_b));
                3: fpu_res_q <= $realtobits($bitstoreal(fe_fpu_req_a) * $bitstoreal(fe_fpu_req_b) + $bitstoreal(fe_fpu_req_c));
                4: fpu_res_q <= $realtobits($bitstoreal(fe_fpu_req_a) * $bitstoreal(fe_fpu_req_b) - $bitstoreal(fe_fpu_req_c));
                6: fpu_res_q <= $realtobits(-$bitstoreal(fe_fpu_req_a) * $bitstoreal(fe_fpu_req_b) + $bitstoreal(fe_fpu_req_c));
            endcase fpu_val_q <= 1;
        end else fpu_val_q <= 0;
    end
    assign fe_fpu_rsp_valid=fpu_val_q; assign fe_fpu_rsp_result=fpu_res_q;
    assign fe_fpu_req_ready=1; assign fe_sz_cmd_ready=1;

    // Twiddle ROM
    reg [63:0] tw_rom_re[0:255], tw_rom_im[0:255]; integer ti;
    initial begin
        $readmemh("DOC/twiddle_rom_re.hex", tw_rom_re);
        $readmemh("DOC/twiddle_rom_im.hex", tw_rom_im);
    end
    assign tw_re=tw_rom_re[tw_addr]; assign tw_im=tw_rom_im[tw_addr];

    assign fe_rd_data = mem[fe_rd_addr];

    // Trace: capture write data and SamplerZ activity
    reg [63:0] last_wr_re, last_wr_im;
    reg [13:0] last_wr_addr;
    reg        last_wr_valid;

    always @(posedge clk) begin
        last_wr_valid <= fe_wr_en;
        if (fe_wr_en) begin
            mem[fe_wr_addr] <= fe_wr_data;
            last_wr_addr <= fe_wr_addr;
            last_wr_re   <= fe_wr_data[63:0];
            last_wr_im   <= fe_wr_data[127:64];
        end
    end

    // Per-node trace: watch critical memory regions
    // When EXU reads from specific addresses, capture the values
    wire rd_adj_t1 = fe_rd_en && fe_rd_addr >= 4480 && fe_rd_addr < 4610;
    wire rd_adj_z1 = fe_rd_en && fe_rd_addr >= 7552 && fe_rd_addr < 8070;
    wire rd_adj_t0 = fe_rd_en && fe_rd_addr >= 4352 && fe_rd_addr < 4480;
    wire rd_tree   = fe_rd_en && fe_rd_addr >= 1024 && fe_rd_addr < 1050;

    reg [63:0] cap_re, cap_im;
    reg [13:0] cap_addr;
    always @(posedge clk) if (fe_rd_en) begin
        cap_addr <= fe_rd_addr;
        cap_re   <= fe_rd_data[63:0];
        cap_im   <= fe_rd_data[127:64];
    end

    // Print only first few writes, then stop printing
    reg [31:0] print_cnt;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) print_cnt <= 0;
        else if (fe_wr_en && print_cnt < 100) begin
            print_cnt <= print_cnt + 1;
            $display("[%0t] WR[%0d] = re=%016x im=%016x  (task opq=%0d L=%0d)", $time, fe_wr_addr, fe_wr_data[63:0], fe_wr_data[127:64], u_fe.op_q, u_fe.level_q);
        end
    end

    assign fe_task_valid=ts_task_valid; assign fe_task_word=ts_task_word;
    assign ts_task_ready=fe_task_ready; assign ts_task_done=fe_task_done;
    assign ts_task_fail=fe_task_fail;

    always #5 clk = ~clk;

    initial begin
        integer i;
        clk=0; rst_n=0; ts_start=0;
        ts_cfg_depth=9; ts_cfg_dynamic=0;
        ts_t_base=0; ts_tree_base=1024; ts_z_base=3840; ts_tmp_base=7552;
        fpu_val_q=0;

        for (i=0;i<8192;i=i+1) mem[i]=256'd0;

        // t0: FFT([1]*512) → DC=512
        mem[0] = {128'd0, $realtobits(0.0), $realtobits(512.0)};
        // t1: FFT([2]*512) → DC=1024 at addr 512
        mem[512] = {128'd0, $realtobits(0.0), $realtobits(1024.0)};
        // Tree at 1024: all fpr_of(1.0)
        for (i=0;i<5120;i=i+1) mem[1024+i] = {128'd0, $realtobits(0.0), $realtobits(1.0)};

        #20 rst_n=1; #10;
        @(posedge clk);
        $display("=== ffSampling trace (depth=9, const signal) ===");
        ts_start=1; @(posedge clk); ts_start=0;
        wait(ts_done); @(posedge clk);

        $display("\n=== Final z ===");
        $display("C golden: z0[0]=4080259c9e82d752  z1[0]=40900382c8336cfc");
        for (i=0;i<4;i=i+1)
            $display("HW z0[%0d] re=%016x im=%016x", i, mem[3840+i][63:0], mem[3840+i][127:64]);
        for (i=0;i<4;i=i+1)
            $display("HW z1[%0d] re=%016x im=%016x", i, mem[4352+i][63:0], mem[4352+i][127:64]);
        $finish;
    end
endmodule
