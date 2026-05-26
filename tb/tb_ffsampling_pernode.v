`timescale 1ns/1ps
// ffSampling depth=9 per-node test: constant signal, compare with C golden.
// Memory layout matching falconsign_top.v:
//   T0=0, T1=512, TREE=1024, Z0=3840, Z1=4352, TMP=7552

module tb_ffsampling_pernode;
    reg clk, rst_n;
    reg [255:0] mem [0:8191];

    // Task scheduler
    reg         ts_start;  wire ts_start_ready;
    reg  [3:0]  ts_cfg_depth; reg ts_cfg_dynamic;
    reg  [13:0] ts_t_base, ts_tree_base, ts_z_base, ts_tmp_base;
    wire        ts_task_valid, ts_task_ready, ts_task_done, ts_task_fail;
    wire [67:0] ts_task_word;
    wire        ts_busy, ts_done;

    falconsign_ffsampling_task_update #(.LEVEL_W(4),.INDEX_W(10),.ADDR_W(14)) u_ts(
        .clk(clk),.rst_n(rst_n),.start(ts_start),.start_ready(ts_start_ready),
        .cfg_depth(ts_cfg_depth),.cfg_dynamic_tree(ts_cfg_dynamic),
        .cfg_t_base(ts_t_base),.cfg_tree_base(ts_tree_base),.cfg_z_base(ts_z_base),
        .cfg_tmp_base(ts_tmp_base),
        .task_valid(ts_task_valid),.task_ready(ts_task_ready),
        .task_word(ts_task_word),.task_done(ts_task_done),.task_fail(ts_task_fail),
        .task_status(),.busy(ts_busy),.done(ts_done),.fail(),.status(),
        .dbg_level(),.dbg_index(),.dbg_state());

    // EXU
    wire fe_task_valid,fe_task_ready; wire[67:0] fe_task_word;
    wire fe_task_done,fe_task_fail;
    wire[13:0] fe_rd_addr,fe_wr_addr; wire[255:0] fe_rd_data,fe_wr_data;
    wire fe_rd_en,fe_wr_en; wire[9:0] tw_addr; reg[63:0] tw_re,tw_im;
    wire fe_fpu_req_valid,fe_fpu_req_ready; wire[3:0] fe_fpu_req_op;
    wire[63:0] fe_fpu_req_a,fe_fpu_req_b,fe_fpu_req_c;
    wire fe_fpu_rsp_valid; wire[63:0] fe_fpu_rsp_result;
    wire fe_sz_cmd_valid,fe_sz_cmd_ready;
    reg [63:0] fe_sz_cmd_mu,fe_sz_cmd_sigma_inv; reg fe_sz_cmd_pair;
    reg fe_sz_rsp_valid; reg[63:0] fe_sz_rsp_z0,fe_sz_rsp_z1;

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
        .sz_cmd_mu(fe_sz_cmd_mu),.sz_cmd_sigma_inv(fe_sz_cmd_sigma_inv),
        .sz_cmd_pair(fe_sz_cmd_pair),.sz_rsp_valid(fe_sz_rsp_valid),
        .sz_rsp_z0(fe_sz_rsp_z0),.sz_rsp_z1(fe_sz_rsp_z1));

    // FPU
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
            endcase
            fpu_val_q <= 1;
        end else fpu_val_q <= 0;
    end
    assign fe_fpu_rsp_valid = fpu_val_q;
    assign fe_fpu_rsp_result = fpu_res_q;
    assign fe_fpu_req_ready = 1;

    // SamplerZ identity
    assign fe_sz_cmd_ready = 1;
    always @(posedge clk) begin
        fe_sz_rsp_valid <= fe_sz_cmd_valid;
        if (fe_sz_cmd_valid) begin
            fe_sz_rsp_z0 <= fe_sz_cmd_mu;
            fe_sz_rsp_z1 <= fe_sz_cmd_mu;
        end
    end

    // Twiddle ROM — use correct cos/-sin table (matching gen_twiddle_rom.py)
    reg [63:0] tw_rom_re[0:255], tw_rom_im[0:255];
    integer ti;
    initial begin
        $readmemh("DOC/twiddle_rom_re.hex", tw_rom_re);
        $readmemh("DOC/twiddle_rom_im.hex", tw_rom_im);
        // Fill missing entry 255
        tw_rom_re[255] = $realtobits($cos(-2.0*3.1415926535897932*255.0/512.0));
        tw_rom_im[255] = $realtobits($sin(-2.0*3.1415926535897932*255.0/512.0));
    end
    assign tw_re = tw_rom_re[tw_addr];
    assign tw_im = tw_rom_im[tw_addr];

    assign fe_rd_data = mem[fe_rd_addr];
    always @(posedge clk) if (fe_wr_en) begin
        mem[fe_wr_addr] <= fe_wr_data;
        // Track first few writes to root-level z and TMP areas
        if (fe_wr_addr >= 4352 && fe_wr_addr < 4370 && $time < 200000)
            $display("[%0t] WR z1[%0d]=re=%016x im=%016x",
                $time, fe_wr_addr-4352, fe_wr_data[63:0], fe_wr_data[127:64]);
        if (fe_wr_addr >= 7552 && fe_wr_addr < 7570 && $time < 200000)
            $display("[%0t] WR TMP[%0d]=re=%016x im=%016x",
                $time, fe_wr_addr-7552, fe_wr_data[63:0], fe_wr_data[127:64]);
    end

    assign fe_task_valid=ts_task_valid; assign fe_task_word=ts_task_word;
    assign ts_task_ready=fe_task_ready; assign ts_task_done=fe_task_done;
    assign ts_task_fail=fe_task_fail;

    always #5 clk = ~clk;

    reg [31:0] cycle;
    always @(posedge clk) if(rst_n) cycle <= cycle + 1;

    initial begin
        integer i;
        clk=0; rst_n=0; cycle=0;
        ts_start=0; ts_cfg_depth=9; ts_cfg_dynamic=0;
        ts_t_base=0; ts_tree_base=1024; ts_z_base=3840; ts_tmp_base=7552;
        fpu_val_q=0; fe_sz_rsp_valid=0;

        for (i=0;i<8192;i=i+1) mem[i]=256'd0;

        // t0 at 0: FFT([1]*512) → DC=512
        mem[0] = {128'd0, $realtobits(0.0), $realtobits(512.0)};

        // t1 at 512: FFT([2]*512) → DC=1024
        mem[512] = {128'd0, $realtobits(0.0), $realtobits(1024.0)};

        // Tree at 1024: all fpr_of(1.0)
        for (i=0;i<5120;i=i+1) mem[1024+i] = {128'd0, $realtobits(0.0), $realtobits(1.0)};

        #20 rst_n=1; #10;
        @(posedge clk);
        $display("=== ffSampling depth=9, constant signal ===");
        ts_start=1; @(posedge clk); ts_start=0;

        wait(ts_done); @(posedge clk);
        $display("=== Done after %0d cycles ===", cycle);

        $display("z0:");
        for (i=0;i<4;i=i+1) $display("  z0[%0d] re=%016x im=%016x", i, mem[3840+i][63:0], mem[3840+i][127:64]);
        $display("z1:");
        for (i=0;i<4;i=i+1) $display("  z1[%0d] re=%016x im=%016x", i, mem[4352+i][63:0], mem[4352+i][127:64]);

        $finish;
    end
endmodule
