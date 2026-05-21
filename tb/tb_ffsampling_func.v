`timescale 1ns/1ps
// ffSampling functional test: load data → run tree → compare z.

module tb_ffsampling_func;
    reg clk, rst_n;

    reg [255:0] mem [0:16383];
    wire [13:0]  rd_addr; reg [255:0] rd_data;
    wire [13:0]  wr_addr; wire [255:0] wr_data;
    wire        rd_en, wr_en;
    assign rd_data = rd_en ? mem[rd_addr] : 256'd0;
    always @(posedge clk) if (wr_en) mem[wr_addr] <= wr_data;

    // Read 32-bit memory file into 256-bit words
    reg [31:0] m32 [0:131071];
    integer i, j, word_cnt;
    reg found_last;
    initial begin
        $readmemh("ffsampling_func.mem", m32);
        found_last = 0;
        word_cnt = 4096;
        for (i = 0; i < 4096; i = i + 1) begin
            if (!found_last && m32[i] === 32'hxxxxxxxx) begin word_cnt = i; found_last = 1; end
        end
        // Pack 8 × 32-bit into 1 × 256-bit
        for (i = 0; i < word_cnt / 8; i = i + 1) begin
            mem[i] = {m32[i*8+7], m32[i*8+6], m32[i*8+5], m32[i*8+4],
                      m32[i*8+3], m32[i*8+2], m32[i*8+1], m32[i*8+0]};
        end
        $display("Loaded %0d 256-bit words from mem file", word_cnt/8);
    end

    // Scheduler
    reg         ts_start;
    wire        ts_start_ready;
    reg  [3:0]  ts_cfg_depth;
    reg         ts_cfg_dynamic;
    reg  [13:0]  ts_t_base, ts_tree_base, ts_z_base;
    wire        ts_task_valid, ts_task_ready, ts_task_done, ts_task_fail;
    wire [67:0] ts_task_word;
    wire        ts_busy, ts_done;

    falconsign_ffsampling_task_update #(.LEVEL_W(4),.INDEX_W(10),.ADDR_W(14)) u_ts(
        .clk(clk),.rst_n(rst_n),
        .start(ts_start),.start_ready(ts_start_ready),
        .cfg_depth(ts_cfg_depth),.cfg_dynamic_tree(ts_cfg_dynamic),
        .cfg_t_base(ts_t_base),.cfg_tree_base(ts_tree_base),.cfg_z_base(ts_z_base),
        .task_valid(ts_task_valid),.task_ready(ts_task_ready),
        .task_word(ts_task_word),.task_done(ts_task_done),
        .task_fail(ts_task_fail),.task_status(),
        .busy(ts_busy),.done(ts_done),.fail(),.status(),
        .dbg_level(),.dbg_index(),.dbg_state());

    // EXU + FPU + SamplerZ (same as tree test)
    wire        fe_task_valid, fe_task_ready;
    wire [67:0] fe_task_word;
    wire        fe_task_done, fe_task_fail;
    wire [13:0]  fe_rd_addr, fe_wr_addr;
    wire [255:0] fe_rd_data, fe_wr_data;
    wire [9:0]  tw_addr; reg [63:0] tw_re, tw_im;
    wire        fe_fpu_req_valid, fe_fpu_req_ready;
    wire [3:0]  fe_fpu_req_op;
    wire [63:0] fe_fpu_req_a, fe_fpu_req_b, fe_fpu_req_c;
    wire        fe_fpu_rsp_valid;
    wire [63:0] fe_fpu_rsp_result;
    wire        fe_sz_cmd_valid, fe_sz_cmd_ready;
    reg  [63:0] fe_sz_cmd_mu, fe_sz_cmd_sigma_inv;
    reg         fe_sz_cmd_pair;
    reg         fe_sz_rsp_valid;
    reg  [63:0] fe_sz_rsp_z0, fe_sz_rsp_z1;

    falcon_f64_ffsampling_exu #(.ADDR_W(14)) u_fe(
        .clk(clk),.rst_n(rst_n),
        .task_valid(fe_task_valid),.task_ready(fe_task_ready),
        .task_word(fe_task_word),.task_done(fe_task_done),
        .task_fail(fe_task_fail),.task_status(),
        .mem_rd_en(rd_en),.mem_rd_addr(fe_rd_addr),
        .mem_rd_data(rd_data),.mem_wr_en(wr_en),
        .mem_wr_addr(fe_wr_addr),.mem_wr_data(wr_data),
        .twiddle_addr(tw_addr),.twiddle_re(tw_re),.twiddle_im(tw_im),
        .fpu_req_valid(fe_fpu_req_valid),.fpu_req_ready(fe_fpu_req_ready),
        .fpu_req_op(fe_fpu_req_op),.fpu_req_a(fe_fpu_req_a),
        .fpu_req_b(fe_fpu_req_b),.fpu_req_c(fe_fpu_req_c),
        .fpu_rsp_valid(fe_fpu_rsp_valid),.fpu_rsp_result(fe_fpu_rsp_result),
        .sz_cmd_valid(fe_sz_cmd_valid),.sz_cmd_ready(fe_sz_cmd_ready),
        .sz_cmd_mu(fe_sz_cmd_mu),.sz_cmd_sigma_inv(fe_sz_cmd_sigma_inv),
        .sz_cmd_pair(fe_sz_cmd_pair),.sz_rsp_valid(fe_sz_rsp_valid),
        .sz_rsp_z0(fe_sz_rsp_z0),.sz_rsp_z1(fe_sz_rsp_z1));

    reg fpu_val_q; reg [63:0] fpu_res_q;
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

    assign fe_sz_cmd_ready = 1;
    always @(posedge clk) begin
        fe_sz_rsp_valid <= fe_sz_cmd_valid;
        if (fe_sz_cmd_valid) begin
            fe_sz_rsp_z0 <= $realtobits(0.0);
            fe_sz_rsp_z1 <= $realtobits(0.0);
        end
    end

    reg [63:0] tw_rom[0:1023];
    initial begin
        integer i; real pi; pi = 3.1415926535897932;
        for (i = 0; i < 512; i = i + 1)
            tw_rom[i] = $realtobits($cos(-2.0 * pi * i / 512));
    end
    assign tw_re = tw_rom[tw_addr];
    assign tw_im = 64'd0;

    assign fe_task_valid = ts_task_valid;
    assign fe_task_word  = ts_task_word;
    assign ts_task_ready = fe_task_ready;
    assign ts_task_done  = fe_task_done;
    assign ts_task_fail  = fe_task_fail;

    // Load expected z from file
    reg [63:0] exp_z_re [0:511], exp_z_im [0:511];
    reg [31:0] e32 [0:2047];
    initial begin
        $readmemh("ffsampling_expected_z.mem", e32);
        for (i = 0; i < 512; i = i + 1) begin
            exp_z_re[i] = {e32[i*4], e32[i*4+1]};
            exp_z_im[i] = {e32[i*4+2], e32[i*4+3]};
        end
        $display("Loaded expected z (sample: z[0]=%f %fi)", $bitstoreal(exp_z_re[0]), $bitstoreal(exp_z_im[0]));
    end

    always #5 clk = ~clk;

    integer pass, fail_c;
    reg [63:0] hw_re, hw_im;
    reg [255:0] hw_word;
    initial begin
        clk = 0; rst_n = 0; ts_start = 0;
        ts_cfg_depth = 9; ts_cfg_dynamic = 0;
        ts_t_base = 14'd0; ts_tree_base = 14'd4608;  // t at word 0, tree after all segments (9*512=4608)
        ts_z_base = 14'd0;
        pass = 0; fail_c = 0;

        #20 rst_n = 1; #10;

        @(posedge clk); ts_start = 1;
        @(posedge clk); ts_start = 0;

        wait(ts_done);
        @(posedge clk);
        $display("Tree completed after %0d cycles", $time/10);

        // Read back z from memory (MERGE writes one complex value per word at z_base)
        // Each word has [63:0]=re, [127:64]=im, [255:128]=0
        for (i = 0; i < 512; i = i + 1) begin
            hw_word = mem[i];
            hw_re = hw_word[63:0];
            hw_im = hw_word[127:64];
            if (i == 0 || i == 100 || i == 200)
                $display("  z[%0d] = %f %fi (exp %f %fi)", i,
                    $bitstoreal(hw_re), $bitstoreal(hw_im),
                    $bitstoreal(exp_z_re[i]), $bitstoreal(exp_z_im[i]));
            if (hw_re == exp_z_re[i] && hw_im == exp_z_im[i])
                pass = pass + 1;
            else
                fail_c = fail_c + 1;
        end

        if (fail_c == 0)
            $display("PASS: all %0d z values match", pass);
        else
            $display("FAIL: %0d pass, %0d fail", pass, fail_c);
        $finish;
    end
endmodule
