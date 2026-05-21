`timescale 1ns/1ps
// ffSampling full tree test: scheduler + EXU + memory, verify traversal completes.

module tb_ffsampling_tree;
    reg clk, rst_n;

    // Simple 256-bit memory (1024 words)
    reg [255:0] mem [0:16383];
    wire [13:0]  rd_addr, wr_addr;
    wire [255:0] wr_data;
    reg  [255:0] rd_data;
    wire        rd_en, wr_en;
    assign rd_data = rd_en ? mem[rd_addr] : 256'd0;
    always @(posedge clk) if (wr_en) mem[wr_addr] <= wr_data;

    // Scheduler
    reg         ts_start;
    wire        ts_start_ready;
    reg  [3:0]  ts_cfg_depth;
    reg         ts_cfg_dynamic;
    reg  [13:0]  ts_t_base, ts_tree_base, ts_z_base;
    wire        ts_task_valid, ts_task_ready;
    wire [67:0] ts_task_word;
    wire        ts_task_done, ts_task_fail;
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

    // EXU
    wire        fe_task_valid, fe_task_ready;
    wire [67:0] fe_task_word;
    wire        fe_task_done, fe_task_fail;
    wire [13:0]  fe_rd_addr, fe_wr_addr;
    wire [255:0] fe_rd_data, fe_wr_data;
    wire [13:0]  tw_addr;
    reg  [63:0] tw_re, tw_im;
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
    reg         sz_pending;
    reg  [63:0] sz_pending_mu;

    falcon_f64_ffsampling_exu #(.ADDR_W(14)) u_fe(
        .clk(clk),.rst_n(rst_n),
        .task_valid(fe_task_valid),.task_ready(fe_task_ready),
        .task_word(fe_task_word),.task_done(fe_task_done),
        .task_fail(fe_task_fail),.task_status(),
        .mem_rd_en(rd_en),.mem_rd_addr(fe_rd_addr),
        .mem_rd_data(rd_data),
        .mem_wr_en(wr_en),.mem_wr_addr(fe_wr_addr),
        .mem_wr_data(wr_data),
        .twiddle_addr(tw_addr),.twiddle_re(tw_re),.twiddle_im(tw_im),
        .fpu_req_valid(fe_fpu_req_valid),.fpu_req_ready(fe_fpu_req_ready),
        .fpu_req_op(fe_fpu_req_op),.fpu_req_a(fe_fpu_req_a),
        .fpu_req_b(fe_fpu_req_b),.fpu_req_c(fe_fpu_req_c),
        .fpu_rsp_valid(fe_fpu_rsp_valid),.fpu_rsp_result(fe_fpu_rsp_result),
        .sz_cmd_valid(fe_sz_cmd_valid),.sz_cmd_ready(fe_sz_cmd_ready),
        .sz_cmd_mu(fe_sz_cmd_mu),.sz_cmd_sigma_inv(fe_sz_cmd_sigma_inv),
        .sz_cmd_pair(fe_sz_cmd_pair),.sz_rsp_valid(fe_sz_rsp_valid),
        .sz_rsp_z0(fe_sz_rsp_z0),.sz_rsp_z1(fe_sz_rsp_z1));

    // FPU model (1-cycle delay)
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

    // SamplerZ stub (identity mode): return z=mu.
    assign fe_sz_cmd_ready = 1;
    always @(posedge clk) begin
        fe_sz_rsp_valid <= sz_pending;
        if (sz_pending) begin
            fe_sz_rsp_z0 <= sz_pending_mu;
            fe_sz_rsp_z1 <= sz_pending_mu;
        end
        sz_pending <= fe_sz_cmd_valid;
        if (fe_sz_cmd_valid) begin
            sz_pending_mu <= fe_sz_cmd_mu;
        end
    end

    // Twiddle ROM
    reg [63:0] tw_rom[0:1023];
    initial begin
        integer i; real pi;
        pi = 3.1415926535897932;
        for (i = 0; i < 512; i = i + 1) begin
            tw_rom[i] = $realtobits($cos(-2.0 * pi * i / 512));
        end
    end
    assign tw_re = tw_rom[tw_addr];
    assign tw_im = 64'd0;  // simplified: real twiddles only for this test

    // Task routing: scheduler → EXU
    assign fe_task_valid = ts_task_valid;
    assign fe_task_word  = ts_task_word;
    assign ts_task_ready = fe_task_ready;
    assign ts_task_done  = fe_task_done;
    assign ts_task_fail  = fe_task_fail;

    always #5 clk = ~clk;

    initial begin
        integer i;
        clk = 0; rst_n = 0;
        fpu_val_q = 0;
        fpu_res_q = 0;
        sz_pending = 0;
        sz_pending_mu = 0;
        fe_sz_rsp_valid = 0;
        ts_cfg_depth = 4'd2;
        ts_cfg_dynamic = 0;
        ts_t_base = 10'd100;
        ts_tree_base = 10'd300;
        ts_z_base = 10'd700;

        // Fill memory with dummy data
        for (i = 0; i < 16384; i = i + 1) mem[i] = 256'd0;
        mem[100] = {128'd0, $realtobits(6.0), $realtobits(5.0)};
        mem[101] = {128'd0, $realtobits(8.0), $realtobits(7.0)};
        mem[104] = {128'd0, $realtobits(2.0), $realtobits(1.0)};
        mem[105] = {128'd0, $realtobits(4.0), $realtobits(3.0)};
        // L10/tree values are irrelevant for identity once z1=t1; keep zeros.

        #20 rst_n = 1; #10;

        // Start scheduler
        @(posedge clk);
        $display("Starting ffSampling tree traversal...");
        ts_start = 1;
        @(posedge clk);
        ts_start = 0;

        // Wait for completion
        wait(ts_done);
        @(posedge clk);
        $display("z0[0]=%h exp=%h", mem[700], mem[100]);
        $display("z0[1]=%h exp=%h", mem[701], mem[101]);
        $display("z1[0]=%h exp=%h", mem[704], mem[104]);
        $display("z1[1]=%h exp=%h", mem[705], mem[105]);
        if (mem[700] === mem[100] && mem[701] === mem[101]
            && mem[704] === mem[104] && mem[705] === mem[105]) begin
            $display("TB_PASS ffsampling_tree_identity_depth2");
        end else begin
            $display("TB_FAIL ffsampling_tree_identity_depth2");
        end
        $finish;
    end

    // Progress monitor
    always @(posedge clk) begin
        if (ts_task_valid && ts_task_ready) begin
            $display("  TASK op=%d lv=%d idx=%d", ts_task_word[67:64],
                ts_task_word[63:60], ts_task_word[59:50]);
        end
        if (fe_sz_cmd_valid && fe_sz_cmd_ready) begin
            $display("    SZ mu=%h", fe_sz_cmd_mu);
        end
        if (wr_en && (wr_addr >= 700) && (wr_addr < 708)) begin
            $display("    WR z[%0d]=%h", wr_addr, wr_data);
        end
        if (ts_done) $display("  SCHEDULER DONE");
    end

endmodule
