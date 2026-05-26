`timescale 1ns/1ps

module tb_ffsampling_layer_compare;
    localparam ADDR_W = 14;
    localparam OP_SPLIT  = 4'd1;
    localparam OP_ADJUST = 4'd2;
    localparam OP_SAMPLE = 4'd3;
    localparam OP_MERGE  = 4'd4;
    localparam OP_COPY   = 4'd6;

    localparam T_BASE    = 14'd0;
    localparam TREE_BASE = 14'd1024;
    localparam Z_BASE    = 14'd3840;
    localparam TMP_BASE  = 14'd7552;
    localparam WORDS     = 512;
    localparam [63:0] ULP_TOL = 64'd1048576;

    reg clk;
    reg rst_n;

    reg [255:0] mem [0:16383];
    reg [255:0] exp_t0_nodes [0:2303];
    reg [255:0] exp_t1_nodes [0:2303];
    reg [255:0] t0_initial [0:511];
    reg [255:0] t1_initial [0:511];

    wire fe_rd_en;
    wire [ADDR_W-1:0] fe_rd_addr;
    reg [255:0] fe_rd_data;
    wire fe_wr_en;
    wire [ADDR_W-1:0] fe_wr_addr;
    wire [255:0] fe_wr_data;

    reg ts_start;
    wire ts_start_ready;
    wire ts_task_valid;
    wire ts_task_ready;
    wire [67:0] ts_task_word;
    wire ts_task_done;
    wire ts_task_fail;
    wire ts_busy;
    wire ts_done;
    wire ts_fail;
    wire [7:0] ts_status;

    wire [ADDR_W-1:0] twiddle_addr;
    wire [63:0] twiddle_re;
    wire [63:0] twiddle_im;

    wire fpu_req_valid;
    wire fpu_req_ready;
    wire [3:0] fpu_req_op;
    wire [63:0] fpu_req_a;
    wire [63:0] fpu_req_b;
    wire [63:0] fpu_req_c;
    wire fpu_rsp_valid;
    wire [63:0] fpu_rsp_result;
    wire [4:0] fpu_rsp_flags;

    wire sz_cmd_valid;
    wire sz_cmd_ready;
    wire [63:0] sz_cmd_mu;
    wire [63:0] sz_cmd_sigma_inv;
    wire sz_cmd_pair;
    reg sz_rsp_valid;
    reg [63:0] sz_rsp_z0;
    reg [63:0] sz_rsp_z1;

    reg [67:0] active_task;
    reg active_bank;
    integer errors;
    integer merge_checks;
    integer adjust_checks;
    integer sample_count;
    integer first_bad_seen;
    integer verbose_pass;
    integer stop_first;
    integer i;

    falconsign_ffsampling_task_update #(.LEVEL_W(4), .INDEX_W(10), .ADDR_W(ADDR_W)) u_ts (
        .clk(clk),
        .rst_n(rst_n),
        .start(ts_start),
        .start_ready(ts_start_ready),
        .cfg_depth(4'd9),
        .cfg_dynamic_tree(1'b0),
        .cfg_t_base(T_BASE),
        .cfg_tree_base(TREE_BASE),
        .cfg_z_base(Z_BASE),
        .cfg_tmp_base(TMP_BASE),
        .task_valid(ts_task_valid),
        .task_ready(ts_task_ready),
        .task_word(ts_task_word),
        .task_done(ts_task_done),
        .task_fail(ts_task_fail),
        .task_status(8'd0),
        .busy(ts_busy),
        .done(ts_done),
        .fail(ts_fail),
        .status(ts_status),
        .dbg_level(),
        .dbg_index(),
        .dbg_state()
    );

    falcon_f64_ffsampling_exu #(.ADDR_W(ADDR_W)) u_fe (
        .clk(clk),
        .rst_n(rst_n),
        .task_valid(ts_task_valid),
        .task_ready(ts_task_ready),
        .task_word(ts_task_word),
        .task_done(ts_task_done),
        .task_fail(ts_task_fail),
        .task_status(),
        .mem_rd_en(fe_rd_en),
        .mem_rd_addr(fe_rd_addr),
        .mem_rd_data(fe_rd_data),
        .mem_wr_en(fe_wr_en),
        .mem_wr_addr(fe_wr_addr),
        .mem_wr_data(fe_wr_data),
        .twiddle_addr(twiddle_addr),
        .twiddle_re(twiddle_re),
        .twiddle_im(twiddle_im),
        .fpu_req_valid(fpu_req_valid),
        .fpu_req_ready(fpu_req_ready),
        .fpu_req_op(fpu_req_op),
        .fpu_req_a(fpu_req_a),
        .fpu_req_b(fpu_req_b),
        .fpu_req_c(fpu_req_c),
        .fpu_rsp_valid(fpu_rsp_valid),
        .fpu_rsp_result(fpu_rsp_result),
        .sz_cmd_valid(sz_cmd_valid),
        .sz_cmd_ready(sz_cmd_ready),
        .sz_cmd_mu(sz_cmd_mu),
        .sz_cmd_sigma_inv(sz_cmd_sigma_inv),
        .sz_cmd_pair(sz_cmd_pair),
        .sz_rsp_valid(sz_rsp_valid),
        .sz_rsp_z0(sz_rsp_z0),
        .sz_rsp_z1(sz_rsp_z1)
    );

    falcon_fp_fpu u_fpu (
        .clk(clk),
        .rst_n(rst_n),
        .req_valid(fpu_req_valid),
        .req_ready(fpu_req_ready),
        .req_op(fpu_req_op),
        .req_a(fpu_req_a),
        .req_b(fpu_req_b),
        .req_c(fpu_req_c),
        .req_fmt(2'b01),
        .req_rm(3'b000),
        .req_fcvt_op(2'b00),
        .rsp_valid(fpu_rsp_valid),
        .rsp_ready(1'b1),
        .rsp_result(fpu_rsp_result),
        .rsp_flags(fpu_rsp_flags),
        .busy()
    );

    falconsign_gm_rom #(.ADDR_W(ADDR_W), .DEPTH(255)) u_gm (
        .clk(clk),
        .addr(twiddle_addr),
        .gm_re(twiddle_re),
        .gm_im(twiddle_im)
    );

    assign sz_cmd_ready = 1'b1;

    always #5 clk = ~clk;

    always @(*) begin
        fe_rd_data = mem[fe_rd_addr];
    end

    always @(posedge clk) begin
        if (fe_wr_en) begin
            mem[fe_wr_addr] <= fe_wr_data;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sz_rsp_valid <= 1'b0;
            sz_rsp_z0 <= 64'd0;
            sz_rsp_z1 <= 64'd0;
            sample_count <= 0;
        end else begin
            sz_rsp_valid <= sz_cmd_valid;
            if (sz_cmd_valid) begin
                sz_rsp_z0 <= sz_cmd_mu;
                sz_rsp_z1 <= 64'd0;
                sample_count <= sample_count + 1;
            end
        end
    end

    function [63:0] canon;
        input [63:0] v;
        begin
            canon = (v[62:0] == 63'd0) ? 64'd0 : v;
        end
    endfunction

    function word_match;
        input [255:0] got;
        input [255:0] exp;
        begin
            word_match = near64(got[63:0], exp[63:0])
                      && near64(got[127:64], exp[127:64]);
        end
    endfunction

    function near64;
        input [63:0] got;
        input [63:0] exp;
        reg [63:0] cg;
        reg [63:0] ce;
        reg [63:0] diff;
        begin
            cg = canon(got);
            ce = canon(exp);
            if (cg == ce) begin
                near64 = 1'b1;
            end else if (cg[63] != ce[63]) begin
                near64 = 1'b0;
            end else begin
                diff = (cg > ce) ? (cg - ce) : (ce - cg);
                near64 = (diff <= ULP_TOL);
            end
        end
    endfunction

    function integer node_words_at_level;
        input [3:0] level;
        begin
            node_words_at_level = 256 >> level;
        end
    endfunction

    function integer node_exp_base;
        input [3:0] level;
        input [9:0] index;
        begin
            node_exp_base = level * 256 + index * (256 >> level);
        end
    endfunction

    task compare_node;
        input [8*16-1:0] label;
        input [13:0] mem_base;
        input [3:0] level;
        input [9:0] index;
        input bank;
        integer k;
        integer bad;
        integer count;
        integer exp_base;
        reg [255:0] exp_word;
        begin
            count = node_words_at_level(level);
            exp_base = node_exp_base(level, index);
            bad = 0;
            for (k = 0; k < count; k = k + 1) begin
                exp_word = bank ? exp_t1_nodes[exp_base + k] : exp_t0_nodes[exp_base + k];
                if (!word_match(mem[mem_base + k], exp_word)) begin
                    bad = bad + 1;
                    if (!first_bad_seen) begin
                        first_bad_seen = 1;
                        $display("FIRST_MISMATCH %0s bank=%0d L=%0d I=%0d word=%0d addr=%0d got=%064x exp=%064x",
                            label, bank, level, index, k, mem_base + k, mem[mem_base + k], exp_word);
                        if (stop_first) begin
                            $display("TB_STOP_FIRST");
                            $finish;
                        end
                    end
                    if (bad <= 4) begin
                        $display("  MISMATCH %0s bank=%0d L=%0d I=%0d word=%0d got=%064x exp=%064x",
                            label, bank, level, index, k, mem[mem_base + k], exp_word);
                    end
                end
            end
            if (bad != 0) begin
                $display("CHECK_FAIL %0s bank=%0d L=%0d I=%0d bad=%0d/%0d base=%0d",
                    label, bank, level, index, bad, count, mem_base);
                errors = errors + bad;
            end else if (verbose_pass) begin
                $display("CHECK_PASS %0s bank=%0d L=%0d I=%0d count=%0d base=%0d",
                    label, bank, level, index, count, mem_base);
            end
        end
    endtask

    task compare_scalar_split;
        input [13:0] mem_base;
        input [3:0] level;
        input [9:0] index;
        input bank;
        integer exp_base;
        reg [255:0] exp_word;
        reg [63:0] got_re;
        reg [63:0] got_im;
        begin
            exp_base = node_exp_base(level, index);
            exp_word = bank ? exp_t1_nodes[exp_base] : exp_t0_nodes[exp_base];
            got_re = mem[mem_base][63:0];
            got_im = mem[mem_base + 1][63:0];
            if (!near64(got_re, exp_word[63:0]) ||
                !near64(got_im, exp_word[127:64])) begin
                if (!first_bad_seen) begin
                    first_bad_seen = 1;
                    $display("FIRST_MISMATCH SPLIT_SCALAR bank=%0d L=%0d I=%0d base=%0d got_re=%016x got_im=%016x exp=%064x",
                        bank, level, index, mem_base, got_re, got_im, exp_word);
                    if (stop_first) begin
                        $display("TB_STOP_FIRST");
                        $finish;
                    end
                end
                $display("CHECK_FAIL SPLIT_SCALAR bank=%0d L=%0d I=%0d base=%0d got_re=%016x got_im=%016x exp=%064x",
                    bank, level, index, mem_base, got_re, got_im, exp_word);
                errors = errors + 1;
            end else if (verbose_pass) begin
                $display("CHECK_PASS SPLIT_SCALAR bank=%0d L=%0d I=%0d base=%0d",
                    bank, level, index, mem_base);
            end
        end
    endtask

    task compare_full_final;
        input [13:0] mem_base;
        input bank;
        integer k;
        integer bad;
        reg [255:0] exp_word;
        begin
            bad = 0;
            for (k = 0; k < WORDS; k = k + 1) begin
                exp_word = bank ? t1_initial[k] : t0_initial[k];
                if (!word_match(mem[mem_base + k], exp_word)) begin
                    bad = bad + 1;
                    if (!first_bad_seen) begin
                        first_bad_seen = 1;
                        $display("FIRST_MISMATCH FINAL bank=%0d word=%0d got=%064x exp=%064x",
                            bank, k, mem[mem_base + k], exp_word);
                    end
                    if (bad <= 4) begin
                        $display("  FINAL_MISMATCH bank=%0d word=%0d got=%064x exp=%064x",
                            bank, k, mem[mem_base + k], exp_word);
                    end
                end
            end
            if (bad != 0) begin
                $display("CHECK_FAIL FINAL bank=%0d bad=%0d/%0d", bank, bad, WORDS);
                errors = errors + bad;
            end else if (verbose_pass) begin
                $display("CHECK_PASS FINAL bank=%0d count=%0d", bank, WORDS);
            end
        end
    endtask

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            active_task <= 68'd0;
            active_bank <= 1'b0;
        end else if (ts_task_valid && ts_task_ready) begin
            active_task <= ts_task_word;
            active_bank <= u_ts.bank_q;
            if (verbose_pass) begin
                $display("ISSUE op=%0d bank=%0d L=%0d I=%0d src0=%0d src1=%0d dst=%0d",
                    ts_task_word[67:64], u_ts.bank_q, ts_task_word[63:60], ts_task_word[59:50],
                    ts_task_word[49:36], ts_task_word[35:22], ts_task_word[21:8]);
            end
        end
    end

    always @(posedge clk) begin
        if (rst_n && ts_task_done && !ts_task_fail) begin
            if (active_task[67:64] == OP_COPY) begin
                if (active_task[63:60] < 4'd8) begin
                    compare_node("COPY_RIGHT", active_task[21:8], active_task[63:60] + 1'b1,
                        (active_task[59:50] << 1) + 1'b1, active_bank);
                end
            end else if (active_task[67:64] == OP_SPLIT) begin
                if (active_task[63:60] < 4'd8) begin
                    compare_node("SPLIT_LEFT", active_task[21:8], active_task[63:60] + 1'b1,
                        active_task[59:50] << 1, active_bank);
                    compare_node("SPLIT_RIGHT", active_task[21:8] + node_words_at_level(active_task[63:60] + 1'b1),
                        active_task[63:60] + 1'b1, (active_task[59:50] << 1) + 1'b1, active_bank);
                end else begin
                    compare_scalar_split(active_task[21:8], active_task[63:60],
                        active_task[59:50], active_bank);
                end
            end else if (active_task[67:64] == OP_MERGE) begin
                merge_checks = merge_checks + 1;
                compare_node("MERGE", active_task[21:8], active_task[63:60],
                    active_task[59:50], active_bank);
            end else if ($test$plusargs("CHECK_ADJUST") && (active_task[67:64] == OP_ADJUST)) begin
                adjust_checks = adjust_checks + 1;
                if (active_task[0]) begin
                    compare_node("OUTER_ADJUST_T0", {active_task[7:4], active_task[59:50]},
                        4'd0, 10'd0, 1'b0);
                end else begin
                    compare_node("ADJUST_LEFT", {active_task[7:4], active_task[59:50]},
                        active_task[63:60] + 1'b1, active_task[59:50] << 1, active_bank);
                end
            end
        end
    end

    initial begin
        clk = 1'b0;
        rst_n = 1'b0;
        ts_start = 1'b0;
        errors = 0;
        merge_checks = 0;
        adjust_checks = 0;
        first_bad_seen = 0;
        verbose_pass = $test$plusargs("VERBOSE_PASS");
        stop_first = $test$plusargs("STOP_FIRST");
        sample_count = 0;

        for (i = 0; i < 16384; i = i + 1) begin
            mem[i] = 256'd0;
        end

        $readmemh("t0_target.hex", t0_initial);
        $readmemh("t1_target.hex", t1_initial);
        $readmemh("ffid_t0_nodes.hex", exp_t0_nodes);
        $readmemh("ffid_t1_nodes.hex", exp_t1_nodes);

        for (i = 0; i < WORDS; i = i + 1) begin
            mem[0 + i] = t0_initial[i];
            mem[512 + i] = t1_initial[i];
        end
        $readmemh("tree_full_poly.hex", mem, TREE_BASE);

        repeat (8) @(posedge clk);
        rst_n = 1'b1;
        repeat (8) @(posedge clk);

        @(posedge clk);
        ts_start <= 1'b1;
        @(posedge clk);
        ts_start <= 1'b0;

        wait (ts_done || ts_fail);
        repeat (4) @(posedge clk);

        if (ts_fail) begin
            $display("TB_FAIL scheduler status=%02x", ts_status);
            errors = errors + 1;
        end

        compare_full_final(14'd4352, 1'b1);
        compare_full_final(14'd3840, 1'b0);

        $display("SUMMARY merge_checks=%0d adjust_checks=%0d sample_count=%0d errors=%0d",
            merge_checks, adjust_checks, sample_count, errors);
        if (errors == 0) begin
            $display("TB_PASS tb_ffsampling_layer_compare");
        end else begin
            $display("TB_FAIL tb_ffsampling_layer_compare");
        end
        $finish;
    end
endmodule
