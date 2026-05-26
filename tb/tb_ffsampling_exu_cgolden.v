`timescale 1ns/1ps

module tb_ffsampling_exu_cgolden;
    localparam ADDR_W = 11;
    localparam OP_READ_L10 = 4'd0;
    localparam OP_SPLIT    = 4'd1;
    localparam OP_ADJUST   = 4'd2;
    localparam OP_MERGE    = 4'd4;

    localparam SPLIT_SRC = 14'd0;
    localparam SPLIT_DST = 14'd512;
    localparam MERGE_SRC = 14'd0;
    localparam MERGE_DST = 14'd768;
    localparam ADJ_T0    = 14'd0;
    localparam ADJ_T1    = 14'd128;
    localparam ADJ_Z1    = 14'd256;
    localparam ADJ_L10   = 14'd384;

    reg clk;
    reg rst_n;
    reg task_valid;
    wire task_ready;
    reg [67:0] task_word;
    wire task_done;
    wire task_fail;
    wire [7:0] task_status;

    wire mem_rd_en;
    wire [ADDR_W-1:0] mem_rd_addr;
    reg [255:0] mem_rd_data;
    wire mem_wr_en;
    wire [ADDR_W-1:0] mem_wr_addr;
    wire [255:0] mem_wr_data;

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
    wire [63:0] sz_cmd_mu;
    wire [63:0] sz_cmd_sigma_inv;
    wire sz_cmd_pair;

    reg [255:0] mem [0:2047];
    reg [255:0] split_in [0:255];
    reg [255:0] split_exp [0:255];
    reg [255:0] merge_in [0:255];
    reg [255:0] merge_exp [0:511];
    reg [255:0] adj_t0 [0:127];
    reg [255:0] adj_t1 [0:127];
    reg [255:0] adj_z1 [0:127];
    reg [255:0] adj_l10 [0:127];
    reg [255:0] adj_exp [0:127];
    integer errors;
    integer mismatches;
    integer i;

    falcon_f64_ffsampling_exu #(.ADDR_W(ADDR_W)) dut (
        .clk(clk),
        .rst_n(rst_n),
        .task_valid(task_valid),
        .task_ready(task_ready),
        .task_word(task_word),
        .task_done(task_done),
        .task_fail(task_fail),
        .task_status(task_status),
        .mem_rd_en(mem_rd_en),
        .mem_rd_addr(mem_rd_addr),
        .mem_rd_data(mem_rd_data),
        .mem_wr_en(mem_wr_en),
        .mem_wr_addr(mem_wr_addr),
        .mem_wr_data(mem_wr_data),
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
        .sz_cmd_ready(1'b0),
        .sz_cmd_mu(sz_cmd_mu),
        .sz_cmd_sigma_inv(sz_cmd_sigma_inv),
        .sz_cmd_pair(sz_cmd_pair),
        .sz_rsp_valid(1'b0),
        .sz_rsp_z0(64'd0),
        .sz_rsp_z1(64'd0)
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

    always #5 clk = ~clk;

    always @(*) begin
        mem_rd_data = mem[mem_rd_addr];
    end

    always @(posedge clk) begin
        if (mem_wr_en) begin
            mem[mem_wr_addr] <= mem_wr_data;
        end
    end

    function [67:0] pack_task;
        input [3:0] opcode;
        input [3:0] level;
        input [13:0] src0;
        input [13:0] src1;
        input [13:0] dst;
        begin
            pack_task = 68'd0;
            pack_task[67:64] = opcode;
            pack_task[63:60] = level;
            pack_task[49:36] = src0;
            pack_task[35:22] = src1;
            pack_task[21:8]  = dst;
        end
    endfunction

    function [67:0] pack_adjust_task;
        input [3:0] level;
        input [13:0] t0_dst;
        input [13:0] t1_src;
        input [13:0] l10_src;
        input [13:0] z1_src;
        begin
            pack_adjust_task = pack_task(OP_ADJUST, level, t1_src, l10_src, z1_src);
            pack_adjust_task[59:50] = t0_dst[9:0];
            pack_adjust_task[7:0]   = {t0_dst[13:10], 4'h0};
        end
    endfunction

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
            word_match = (canon(got[63:0]) == canon(exp[63:0]))
                      && (canon(got[127:64]) == canon(exp[127:64]));
        end
    endfunction

    task issue_task;
        input [67:0] word;
        integer timeout;
        begin
            @(posedge clk);
            while (!task_ready) @(posedge clk);
            task_word  <= word;
            task_valid <= 1'b1;
            @(posedge clk);
            task_valid <= 1'b0;

            timeout = 0;
            while (!task_done && !task_fail && timeout < 200000) begin
                timeout = timeout + 1;
                @(posedge clk);
            end
            if (timeout >= 200000) begin
                $display("TB_ERROR timeout opcode=%0d", word[67:64]);
                errors = errors + 1;
            end
            if (task_fail) begin
                $display("TB_ERROR task_fail opcode=%0d status=%02x", word[67:64], task_status);
                errors = errors + 1;
            end
            repeat (2) @(posedge clk);
        end
    endtask

    task compare_region;
        input [8*16-1:0] label;
        input [13:0] base;
        input integer count;
        input integer exp_sel;
        integer k;
        reg [255:0] exp_word;
        begin
            mismatches = 0;
            for (k = 0; k < count; k = k + 1) begin
                case (exp_sel)
                    0: exp_word = split_exp[k];
                    1: exp_word = merge_exp[k];
                    default: exp_word = adj_exp[k];
                endcase
                if (!word_match(mem[base + k], exp_word)) begin
                    if (mismatches < 12) begin
                        $display("TB_MISMATCH %0s[%0d] got=%064x exp=%064x",
                                 label, k, mem[base + k], exp_word);
                    end
                    mismatches = mismatches + 1;
                end
            end
            if (mismatches == 0) begin
                $display("TB_CHECK %0s PASS count=%0d", label, count);
            end else begin
                $display("TB_CHECK %0s FAIL mismatches=%0d/%0d", label, mismatches, count);
                errors = errors + mismatches;
            end
        end
    endtask

    initial begin
        clk = 1'b0;
        rst_n = 1'b0;
        task_valid = 1'b0;
        task_word = 68'd0;
        errors = 0;
        mismatches = 0;
        for (i = 0; i < 2048; i = i + 1) begin
            mem[i] = 256'd0;
        end

        $readmemh("ffexu_split_in.hex", split_in);
        $readmemh("ffexu_split_exp.hex", split_exp);
        $readmemh("ffexu_merge_in.hex", merge_in);
        $readmemh("ffexu_merge_exp.hex", merge_exp);
        $readmemh("ffexu_adjust_t0.hex", adj_t0);
        $readmemh("ffexu_adjust_t1.hex", adj_t1);
        $readmemh("ffexu_adjust_z1.hex", adj_z1);
        $readmemh("ffexu_adjust_l10.hex", adj_l10);
        $readmemh("ffexu_adjust_exp.hex", adj_exp);

        repeat (5) @(posedge clk);
        rst_n = 1'b1;
        repeat (5) @(posedge clk);

        for (i = 0; i < 256; i = i + 1) begin
            mem[SPLIT_SRC + i] = split_in[i];
        end
        issue_task(pack_task(OP_SPLIT, 4'd0, SPLIT_SRC, 14'd0, SPLIT_DST));
        compare_region("SPLIT", SPLIT_DST, 256, 0);

        for (i = 0; i < 256; i = i + 1) begin
            mem[MERGE_SRC + i] = merge_in[i];
        end
        issue_task(pack_task(OP_MERGE, 4'd0, MERGE_SRC, 14'd0, MERGE_DST));
        compare_region("MERGE", MERGE_DST, 512, 1);

        for (i = 0; i < 128; i = i + 1) begin
            mem[ADJ_T0 + i]  = adj_t0[i];
            mem[ADJ_T1 + i]  = adj_t1[i];
            mem[ADJ_Z1 + i]  = adj_z1[i];
            mem[ADJ_L10 + i] = adj_l10[i];
        end
        issue_task(pack_adjust_task(4'd0, ADJ_T0, ADJ_T1, ADJ_L10, ADJ_Z1));
        compare_region("ADJUST", ADJ_T0, 128, 2);

        if (errors == 0) begin
            $display("TB_PASS tb_ffsampling_exu_cgolden");
        end else begin
            $display("TB_FAIL tb_ffsampling_exu_cgolden errors=%0d", errors);
        end
        $finish;
    end
endmodule
