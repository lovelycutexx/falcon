`timescale 1ns/1ps

module tb_falcon_f64_split_merge_identity;
    localparam ADDR_W = 14;
    localparam OP_SPLIT = 4'd1;
    localparam OP_MERGE = 4'd4;
    localparam SRC_BASE = 14'd0;
    localparam OUT_BASE = 14'd600;

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
    reg [63:0] gm_re [0:255];
    reg [63:0] gm_im [0:255];
    wire [63:0] twiddle_re = gm_re[twiddle_addr[7:0]];
    wire [63:0] twiddle_im = gm_im[twiddle_addr[7:0]];

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

    reg [255:0] mem [0:1023];
    reg [255:0] initial_word [0:511];
    integer i;
    integer mismatches;

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
        .sz_cmd_ready(1'b1),
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

    task issue_task;
        input [67:0] word;
        integer timeout;
        begin
            @(posedge clk);
            while (!task_ready) @(posedge clk);
            task_word <= word;
            task_valid <= 1'b1;
            @(posedge clk);
            task_valid <= 1'b0;

            timeout = 0;
            while (!task_done && !task_fail && timeout < 200000) begin
                timeout = timeout + 1;
                @(posedge clk);
            end

            if (timeout >= 200000) begin
                $display("TB_FAIL timeout");
                $finish;
            end
            if (task_fail) begin
                $display("TB_FAIL task_fail status=%02x", task_status);
                $finish;
            end
        end
    endtask

    initial begin
        clk = 1'b0;
        rst_n = 1'b0;
        task_valid = 1'b0;
        task_word = 68'd0;
        mismatches = 0;

        for (i = 0; i < 1024; i = i + 1) begin
            mem[i] = 256'd0;
        end

        $readmemh("t1_target.hex", mem, SRC_BASE, SRC_BASE + 511);
        for (i = 0; i < 512; i = i + 1) begin
            initial_word[i] = mem[SRC_BASE + i];
        end
        $readmemh("DOC/gm_rom_re.hex", gm_re);
        $readmemh("DOC/gm_rom_im.hex", gm_im);

        repeat (8) @(posedge clk);
        rst_n = 1'b1;
        repeat (4) @(posedge clk);

        issue_task(pack_task(OP_SPLIT, 4'd0, SRC_BASE, 14'd0, SRC_BASE));
        issue_task(pack_task(OP_MERGE, 4'd0, SRC_BASE, 14'd0, OUT_BASE));

        for (i = 0; i < 512; i = i + 1) begin
            if (mem[OUT_BASE + i] !== initial_word[i]) begin
                if (mismatches < 8) begin
                    $display("MISMATCH[%0d] got=%h exp=%h", i, mem[OUT_BASE + i], initial_word[i]);
                end
                mismatches = mismatches + 1;
            end
        end

        if (mismatches == 0) begin
            $display("TB_PASS falcon_f64_split_merge_identity");
        end else begin
            $display("TB_FAIL falcon_f64_split_merge_identity mismatches=%0d", mismatches);
        end
        $finish;
    end
endmodule
