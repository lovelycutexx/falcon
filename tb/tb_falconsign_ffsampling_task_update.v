`timescale 1ns/1ps

module tb_falconsign_ffsampling_task_update;

    reg          clk;
    reg          rst_n;
    reg          start;
    wire         start_ready;
    reg  [3:0]   cfg_depth;
    reg          cfg_dynamic_tree;
    reg  [13:0]  cfg_t_base;
    reg  [13:0]  cfg_tree_base;
    reg  [13:0]  cfg_z_base;
    reg  [13:0]  cfg_tmp_base;
    wire         task_valid;
    reg          task_ready;
    wire [67:0]  task_word;
    reg          task_done;
    reg          task_fail;
    reg  [7:0]   task_status;
    wire         busy;
    wire         done;
    wire         fail;
    wire [7:0]   status;
    wire [3:0]   dbg_level;
    wire [9:0]   dbg_index;
    wire [1:0]   dbg_state;

    integer      errors;
    integer      task_count;
    integer      timeout;
    reg [3:0]    exp_opcode [0:31];
    reg [3:0]    exp_level  [0:31];
    reg [9:0]    exp_index  [0:31];
    reg [13:0]   exp_src0   [0:31];
    reg [13:0]   exp_src1   [0:31];
    reg [13:0]   exp_dst    [0:31];

    falconsign_ffsampling_task_update dut (
        .clk              (clk),
        .rst_n            (rst_n),
        .start            (start),
        .start_ready      (start_ready),
        .cfg_depth        (cfg_depth),
        .cfg_dynamic_tree (cfg_dynamic_tree),
        .cfg_t_base       (cfg_t_base),
        .cfg_tree_base    (cfg_tree_base),
        .cfg_z_base       (cfg_z_base),
        .cfg_tmp_base     (cfg_tmp_base),
        .task_valid       (task_valid),
        .task_ready       (task_ready),
        .task_word        (task_word),
        .task_done        (task_done),
        .task_fail        (task_fail),
        .task_status      (task_status),
        .busy             (busy),
        .done             (done),
        .fail             (fail),
        .status           (status),
        .dbg_level        (dbg_level),
        .dbg_index        (dbg_index),
        .dbg_state        (dbg_state)
    );

    always #5 clk = ~clk;

    task add_expected;
        input [3:0] opcode;
        input [3:0] level;
        input [9:0] index;
        input [13:0] src0;
        input [13:0] src1;
        input [13:0] dst;
        begin
            exp_opcode[task_count] = opcode;
            exp_level[task_count]  = level;
            exp_index[task_count]  = index;
            exp_src0[task_count]   = src0;
            exp_src1[task_count]   = src1;
            exp_dst[task_count]    = dst;
            task_count             = task_count + 1;
        end
    endtask

    task respond_ok;
        begin
            @(negedge clk);
            task_done   = 1'b1;
            task_fail   = 1'b0;
            task_status = 8'h00;
            @(negedge clk);
            task_done   = 1'b0;
        end
    endtask

    initial begin
        clk              = 1'b0;
        rst_n            = 1'b0;
        start            = 1'b0;
        cfg_depth        = 4'd2;
        cfg_dynamic_tree = 1'b0;
        cfg_t_base       = 14'd100;
        cfg_tree_base    = 14'd300;
        cfg_z_base       = 14'd700;
        cfg_tmp_base     = 14'd800;
        task_ready       = 1'b1;
        task_done        = 1'b0;
        task_fail        = 1'b0;
        task_status      = 8'h00;
        errors           = 0;
        task_count       = 0;
        timeout          = 0;

        add_expected(4'd0, 4'd0, 10'd0, 14'd104, 14'd300, 14'd104);
        add_expected(4'd1, 4'd0, 10'd0, 14'd104, 14'd300, 14'd104);
        add_expected(4'd0, 4'd1, 10'd1, 14'd105, 14'd684, 14'd104);
        add_expected(4'd1, 4'd1, 10'd1, 14'd105, 14'd684, 14'd104);
        add_expected(4'd3, 4'd2, 10'd3, 14'd105, 14'd1004, 14'd705);
        add_expected(4'd2, 4'd1, 10'd1, 14'd105, 14'd684, 14'd705);
        add_expected(4'd3, 4'd2, 10'd2, 14'd104, 14'd940, 14'd704);
        add_expected(4'd4, 4'd1, 10'd1, 14'd704, 14'd684, 14'd704);
        add_expected(4'd4, 4'd0, 10'd0, 14'd704, 14'd300, 14'd704);
        add_expected(4'd2, 4'd0, 10'd0, 14'd104, 14'd300, 14'd704);
        add_expected(4'd0, 4'd1, 10'd0, 14'd101, 14'd556, 14'd100);
        add_expected(4'd1, 4'd1, 10'd0, 14'd101, 14'd556, 14'd100);
        add_expected(4'd3, 4'd2, 10'd1, 14'd101, 14'd876, 14'd701);
        add_expected(4'd2, 4'd1, 10'd0, 14'd101, 14'd556, 14'd701);
        add_expected(4'd3, 4'd2, 10'd0, 14'd100, 14'd812, 14'd700);
        add_expected(4'd4, 4'd1, 10'd0, 14'd700, 14'd556, 14'd700);
        add_expected(4'd4, 4'd0, 10'd0, 14'd700, 14'd300, 14'd700);

        repeat (4) @(posedge clk);
        rst_n = 1'b1;
        repeat (2) @(posedge clk);
        @(negedge clk);
        start = 1'b1;
        @(negedge clk);
        start = 1'b0;

        task_count = 0;
        while (!done && !fail && timeout < 300) begin
            @(posedge clk);
            timeout = timeout + 1;
            if (task_valid && task_ready) begin
                if (task_word[67:64] !== exp_opcode[task_count]) begin
                    $display("TB_ERROR opcode %0d got %0d expected %0d", task_count, task_word[67:64], exp_opcode[task_count]);
                    errors = errors + 1;
                end
                if (task_word[63:60] !== exp_level[task_count]) begin
                    $display("TB_ERROR level %0d got %0d expected %0d", task_count, task_word[63:60], exp_level[task_count]);
                    errors = errors + 1;
                end
                if ((task_word[67:64] !== 4'd2) && (task_word[59:50] !== exp_index[task_count])) begin
                    $display("TB_ERROR index %0d got %0d expected %0d", task_count, task_word[59:50], exp_index[task_count]);
                    errors = errors + 1;
                end
                if (task_word[49:36] !== exp_src0[task_count]) begin
                    $display("TB_ERROR src0 %0d got %0d expected %0d", task_count, task_word[49:36], exp_src0[task_count]);
                    errors = errors + 1;
                end
                if (task_word[35:22] !== exp_src1[task_count]) begin
                    $display("TB_ERROR src1 %0d got %0d expected %0d", task_count, task_word[35:22], exp_src1[task_count]);
                    errors = errors + 1;
                end
                if (task_word[21:8] !== exp_dst[task_count]) begin
                    $display("TB_ERROR dst %0d got %0d expected %0d", task_count, task_word[21:8], exp_dst[task_count]);
                    errors = errors + 1;
                end
                respond_ok;
                task_count = task_count + 1;
            end
        end

        if (timeout >= 300) begin
            $display("TB_ERROR timeout");
            errors = errors + 1;
        end
        if (fail) begin
            $display("TB_ERROR unexpected fail status=%02x", status);
            errors = errors + 1;
        end
        if (task_count != 17) begin
            $display("TB_ERROR task count got %0d expected 17", task_count);
            errors = errors + 1;
        end

        if (errors == 0) begin
            $display("TB_PASS falconsign_ffsampling_task_update");
        end else begin
            $display("TB_FAIL falconsign_ffsampling_task_update errors=%0d", errors);
        end
        $finish;
    end

endmodule
