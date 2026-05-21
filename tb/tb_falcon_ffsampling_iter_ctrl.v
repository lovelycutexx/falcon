`timescale 1ns/1ps

module tb_falcon_ffsampling_iter_ctrl;

    localparam LOGN_W = 4;
    localparam NODE_W = 11;

    reg                  clk;
    reg                  rst_n;
    reg                  start;
    wire                 start_ready;
    reg  [LOGN_W-1:0]    cfg_logn;
    wire                 cmd_valid;
    reg                  cmd_ready;
    wire [3:0]           cmd_opcode;
    wire [LOGN_W-1:0]    cmd_level;
    wire [NODE_W-1:0]    cmd_node;
    reg                  rsp_valid;
    reg                  rsp_fail;
    reg  [7:0]           rsp_status;
    wire                 busy;
    wire                 done;
    wire                 fail;
    wire [7:0]           status;

    integer              errors;
    integer              cmd_count;
    integer              timeout;
    reg [3:0]            exp_opcode [0:31];
    reg [LOGN_W-1:0]     exp_level  [0:31];
    reg [NODE_W-1:0]     exp_node   [0:31];

    falcon_ffsampling_iter_ctrl #
    (
        .LOGN_W      (LOGN_W),
        .NODE_W      (NODE_W),
        .STACK_DEPTH (12)
    )
    dut (
        .clk         (clk),
        .rst_n       (rst_n),
        .start       (start),
        .start_ready (start_ready),
        .cfg_logn    (cfg_logn),
        .cmd_valid   (cmd_valid),
        .cmd_ready   (cmd_ready),
        .cmd_opcode  (cmd_opcode),
        .cmd_level   (cmd_level),
        .cmd_node    (cmd_node),
        .rsp_valid   (rsp_valid),
        .rsp_fail    (rsp_fail),
        .rsp_status  (rsp_status),
        .busy        (busy),
        .done        (done),
        .fail        (fail),
        .status      (status)
    );

    always #5 clk = ~clk;

    task expect_cmd;
        input [3:0]        opcode;
        input [LOGN_W-1:0] level;
        input [NODE_W-1:0] node;
        begin
            exp_opcode[cmd_count] = opcode;
            exp_level[cmd_count]  = level;
            exp_node[cmd_count]   = node;
            cmd_count             = cmd_count + 1;
        end
    endtask

    task drive_rsp_ok;
        begin
            @(negedge clk);
            rsp_valid  = 1'b1;
            rsp_fail   = 1'b0;
            rsp_status = 8'h00;
            @(negedge clk);
            rsp_valid  = 1'b0;
        end
    endtask

    initial begin
        clk        = 1'b0;
        rst_n      = 1'b0;
        start      = 1'b0;
        cfg_logn   = 4'd0;
        cmd_ready  = 1'b1;
        rsp_valid  = 1'b0;
        rsp_fail   = 1'b0;
        rsp_status = 8'h00;
        errors     = 0;
        cmd_count  = 0;
        timeout    = 0;

        expect_cmd(4'd0, 4'd2, 11'd0);
        expect_cmd(4'd1, 4'd2, 11'd0);
        expect_cmd(4'd0, 4'd1, 11'd1);
        expect_cmd(4'd1, 4'd1, 11'd1);
        expect_cmd(4'd3, 4'd0, 11'd3);
        expect_cmd(4'd2, 4'd1, 11'd1);
        expect_cmd(4'd3, 4'd0, 11'd2);
        expect_cmd(4'd4, 4'd1, 11'd1);
        expect_cmd(4'd2, 4'd2, 11'd0);
        expect_cmd(4'd0, 4'd1, 11'd0);
        expect_cmd(4'd1, 4'd1, 11'd0);
        expect_cmd(4'd3, 4'd0, 11'd1);
        expect_cmd(4'd2, 4'd1, 11'd0);
        expect_cmd(4'd3, 4'd0, 11'd0);
        expect_cmd(4'd4, 4'd1, 11'd0);
        expect_cmd(4'd4, 4'd2, 11'd0);

        repeat (4) @(posedge clk);
        rst_n = 1'b1;
        repeat (2) @(posedge clk);

        @(negedge clk);
        cfg_logn = 4'd2;
        start    = 1'b1;
        @(negedge clk);
        start    = 1'b0;

        cmd_count = 0;
        while (!done && !fail && timeout < 200) begin
            @(posedge clk);
            timeout = timeout + 1;
            if (cmd_valid && cmd_ready) begin
                if (cmd_opcode !== exp_opcode[cmd_count]) begin
                    $display("TB_ERROR opcode at %0d got %0d expected %0d", cmd_count, cmd_opcode, exp_opcode[cmd_count]);
                    errors = errors + 1;
                end
                if (cmd_level !== exp_level[cmd_count]) begin
                    $display("TB_ERROR level at %0d got %0d expected %0d", cmd_count, cmd_level, exp_level[cmd_count]);
                    errors = errors + 1;
                end
                if (cmd_node !== exp_node[cmd_count]) begin
                    $display("TB_ERROR node at %0d got %0d expected %0d", cmd_count, cmd_node, exp_node[cmd_count]);
                    errors = errors + 1;
                end
                drive_rsp_ok;
                cmd_count = cmd_count + 1;
            end
        end

        if (timeout >= 200) begin
            $display("TB_ERROR timeout");
            errors = errors + 1;
        end
        if (fail) begin
            $display("TB_ERROR unexpected fail status=%02x", status);
            errors = errors + 1;
        end
        if (cmd_count != 16) begin
            $display("TB_ERROR command count got %0d expected 16", cmd_count);
            errors = errors + 1;
        end

        if (errors == 0) begin
            $display("TB_PASS falcon_ffsampling_iter_ctrl");
        end else begin
            $display("TB_FAIL falcon_ffsampling_iter_ctrl errors=%0d", errors);
        end
        $finish;
    end

endmodule
