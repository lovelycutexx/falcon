`timescale 1ns/1ps
module falcon_ffsampling_iter_ctrl #
(
    parameter LOGN_W     = 4,
    parameter NODE_W     = 11,
    parameter STACK_DEPTH = 12
)
(
    input                  clk,
    input                  rst_n,

    input                  start,
    output                 start_ready,
    input      [LOGN_W-1:0] cfg_logn,

    output reg             cmd_valid,
    input                  cmd_ready,
    output reg [3:0]       cmd_opcode,
    output reg [LOGN_W-1:0] cmd_level,
    output reg [NODE_W-1:0] cmd_node,

    input                  rsp_valid,
    input                  rsp_fail,
    input      [7:0]       rsp_status,

    output reg             busy,
    output reg             done,
    output reg             fail,
    output reg [7:0]       status
);

    localparam [3:0] CMD_READ_L10    = 4'd0;
    localparam [3:0] CMD_SPLIT_T1    = 4'd1;
    localparam [3:0] CMD_ADJUST_T0   = 4'd2;
    localparam [3:0] CMD_SAMPLE_LEAF = 4'd3;
    localparam [3:0] CMD_MERGE_Z     = 4'd4;

    localparam [2:0] PH_ENTER       = 3'd0;
    localparam [2:0] PH_SPLIT       = 3'd1;
    localparam [2:0] PH_AFTER_RIGHT = 3'd2;
    localparam [2:0] PH_AFTER_LEFT  = 3'd3;

    localparam [2:0] ST_IDLE       = 3'd0;
    localparam [2:0] ST_ISSUE      = 3'd1;
    localparam [2:0] ST_WAIT       = 3'd2;
    localparam [2:0] ST_DONE       = 3'd3;
    localparam [2:0] ST_FAIL       = 3'd4;

    reg [2:0]        state;
    reg [2:0]        next_phase;
    reg              pop_after_rsp;
    reg              push_after_rsp;
    reg [LOGN_W-1:0] push_level_q;
    reg [NODE_W-1:0] push_node_q;
    reg [2:0]        push_phase_q;
    reg [2:0]        rsp_next_phase_q;
    reg              rsp_pop_q;
    reg              rsp_push_q;
    reg [LOGN_W-1:0] rsp_push_level_q;
    reg [NODE_W-1:0] rsp_push_node_q;
    reg [2:0]        rsp_push_phase_q;
    reg [7:0]        fail_status_q;

    reg [LOGN_W-1:0] stack_level [0:STACK_DEPTH-1];
    reg [NODE_W-1:0] stack_node  [0:STACK_DEPTH-1];
    reg [2:0]        stack_phase [0:STACK_DEPTH-1];
    reg [3:0]        sp;

    wire             stack_empty;
    wire             stack_full;
    wire [LOGN_W-1:0] top_level;
    wire [NODE_W-1:0] top_node;
    wire [2:0]       top_phase;
    wire [NODE_W-1:0] left_child;
    wire [NODE_W-1:0] right_child;

    assign stack_empty = (sp == 4'd0);
    assign stack_full  = (sp >= STACK_DEPTH[3:0]);
    assign top_level   = stack_level[sp - 1'b1];
    assign top_node    = stack_node[sp - 1'b1];
    assign top_phase   = stack_phase[sp - 1'b1];
    assign left_child  = top_node << 1;
    assign right_child = (top_node << 1) + 1'b1;
    assign start_ready = (state == ST_IDLE);

    integer idx;

    always @(*) begin
        cmd_valid      = 1'b0;
        cmd_opcode     = CMD_READ_L10;
        cmd_level      = {LOGN_W{1'b0}};
        cmd_node       = {NODE_W{1'b0}};
        next_phase     = PH_ENTER;
        pop_after_rsp  = 1'b0;
        push_after_rsp = 1'b0;
        push_level_q   = {LOGN_W{1'b0}};
        push_node_q    = {NODE_W{1'b0}};
        push_phase_q   = PH_ENTER;

        if ((state == ST_ISSUE) && !stack_empty) begin
            cmd_valid = 1'b1;
            cmd_level = top_level;
            cmd_node  = top_node;

            case (top_phase)
                PH_ENTER: begin
                    if (top_level == {LOGN_W{1'b0}}) begin
                        cmd_opcode    = CMD_SAMPLE_LEAF;
                        pop_after_rsp = 1'b1;
                    end else begin
                        cmd_opcode = CMD_READ_L10;
                        next_phase = PH_SPLIT;
                    end
                end

                PH_SPLIT: begin
                    cmd_opcode     = CMD_SPLIT_T1;
                    next_phase     = PH_AFTER_RIGHT;
                    push_after_rsp = 1'b1;
                    push_level_q   = top_level - 1'b1;
                    push_node_q    = right_child;
                    push_phase_q   = PH_ENTER;
                end

                PH_AFTER_RIGHT: begin
                    cmd_opcode = CMD_ADJUST_T0;
                    next_phase = PH_AFTER_LEFT;
                    push_after_rsp = 1'b1;
                    push_level_q   = top_level - 1'b1;
                    push_node_q    = left_child;
                    push_phase_q   = PH_ENTER;
                end

                PH_AFTER_LEFT: begin
                    cmd_opcode    = CMD_MERGE_Z;
                    pop_after_rsp = 1'b1;
                end

                default: begin
                    cmd_opcode    = CMD_SAMPLE_LEAF;
                    pop_after_rsp = 1'b1;
                end
            endcase
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state         <= ST_IDLE;
            sp            <= 4'd0;
            busy          <= 1'b0;
            done          <= 1'b0;
            fail          <= 1'b0;
            status        <= 8'h00;
            fail_status_q <= 8'h00;
            rsp_next_phase_q <= PH_ENTER;
            rsp_pop_q        <= 1'b0;
            rsp_push_q       <= 1'b0;
            rsp_push_level_q <= {LOGN_W{1'b0}};
            rsp_push_node_q  <= {NODE_W{1'b0}};
            rsp_push_phase_q <= PH_ENTER;
            for (idx = 0; idx < STACK_DEPTH; idx = idx + 1) begin
                stack_level[idx] <= {LOGN_W{1'b0}};
                stack_node[idx]  <= {NODE_W{1'b0}};
                stack_phase[idx] <= PH_ENTER;
            end
        end else begin
            done <= 1'b0;
            fail <= 1'b0;

            case (state)
                ST_IDLE: begin
                    busy   <= 1'b0;
                    status <= 8'h00;
                    if (start) begin
                        if ((cfg_logn == {LOGN_W{1'b0}}) || (STACK_DEPTH < 2)) begin
                            fail_status_q <= 8'hE1;
                            state         <= ST_FAIL;
                            busy          <= 1'b1;
                        end else begin
                            stack_level[0] <= cfg_logn;
                            stack_node[0]  <= {NODE_W{1'b0}};
                            stack_phase[0] <= PH_ENTER;
                            sp             <= 4'd1;
                            busy           <= 1'b1;
                            state          <= ST_ISSUE;
                        end
                    end
                end

                ST_ISSUE: begin
                    if (stack_empty) begin
                        state <= ST_DONE;
                    end else if (cmd_valid && cmd_ready) begin
                        rsp_next_phase_q <= next_phase;
                        rsp_pop_q        <= pop_after_rsp;
                        rsp_push_q       <= push_after_rsp;
                        rsp_push_level_q <= push_level_q;
                        rsp_push_node_q  <= push_node_q;
                        rsp_push_phase_q <= push_phase_q;
                        state <= ST_WAIT;
                    end
                end

                ST_WAIT: begin
                    if (rsp_valid) begin
                        if (rsp_fail) begin
                            fail_status_q <= rsp_status;
                            state         <= ST_FAIL;
                        end else begin
                            if (rsp_pop_q) begin
                                sp <= sp - 1'b1;
                                state <= ST_ISSUE;
                            end else begin
                                stack_phase[sp - 1'b1] <= rsp_next_phase_q;
                                if (rsp_push_q) begin
                                    if (stack_full) begin
                                        fail_status_q <= 8'hE2;
                                        state         <= ST_FAIL;
                                    end else begin
                                        stack_level[sp] <= rsp_push_level_q;
                                        stack_node[sp]  <= rsp_push_node_q;
                                        stack_phase[sp] <= rsp_push_phase_q;
                                        sp              <= sp + 1'b1;
                                        state           <= ST_ISSUE;
                                    end
                                end else begin
                                    state <= ST_ISSUE;
                                end
                            end
                        end
                    end
                end

                ST_DONE: begin
                    busy   <= 1'b0;
                    done   <= 1'b1;
                    status <= 8'h00;
                    state  <= ST_IDLE;
                end

                ST_FAIL: begin
                    busy   <= 1'b0;
                    fail   <= 1'b1;
                    status <= fail_status_q;
                    sp     <= 4'd0;
                    state  <= ST_IDLE;
                end

                default: begin
                    busy  <= 1'b0;
                    state <= ST_IDLE;
                end
            endcase
        end
    end

endmodule
