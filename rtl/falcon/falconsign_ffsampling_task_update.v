`timescale 1ns/1ps
module falconsign_ffsampling_task_update #
(
    parameter LEVEL_W = 4,
    parameter INDEX_W = 10,
    parameter ADDR_W  = 14
)
(
    input                  clk,
    input                  rst_n,

    input                  start,
    output                 start_ready,
    input      [LEVEL_W-1:0] cfg_depth,
    input                  cfg_dynamic_tree,
    input      [ADDR_W-1:0] cfg_t_base,
    input      [ADDR_W-1:0] cfg_tree_base,
    input      [ADDR_W-1:0] cfg_z_base,

    output reg             task_valid,
    input                  task_ready,
    output reg [67:0]      task_word,

    input                  task_done,
    input                  task_fail,
    input      [7:0]       task_status,

    output reg             busy,
    output reg             done,
    output reg             fail,
    output reg [7:0]       status,

    output reg [LEVEL_W-1:0] dbg_level,
    output reg [INDEX_W-1:0] dbg_index,
    output reg [1:0]       dbg_state
);

    localparam [3:0] OP_READ_L10     = 4'd0;
    localparam [3:0] OP_SPLIT_T1     = 4'd1;
    localparam [3:0] OP_ADJUST_T0    = 4'd2;
    localparam [3:0] OP_SAMPLE_PAIR  = 4'd3;
    localparam [3:0] OP_MERGE_Z      = 4'd4;
    localparam [3:0] OP_DYNAMIC_LDL  = 4'd5;

    localparam [1:0] ST_RIGHT_DOWN = 2'd0;
    localparam [1:0] ST_RIGHT_UP   = 2'd1;
    localparam [1:0] ST_LEFT_DOWN  = 2'd2;
    localparam [1:0] ST_LEFT_UP    = 2'd3;

    localparam [1:0] SUB_READ  = 2'd0;
    localparam [1:0] SUB_SPLIT = 2'd1;
    localparam [1:0] SUB_LDL   = 2'd2;

    localparam [2:0] RUN_IDLE  = 3'd0;
    localparam [2:0] RUN_EMIT  = 3'd1;
    localparam [2:0] RUN_WAIT  = 3'd2;
    localparam [2:0] RUN_DONE  = 3'd3;
    localparam [2:0] RUN_FAIL  = 3'd4;

    reg [2:0]         run_state;
    reg [LEVEL_W-1:0] level_q;
    reg [INDEX_W-1:0] index_q;
    reg [1:0]         state_q;
    reg [1:0]         sub_q;
    reg               bank_q;
    reg               root_z1_merged_q;
    reg [3:0]         issued_op_q;
    reg [7:0]         fail_status_q;
`ifndef SYNTHESIS
    reg               debug_trace_tasks;
    initial debug_trace_tasks = $test$plusargs("FS_TRACE_TASKS");
`endif

    wire              at_leaf;
    wire              at_root;
    wire              from_right_child;
    wire [LEVEL_W-1:0] child_level;
    wire [INDEX_W-1:0] left_child_index;
    wire [INDEX_W-1:0] right_child_index;
    wire [LEVEL_W-1:0] parent_level;
    wire [INDEX_W-1:0] parent_index;
    wire [ADDR_W-1:0]  src0_addr;
    wire [ADDR_W-1:0]  src1_addr;
    wire [ADDR_W-1:0]  dst_addr;

    assign start_ready       = (run_state == RUN_IDLE);
    assign at_leaf           = (level_q == cfg_depth);
    assign at_root           = ((level_q == {LEVEL_W{1'b0}}) && (index_q == {INDEX_W{1'b0}}));
    assign from_right_child  = index_q[0];
    assign child_level       = level_q + 1'b1;
    assign left_child_index  = index_q << 1;
    assign right_child_index = (index_q << 1) + 1'b1;
    assign parent_level      = level_q - 1'b1;
    assign parent_index      = index_q >> 1;

    // Segment-tree + {level,index} addressing.
    // t/z data: segment at idx << (depth - level)
    // tree data: 256-word buckets per recursion level. For an internal node
    // at level d from the root, its L10 polynomial starts at:
    //   d*256 + index*2^(8-d)
    // leaves start at 2304 + index.
    reg [ADDR_W-1:0] component_stride, component_words;
    reg [ADDR_W-1:0] seg_offset, node_words, pair_cnt_for_adj, tree_ofs;
    reg [INDEX_W-1:0] data_index;

    always @(*) begin
        // component_stride = pair_limit at the CURRENT level
        // (half the complex-pair count of this node).
        // This equals the distance between left/right children after SPLIT.
        case (cfg_depth - level_q)
            4'd9:  component_stride = {{(ADDR_W-9){1'b0}}, 9'd256};
            4'd8:  component_stride = {{(ADDR_W-8){1'b0}}, 8'd128};
            4'd7:  component_stride = {{(ADDR_W-7){1'b0}}, 7'd64};
            4'd6:  component_stride = {{(ADDR_W-6){1'b0}}, 6'd32};
            4'd5:  component_stride = {{(ADDR_W-5){1'b0}}, 5'd16};
            4'd4:  component_stride = {{(ADDR_W-4){1'b0}}, 4'd8};
            4'd3:  component_stride = {{(ADDR_W-3){1'b0}}, 3'd4};
            4'd2:  component_stride = {{(ADDR_W-2){1'b0}}, 2'd2};
            default: component_stride = {{(ADDR_W-1){1'b0}}, 1'd1};
        endcase
    end
    always @(*) begin
        component_words = {{(ADDR_W-1){1'b0}}, 1'b1} << cfg_depth;
    end

    always @(*) begin
        if (at_root) begin
            data_index = {INDEX_W{1'b0}};
        end else begin
            data_index = index_q;
        end
    end

    // Compute seg_offset by walking the path from root: for each RIGHT
    // turn at level lv, add pair_limit(lv).  This matches the SPLIT output
    // layout: left child reuses parent offset, right child = parent + pair_limit.
    function [ADDR_W-1:0] compute_seg_offset;
        input [INDEX_W-1:0] idx;
        input [3:0]          lv;
        reg [ADDR_W-1:0] off;
        reg [3:0] bitpos;
        begin
            off = {ADDR_W{1'b0}};
            for (bitpos = 0; bitpos < lv; bitpos = bitpos + 1) begin
                if (idx[lv - 1 - bitpos]) begin
                    case (bitpos[3:0])
                        4'd0: off = off + {{(ADDR_W-9){1'b0}}, 9'd256};
                        4'd1: off = off + {{(ADDR_W-8){1'b0}}, 8'd128};
                        4'd2: off = off + {{(ADDR_W-7){1'b0}}, 7'd64};
                        4'd3: off = off + {{(ADDR_W-6){1'b0}}, 6'd32};
                        4'd4: off = off + {{(ADDR_W-5){1'b0}}, 5'd16};
                        4'd5: off = off + {{(ADDR_W-4){1'b0}}, 4'd8};
                        4'd6: off = off + {{(ADDR_W-3){1'b0}}, 3'd4};
                        4'd7: off = off + {{(ADDR_W-2){1'b0}}, 2'd2};
                        default: off = off + {{(ADDR_W-1){1'b0}}, 1'd1};
                    endcase
                end
            end
            compute_seg_offset = off;
        end
    endfunction

    always @(*) begin
        if (at_root) begin
            seg_offset = {ADDR_W{1'b0}};
        end else begin
            // Both inner nodes and leaves use the same path-walking formula:
            // offset = sum of pair_limit(lv) for each right-turn ancestor.
            seg_offset = compute_seg_offset(index_q, level_q);
        end
    end
    always @(*) begin
        // Number of complex words in one component (t0 or t1) at this
        // recursive node. The node's t1 component starts at t_addr+node_words.
        case (cfg_depth - level_q)
            4'd9:  node_words = {{(ADDR_W-9){1'b0}}, 9'd256};
            4'd8:  node_words = {{(ADDR_W-8){1'b0}}, 8'd128};
            4'd7:  node_words = {{(ADDR_W-7){1'b0}}, 7'd64};
            4'd6:  node_words = {{(ADDR_W-6){1'b0}}, 6'd32};
            4'd5:  node_words = {{(ADDR_W-5){1'b0}}, 5'd16};
            4'd4:  node_words = {{(ADDR_W-4){1'b0}}, 4'd8};
            4'd3:  node_words = {{(ADDR_W-3){1'b0}}, 3'd4};
            4'd2:  node_words = {{(ADDR_W-2){1'b0}}, 2'd2};
            default: node_words = {{(ADDR_W-1){1'b0}}, 1'd1};
        endcase
    end

    always @(*) begin
        // pair_cnt_for_adj = offset to the right child after SPLIT at this level.
        // (Same as component_stride = word_count at this level.)
        // Root (L=0): 128.  Level 1: 64.  Level 2: 32 …
        // The actual ADJUST iteration count is controlled by the EXU's word_count,
        // doubled via aux[7] for the root full-polynomial transition.
        case (cfg_depth - level_q)
            4'd9:  pair_cnt_for_adj = {{(ADDR_W-9){1'b0}}, 9'd256};  // root
            4'd8:  pair_cnt_for_adj = {{(ADDR_W-8){1'b0}}, 8'd128};
            4'd7:  pair_cnt_for_adj = {{(ADDR_W-7){1'b0}}, 7'd64};
            4'd6:  pair_cnt_for_adj = {{(ADDR_W-6){1'b0}}, 6'd32};
            4'd5:  pair_cnt_for_adj = {{(ADDR_W-5){1'b0}}, 5'd16};
            4'd4:  pair_cnt_for_adj = {{(ADDR_W-4){1'b0}}, 4'd8};
            4'd3:  pair_cnt_for_adj = {{(ADDR_W-3){1'b0}}, 3'd4};
            4'd2:  pair_cnt_for_adj = {{(ADDR_W-2){1'b0}}, 2'd2};
            default: pair_cnt_for_adj = {{(ADDR_W-1){1'b0}}, 1'd1};
        endcase
    end
    always @(*) begin
        case (level_q)
            4'd0: tree_ofs = {ADDR_W{1'b0}} + (index_q << 8);
            4'd1: tree_ofs = {{(ADDR_W-9){1'b0}}, 9'd256} + (index_q << 7);
            4'd2: tree_ofs = {{(ADDR_W-10){1'b0}}, 10'd512} + (index_q << 6);
            4'd3: tree_ofs = {{(ADDR_W-10){1'b0}}, 10'd768} + (index_q << 5);
            4'd4: tree_ofs = {{(ADDR_W-11){1'b0}}, 11'd1024} + (index_q << 4);
            4'd5: tree_ofs = {{(ADDR_W-11){1'b0}}, 11'd1280} + (index_q << 3);
            4'd6: tree_ofs = {{(ADDR_W-11){1'b0}}, 11'd1536} + (index_q << 2);
            4'd7: tree_ofs = {{(ADDR_W-11){1'b0}}, 11'd1792} + (index_q << 1);
            4'd8: tree_ofs = {{(ADDR_W-12){1'b0}}, 12'd2048} + index_q;
            default: tree_ofs = {{(ADDR_W-12){1'b0}}, 12'd2304} + index_q;
        endcase
    end

    // Target t values live under cfg_t_base; sampled z values live under cfg_z_base.
    // bank_q selects the top-level component:
    //   1: run the whole traversal on t1/z1
    //   0: after root adjust, run the whole traversal on t0/z0
    // split:  t(parent) -> t(left/right)
    // sample: t(leaf), tree(leaf) -> z(leaf)
    // adjust: t(left) += (t(right) - z(right)) * l10
    // merge:  z(left/right) -> z(parent)
    wire [ADDR_W-1:0] t_component_base = cfg_t_base + (bank_q ? component_words : {ADDR_W{1'b0}});
    wire [ADDR_W-1:0] z_component_base = cfg_z_base + (bank_q ? component_words : {ADDR_W{1'b0}});
    // Root SPLIT reads from t_area; all children read from z_area
    // (where the parent SPLIT wrote their data).
    wire [ADDR_W-1:0] input_component_base = at_root ? t_component_base : z_component_base;
    wire [ADDR_W-1:0] input_addr = input_component_base + seg_offset;
    wire [ADDR_W-1:0] t_addr    = t_component_base + seg_offset;
    wire [ADDR_W-1:0] z_addr    = z_component_base + seg_offset;
    wire [ADDR_W-1:0] tree_addr = cfg_tree_base + tree_ofs;

    wire [ADDR_W-1:0] split_src0 = input_addr;
    wire [ADDR_W-1:0] split_src1 = tree_addr;
    wire [ADDR_W-1:0] split_dst  = z_addr;

    wire [ADDR_W-1:0] sample_src0 = input_addr;
    wire [ADDR_W-1:0] sample_src1 = tree_addr;
    wire [ADDR_W-1:0] sample_dst  = z_addr;

    wire [ADDR_W-1:0] merge_src0 = z_addr;
    wire [ADDR_W-1:0] merge_src1 = tree_addr;
    wire [ADDR_W-1:0] merge_dst  = z_addr;

    wire              outer_root_adjust = at_root && bank_q && root_z1_merged_q;
    // inner ADJUST: right child's t1/z1 live in z area at offset pair_cnt_for_adj
    // (SPLIT wrote right child to dst+pair_limit). Use z_addr, not input_addr,
    // to avoid corrupting original t data needed by outer_root_adjust.
    wire [ADDR_W-1:0] adj_src0 = outer_root_adjust ? (cfg_t_base + component_words)
                                                   : (z_addr + pair_cnt_for_adj);
    wire [ADDR_W-1:0] adj_src1 = outer_root_adjust ? (cfg_z_base + component_words)
                                                   : (z_addr + pair_cnt_for_adj);
    wire [ADDR_W-1:0] adj_dst  = outer_root_adjust ? cfg_t_base : z_addr;

    assign src0_addr = sample_src0;
    assign src1_addr = sample_src1;
    assign dst_addr  = sample_dst;

    always @(*) begin
        task_valid = 1'b0;
        task_word  = 68'd0;

        if (run_state == RUN_EMIT) begin
            task_valid = 1'b1;
            case (state_q)
                ST_RIGHT_DOWN: begin
                    if (at_leaf) begin
                        task_word = pack_task(OP_SAMPLE_PAIR, level_q, index_q, sample_src0, sample_src1, sample_dst, 8'h00);
                    end else if (cfg_dynamic_tree && (sub_q == SUB_LDL)) begin
                        task_word = pack_task(OP_DYNAMIC_LDL, level_q, index_q, src0_addr, src1_addr, dst_addr, 8'h00);
                    end else if (sub_q == SUB_READ) begin
                        task_word = pack_task(OP_READ_L10, level_q, index_q, split_src0, split_src1, split_dst, 8'h00);
                    end else begin
                        task_word = pack_task(OP_SPLIT_T1, level_q, index_q, split_src0, split_src1, split_dst, 8'h00);
                    end
                end

                ST_RIGHT_UP: begin
                    task_word = pack_adjust_task(level_q, adj_dst, adj_src0, tree_addr, adj_src1,
                                                 outer_root_adjust);
`ifndef SYNTHESIS
                    if (debug_trace_tasks && outer_root_adjust) begin
                        $display("  FS_OUTER_ROOT_ADJ bank=%0d root_z1=%0d: adj_dst=%0d adj_src0=%0d adj_src1=%0d",
                                 bank_q, root_z1_merged_q, adj_dst, adj_src0, adj_src1);
                    end else if (debug_trace_tasks) begin
                        $display("  FS_INNER_ADJ L=%0d I=%0d bank=%0d: adj_dst=%0d adj_src0=%0d adj_src1=%0d",
                                 level_q, index_q, bank_q, adj_dst, adj_src0, adj_src1);
                    end
`endif
                end

                ST_LEFT_DOWN: begin
                    if (at_leaf) begin
                        task_word = pack_task(OP_SAMPLE_PAIR, level_q, index_q, sample_src0, sample_src1, sample_dst, 8'h00);
                    end else if (cfg_dynamic_tree && (sub_q == SUB_LDL)) begin
                        task_word = pack_task(OP_DYNAMIC_LDL, level_q, index_q, src0_addr, src1_addr, dst_addr, 8'h00);
                    end else if (sub_q == SUB_READ) begin
                        task_word = pack_task(OP_READ_L10, level_q, index_q, split_src0, split_src1, split_dst, 8'h00);
                    end else begin
                        task_word = pack_task(OP_SPLIT_T1, level_q, index_q, split_src0, split_src1, split_dst, 8'h00);
                    end
                end

                ST_LEFT_UP: begin
                    task_word = pack_task(OP_MERGE_Z, level_q, index_q, merge_src0, merge_src1, merge_dst, 8'h00);
                end

                default: begin
                    task_word = pack_task(OP_SAMPLE_PAIR, level_q, index_q, sample_src0, sample_src1, sample_dst, 8'hFF);
                end
            endcase
        end
    end

    function [67:0] pack_task;
        input [3:0]          opcode;
        input [LEVEL_W-1:0]  level;
        input [INDEX_W-1:0]  index;
        input [ADDR_W-1:0]   src0;
        input [ADDR_W-1:0]   src1;
        input [ADDR_W-1:0]   dst;
        input [7:0]          aux;
        begin
            pack_task = 68'd0;
            pack_task[67:64] = opcode;
            pack_task[63:60] = level[3:0];
            pack_task[59:50] = index[9:0];
            pack_task[49:36] = {{(14-ADDR_W){1'b0}}, src0};
            pack_task[35:22] = {{(14-ADDR_W){1'b0}}, src1};
            pack_task[21:8]  = {{(14-ADDR_W){1'b0}}, dst};
            pack_task[7:0]   = aux;
        end
    endfunction

    function [67:0] pack_adjust_task;
        input [LEVEL_W-1:0]  level;
        input [ADDR_W-1:0]   t0_dst;
        input [ADDR_W-1:0]   t1_src;
        input [ADDR_W-1:0]   l10_src;
        input [ADDR_W-1:0]   z1_src;
        input                 root_full;
        reg   [13:0]         t0_dst_ext;
        begin
            t0_dst_ext = {{(14-ADDR_W){1'b0}}, t0_dst};
            pack_adjust_task = pack_task(OP_ADJUST_T0, level, t0_dst[9:0],
                                         t1_src, l10_src, z1_src,
                                         {1'b0, 3'd0, t0_dst_ext[13:10]});
        end
    endfunction

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            run_state     <= RUN_IDLE;
            level_q       <= {LEVEL_W{1'b0}};
            index_q       <= {INDEX_W{1'b0}};
            state_q       <= ST_RIGHT_DOWN;
            sub_q         <= SUB_READ;
            bank_q        <= 1'b1;
            root_z1_merged_q <= 1'b0;
            issued_op_q   <= 4'd0;
            fail_status_q <= 8'h00;
            busy          <= 1'b0;
            done          <= 1'b0;
            fail          <= 1'b0;
            status        <= 8'h00;
            dbg_level     <= {LEVEL_W{1'b0}};
            dbg_index     <= {INDEX_W{1'b0}};
            dbg_state     <= ST_RIGHT_DOWN;
        end else begin
            done      <= 1'b0;
            fail      <= 1'b0;
            dbg_level <= level_q;
            dbg_index <= index_q;
            dbg_state <= state_q;

            case (run_state)
                RUN_IDLE: begin
                    busy   <= 1'b0;
                    status <= 8'h00;
                    if (start) begin
                        if ((cfg_depth == {LEVEL_W{1'b0}}) || (cfg_depth > 4'd10)) begin
                            fail_status_q <= 8'hE1;
                            busy          <= 1'b1;
                            run_state     <= RUN_FAIL;
                        end else begin
                            level_q   <= {LEVEL_W{1'b0}};
                            index_q   <= {INDEX_W{1'b0}};
                            state_q   <= ST_RIGHT_DOWN;
                            sub_q     <= cfg_dynamic_tree ? SUB_LDL : SUB_READ;
                            bank_q    <= 1'b1;
                            root_z1_merged_q <= 1'b0;
                            busy      <= 1'b1;
                            run_state <= RUN_EMIT;
                        end
                    end
                end

                RUN_EMIT: begin
                    if (task_valid && task_ready) begin
                        issued_op_q <= task_word[67:64];
                        run_state   <= RUN_WAIT;
                    end
                end

                RUN_WAIT: begin
                    if (task_done) begin
                        if (task_fail) begin
                            fail_status_q <= task_status;
                            run_state     <= RUN_FAIL;
                        end else begin
                            if (issued_op_q == OP_DYNAMIC_LDL) begin
                                sub_q     <= SUB_READ;
                                run_state <= RUN_EMIT;
                            end else if (issued_op_q == OP_READ_L10) begin
                                sub_q     <= SUB_SPLIT;
                                run_state <= RUN_EMIT;
                            end else if (issued_op_q == OP_SPLIT_T1) begin
                                level_q   <= child_level;
                                index_q   <= right_child_index;
                                state_q   <= ST_RIGHT_DOWN;
                                sub_q     <= cfg_dynamic_tree ? SUB_LDL : SUB_READ;
                                run_state <= RUN_EMIT;
                            end else if (issued_op_q == OP_ADJUST_T0) begin
                                if (outer_root_adjust) begin
                                    bank_q    <= 1'b0;
                                    level_q   <= {LEVEL_W{1'b0}};
                                    index_q   <= {INDEX_W{1'b0}};
                                    state_q   <= ST_RIGHT_DOWN;
                                    sub_q     <= cfg_dynamic_tree ? SUB_LDL : SUB_READ;
                                    root_z1_merged_q <= 1'b0;
                                end else begin
                                    level_q   <= child_level;
                                    index_q   <= left_child_index;
                                    state_q   <= ST_LEFT_DOWN;
                                    sub_q     <= cfg_dynamic_tree ? SUB_LDL : SUB_READ;
                                end
                                run_state <= RUN_EMIT;
                            end else if (issued_op_q == OP_SAMPLE_PAIR) begin
                                level_q   <= parent_level;
                                index_q   <= parent_index;
                                state_q   <= from_right_child ? ST_RIGHT_UP : ST_LEFT_UP;
                                sub_q     <= SUB_READ;
                                run_state <= RUN_EMIT;
                            end else if (issued_op_q == OP_MERGE_Z) begin
                                if (at_root) begin
                                    if (bank_q && !root_z1_merged_q) begin
                                        root_z1_merged_q <= 1'b1;
                                        state_q          <= ST_RIGHT_UP;
                                        sub_q            <= SUB_READ;
                                        run_state        <= RUN_EMIT;
                                    end else begin
                                        run_state <= RUN_DONE;
                                    end
                                end else begin
                                    level_q   <= parent_level;
                                    index_q   <= parent_index;
                                    state_q   <= from_right_child ? ST_RIGHT_UP : ST_LEFT_UP;
                                    sub_q     <= SUB_READ;
                                    run_state <= RUN_EMIT;
                                end
                            end else begin
                                fail_status_q <= 8'hE2;
                                run_state     <= RUN_FAIL;
                            end
                        end
                    end
                end

                RUN_DONE: begin
                    busy      <= 1'b0;
                    done      <= 1'b1;
                    status    <= 8'h00;
                    run_state <= RUN_IDLE;
                end

                RUN_FAIL: begin
                    busy      <= 1'b0;
                    fail      <= 1'b1;
                    status    <= fail_status_q;
                    run_state <= RUN_IDLE;
                end

                default: begin
                    busy      <= 1'b0;
                    run_state <= RUN_IDLE;
                end
            endcase
        end
    end

endmodule
