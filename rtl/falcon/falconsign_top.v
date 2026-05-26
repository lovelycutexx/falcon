`timescale 1ns/1ps
// FalconSign Top — 3-layer control: phase FSM → task scheduler → EXU cluster.
module falconsign_top #(
    parameter ADDR_W=13, LEVEL_W=4, INDEX_W=10
)(
    input  wire        clk, rst_n,
    input  wire        bus_cs, bus_wr,
    input  wire [15:0] bus_addr,
    input  wire [31:0] bus_wdata,
    output reg  [31:0] bus_rdata,
    output reg         bus_ready, bus_irq,
    output wire        busy, output reg done, fail, output reg[7:0] status
);
    localparam [15:0] REG_CR     = 16'h0000;
    localparam [15:0] REG_SR     = 16'h0004;
    localparam [15:0] REG_CFG    = 16'h0008;
    localparam [15:0] REG_MEM_HI = 16'h000C;
    localparam [3:0] SI=0, SH=1, HP=2, FC=3, FS=4, VD=5, IV=6, FI=7, N1=8, RC=9, CN=10, EN=11, OU=12, SD=13;
    localparam integer MEM_DEPTH = 8192;
    localparam [4:0]  FALCON_LOGN = 5'd9;
    localparam [ADDR_W-1:0] FALCON_N_WORDS = 512;
    localparam [ADDR_W-1:0] HTP_C_WORDS    = FALCON_N_WORDS; // one FP64 complex coefficient per word
    // Sign workspace map. Each FP64 complex polynomial occupies 512 memory
    // words. The full ffLDL tree stores every L10 polynomial coefficient
    // plus leaves: 2816 words.
    localparam [ADDR_W-1:0] LAYOUT_C_BASE      = 0;
    localparam [ADDR_W-1:0] LAYOUT_FFT_BASE    = 0;       // FFT(c), aliases t0
    localparam [ADDR_W-1:0] LAYOUT_T0_BASE     = LAYOUT_FFT_BASE;
    localparam [ADDR_W-1:0] LAYOUT_T1_BASE     = 512;
    localparam [ADDR_W-1:0] LAYOUT_TREE_BASE   = 1024;
    localparam [ADDR_W-1:0] LAYOUT_Z0_BASE     = 3840;
    localparam [ADDR_W-1:0] LAYOUT_Z1_BASE     = 4352;
    localparam [ADDR_W-1:0] LAYOUT_H_WORK_BASE = LAYOUT_Z1_BASE + {{(ADDR_W-6){1'b0}}, 6'd32};
    localparam [ADDR_W-1:0] LAYOUT_B00_BASE    = 4864;
    localparam [ADDR_W-1:0] LAYOUT_B01_BASE    = 5376;
    localparam [ADDR_W-1:0] LAYOUT_B10_BASE    = 5888;
    localparam [ADDR_W-1:0] LAYOUT_B11_BASE    = 6400;
    localparam [ADDR_W-1:0] LAYOUT_SIG_BASE    = 6912;
    localparam [ADDR_W-1:0] LAYOUT_C_INT_BASE  = 7424;
    localparam [ADDR_W-1:0] LAYOUT_H_BASE      = 7456;
    localparam [ADDR_W-1:0] LAYOUT_S1_BASE     = 7488;    // s1 output (32 words)
    localparam [ADDR_W-1:0] LAYOUT_TMP_BASE    = 7552;    // ffSampling internal scratch; scalar leaf scratch aliases SIG
    localparam [ADDR_W-1:0] LAYOUT_T_BASE      = LAYOUT_T0_BASE;
    localparam [ADDR_W-1:0] LAYOUT_Z_BASE      = LAYOUT_Z0_BASE;
    localparam [ADDR_W-1:0] NTT_N_WORDS        = 32;      // 512 int16 / 16 per word
    localparam [ADDR_W-1:0] LAYOUT_NORM_WORDS = NTT_N_WORDS;
    reg [3:0] st, sn;
    reg cr_start;
    reg cfg_bypass_fs;
    reg cfg_force_accept;
    reg cfg_start_at_fs;
    reg cfg_dynamic_tree;
    reg [1:0] mem_addr_hi;

    function [63:0] u16_to_f64;
        input [15:0] x;
        integer ii;
        integer pos;
        reg [10:0] exp;
        reg [51:0] frac;
        reg [63:0] x64;
        begin
            if (x == 16'd0) begin
                u16_to_f64 = 64'd0;
            end else begin
                x64 = {48'd0, x};
                pos = 0;
                for (ii = 0; ii < 16; ii = ii + 1) begin
                    if (x[15 - ii]) begin
                        pos = 15 - ii;
                        ii = 16;
                    end
                end
                exp = 11'd1023 + pos;
                frac = (x64 << (63 - pos)) >> 11;
                u16_to_f64 = {1'b0, exp, frac};
            end
        end
    endfunction

    // ─── Memory ───
    wire mem_a_rd_en,mem_a_wr_en,mem_a_rd0,mem_a_rd1;
    wire [ADDR_W-1:0] mem_a_rd_addr0,mem_a_rd_addr1,mem_a_wr_addr0,mem_a_wr_addr1;
    wire [255:0] mem_a_rd_data0,mem_a_rd_data1,mem_a_wr_data0,mem_a_wr_data1;
    wire mem_b_rd_en,mem_b_wr_en;
    wire [ADDR_W-1:0] mem_b_rd_addr,mem_b_wr_addr;
    wire [255:0] mem_b_rd_data,mem_b_wr_data;
    wire mem_c_en,mem_c_wr,mem_c_ready;
    wire [ADDR_W+4:0] mem_c_addr;
    wire [31:0] mem_c_wr_data,mem_c_rd_data;

    falconsign_memory #(.ADDR_W(ADDR_W),.DEPTH(MEM_DEPTH)) u_mem (
        .clk(clk),.rst_n(rst_n),
        .port_a_rd_en(mem_a_rd_en),.port_a_rd_addr0(mem_a_rd_addr0),.port_a_rd_addr1(mem_a_rd_addr1),
        .port_a_rd_data0(mem_a_rd_data0),.port_a_rd_data1(mem_a_rd_data1),
        .port_a_wr_en(mem_a_wr_en),.port_a_wr_addr0(mem_a_wr_addr0),.port_a_wr_addr1(mem_a_wr_addr1),
        .port_a_wr_data0(mem_a_wr_data0),.port_a_wr_data1(mem_a_wr_data1),
        .port_b_rd_en(mem_b_rd_en),.port_b_rd_addr(mem_b_rd_addr),
        .port_b_rd_data(mem_b_rd_data),
        .port_b_wr_en(mem_b_wr_en),.port_b_wr_addr(mem_b_wr_addr),
        .port_b_wr_data(mem_b_wr_data),
        .port_c_en(mem_c_en),.port_c_wr(mem_c_wr),.port_c_addr(mem_c_addr),
        .port_c_wr_data(mem_c_wr_data),.port_c_rd_data(mem_c_rd_data),
        .port_c_ready(mem_c_ready));

    // ─── FPU ───
    wire fpu_req_valid,fpu_req_ready; wire[3:0] fpu_req_op;
    wire[63:0] fpu_req_a,fpu_req_b,fpu_req_c;
    wire[1:0] fpu_req_fmt; wire[2:0] fpu_req_rm; wire[1:0] fpu_req_fcvt_op;
    wire fpu_rsp_valid,fpu_rsp_ready; wire[63:0] fpu_rsp_result; wire[4:0] fpu_rsp_flags;
    falcon_fp_fpu u_fpu(.clk(clk),.rst_n(rst_n),
        .req_valid(fpu_req_valid),.req_ready(fpu_req_ready),
        .req_op(fpu_req_op),.req_a(fpu_req_a),.req_b(fpu_req_b),.req_c(fpu_req_c),
        .req_fmt(fpu_req_fmt),.req_rm(fpu_req_rm),.req_fcvt_op(fpu_req_fcvt_op),
        .rsp_valid(fpu_rsp_valid),.rsp_ready(fpu_rsp_ready),
        .rsp_result(fpu_rsp_result),.rsp_flags(fpu_rsp_flags),.busy());
    assign fpu_req_fmt=0; assign fpu_req_rm=0; assign fpu_req_fcvt_op=0;
    assign fpu_rsp_ready=1;

    wire        sz_fpu_req_valid, sz_fpu_req_ready;
    wire [3:0]  sz_fpu_req_op; wire[63:0] sz_fpu_req_a,sz_fpu_req_b,sz_fpu_req_c;
    wire        sz_fpu_rsp_valid; wire[63:0] sz_fpu_rsp_result;
    wire        fe_fpu_req_valid, fe_fpu_req_ready;
    wire [3:0]  fe_fpu_req_op; wire[63:0] fe_fpu_req_a,fe_fpu_req_b,fe_fpu_req_c;
    wire        fe_fpu_rsp_valid; wire[63:0] fe_fpu_rsp_result;
    wire        vd_fpu_req_valid, vd_fpu_req_ready;
    wire [3:0]  vd_fpu_req_op; wire[63:0] vd_fpu_req_a,vd_fpu_req_b,vd_fpu_req_c;
    wire        vd_fpu_rsp_valid; wire[63:0] vd_fpu_rsp_result;

    // ─── 2-way FPU mux (SamplerZ > ffSampling EXU) ───
    // Track the owner of the outstanding request so a response is never routed
    // according to a later request from the other client.
    reg fpu_owner_valid;
    reg [1:0] fpu_owner;
    wire fpu_mux_idle = !fpu_owner_valid;
    wire fpu_sel_sz   = sz_fpu_req_valid;
    wire fpu_sel_fe   = !sz_fpu_req_valid && fe_fpu_req_valid;
    wire fpu_sel_vd   = !sz_fpu_req_valid && !fe_fpu_req_valid && vd_fpu_req_valid;

    assign fpu_req_valid = fpu_mux_idle && (sz_fpu_req_valid || fe_fpu_req_valid || vd_fpu_req_valid);
    assign fpu_req_op    = fpu_sel_sz ? sz_fpu_req_op :
                           fpu_sel_fe ? fe_fpu_req_op : vd_fpu_req_op;
    assign fpu_req_a     = fpu_sel_sz ? sz_fpu_req_a  :
                           fpu_sel_fe ? fe_fpu_req_a  : vd_fpu_req_a;
    assign fpu_req_b     = fpu_sel_sz ? sz_fpu_req_b  :
                           fpu_sel_fe ? fe_fpu_req_b  : vd_fpu_req_b;
    assign fpu_req_c     = fpu_sel_sz ? sz_fpu_req_c  :
                           fpu_sel_fe ? fe_fpu_req_c  : vd_fpu_req_c;
    assign sz_fpu_req_ready = fpu_mux_idle && fpu_sel_sz && fpu_req_ready;
    assign fe_fpu_req_ready = fpu_mux_idle && fpu_sel_fe && fpu_req_ready;
    assign vd_fpu_req_ready = fpu_mux_idle && fpu_sel_vd && fpu_req_ready;
    assign sz_fpu_rsp_valid = fpu_owner_valid && (fpu_owner == 2'd1) && fpu_rsp_valid;
    assign fe_fpu_rsp_valid = fpu_owner_valid && (fpu_owner == 2'd2) && fpu_rsp_valid;
    assign vd_fpu_rsp_valid = fpu_owner_valid && (fpu_owner == 2'd3) && fpu_rsp_valid;
    assign sz_fpu_rsp_result = fpu_rsp_result;
    assign fe_fpu_rsp_result = fpu_rsp_result;
    assign vd_fpu_rsp_result = fpu_rsp_result;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fpu_owner_valid <= 1'b0;
            fpu_owner       <= 2'd0;
        end else begin
            if (fpu_owner_valid && fpu_rsp_valid && fpu_rsp_ready) begin
                fpu_owner_valid <= 1'b0;
            end
            if (fpu_mux_idle && fpu_req_valid && fpu_req_ready) begin
                fpu_owner_valid <= 1'b1;
                fpu_owner       <= fpu_sel_sz ? 2'd1 :
                                   fpu_sel_fe ? 2'd2 : 2'd3;
            end
        end
    end

    // ─── SHAKE256 ───
    wire        shake_start, shake_ready;
    reg         shake_absorb;
    reg  [63:0] shake_din;
    reg         shake_din_last;
    wire        shake_dout_valid;
    wire [63:0] shake_dout;
    wire        shake_fifo_wr_ready;
    wire        shake_fifo_rd_valid;
    wire [63:0] shake_fifo_rd_data;
    wire        htp_hash_ready;

    falconsign_shake256 u_shake(
        .clk(clk), .rst_n(rst_n),
        .start(shake_start), .ready(shake_ready),
        .absorb(shake_absorb),
        .din(shake_din), .din_last(shake_din_last), .din_last_bytes(3'd0),
        .dout_ready(shake_fifo_wr_ready),
        .dout_valid(shake_dout_valid), .dout(shake_dout));

    falconsign_word_fifo #(.WIDTH(64), .DEPTH(16), .ADDR_W(4)) u_shake_fifo (
        .clk(clk),
        .rst_n(rst_n),
        .wr_valid(shake_dout_valid),
        .wr_ready(shake_fifo_wr_ready),
        .wr_data(shake_dout),
        .rd_valid(shake_fifo_rd_valid),
        .rd_ready(htp_hash_ready),
        .rd_data(shake_fifo_rd_data)
    );

    // ─── HashToPoint ───
    wire        htp_start;
    wire        htp_ready;
    wire [63:0] htp_hash_word;
    wire        htp_hash_valid;
    wire [15:0] htp_coeff;
    wire        htp_coeff_valid;

    assign htp_hash_word  = shake_fifo_rd_data;
    assign htp_hash_valid = shake_fifo_rd_valid;

    falconsign_hash_to_point #(.N(512)) u_htp(
        .clk(clk), .rst_n(rst_n),
        .start(htp_start), .ready(htp_ready),
        .hash_word(htp_hash_word), .hash_valid(htp_hash_valid),
        .hash_ready(htp_hash_ready),
        .coeff(htp_coeff), .coeff_valid(htp_coeff_valid));

    // ─── HP coefficient packing → memory write ───
    reg  [3:0]         hp_coeff_cnt;    // c_int packs 16 coefficients per 256-bit word
    reg  [255:0]       hp_coeff_buf;
    reg  [ADDR_W-1:0]  hp_wr_addr;      // next packed c write address
    reg                hp_wr_en;
    reg  [255:0]       hp_wr_data;
    reg  [ADDR_W-1:0]  hp_cint_wr_addr;
    reg                hp_cint_wr_en;
    reg  [255:0]       hp_cint_wr_data;
    // ─── SH phase: test message absorption ───
    // A short hardcoded test message (32 bytes = 4 x 64-bit words)
    localparam [63:0] TEST_MSG_W0 = 64'h46414C434F4E5F53; // "FALCON_S"
    localparam [63:0] TEST_MSG_W1 = 64'h49474E5F54455354; // "IGN_TEST"
    localparam [63:0] TEST_MSG_W2 = 64'h5F4D53475F56312E; // "_MSG_V1."
    localparam [63:0] TEST_MSG_W3 = 64'h305F5F5F5F5F5F5F; // "0______"
    reg [2:0] sh_word_idx;

    // ─── ffSampling EXU: task / memory / SamplerZ passthrough ───
    wire        fe_task_valid, fe_task_ready, fe_task_done, fe_task_fail;
    wire [7:0]  fe_task_status;
    wire [67:0] fe_task_word;
    wire        fe_mem_rd_en, fe_mem_wr_en;
    wire [ADDR_W-1:0] fe_mem_rd_addr, fe_mem_wr_addr;
    wire [255:0] fe_mem_rd_data, fe_mem_wr_data;
    wire [ADDR_W-1:0] fe_twiddle_addr;
    wire [63:0] fe_twiddle_re, fe_twiddle_im;
    wire        fe_sz_cmd_valid, fe_sz_cmd_ready;
    wire [63:0] fe_sz_cmd_mu, fe_sz_cmd_sigma_inv; wire fe_sz_cmd_pair;

    // ─── FFT EXU ───
    wire fft_cmd_valid,fft_cmd_ready; wire[2:0] fft_cmd_opcode; wire[4:0] fft_cmd_logn;
    wire[ADDR_W-1:0] fft_rd_addr0,fft_rd_addr1,fft_twiddle_addr;
    wire[63:0] fft_rd_data0_re,fft_rd_data0_im,fft_rd_data1_re,fft_rd_data1_im;
    wire fft_wr_en,fft_rsp_valid,fft_rsp_done,fft_rsp_fail,fft_busy;
    wire[ADDR_W-1:0] fft_wr_addr0,fft_wr_addr1;
    wire[63:0] fft_wr_data0_re,fft_wr_data0_im,fft_wr_data1_re,fft_wr_data1_im;
    wire[63:0] fft_twiddle_re, fft_twiddle_im;
    wire[7:0] fft_rsp_status;
    wire[ADDR_W-1:0] fft_mem_base = (st == IV) ? LAYOUT_Z_BASE : LAYOUT_FFT_BASE;
    falcon_f64_fft_exu #(.ADDR_W(ADDR_W)) u_fft(
        .clk(clk),.rst_n(rst_n),
        .cmd_valid(fft_cmd_valid),.cmd_ready(fft_cmd_ready),
        .cmd_opcode(fft_cmd_opcode),.cmd_logn(fft_cmd_logn),
        .mem_rd_addr0(fft_rd_addr0),.mem_rd_addr1(fft_rd_addr1),
        .mem_rd_data0_re(fft_rd_data0_re),.mem_rd_data0_im(fft_rd_data0_im),
        .mem_rd_data1_re(fft_rd_data1_re),.mem_rd_data1_im(fft_rd_data1_im),
        .twiddle_addr(fft_twiddle_addr),.twiddle_re(fft_twiddle_re),.twiddle_im(fft_twiddle_im),
        .mem_wr_en(fft_wr_en),.mem_wr_addr0(fft_wr_addr0),.mem_wr_addr1(fft_wr_addr1),
        .mem_wr_data0_re(fft_wr_data0_re),.mem_wr_data0_im(fft_wr_data0_im),
        .mem_wr_data1_re(fft_wr_data1_re),.mem_wr_data1_im(fft_wr_data1_im),
        .rsp_valid(fft_rsp_valid),.rsp_done(fft_rsp_done),.rsp_fail(fft_rsp_fail),
        .rsp_status(fft_rsp_status),.busy(fft_busy));

    // ─── SamplerZ ───
    wire sz_cmd_valid,sz_cmd_ready; wire[63:0] sz_cmd_mu,sz_cmd_sigma_inv,sz_cmd_sigma_min;
    wire sz_cmd_pair,sz_rsp_valid,sz_rsp_accept,sz_busy,sz_done,sz_fail;
    wire[63:0] sz_rsp_z0,sz_rsp_z1;
    wire sz_rng_req,sz_rng_ack; wire[255:0] sz_rng_data;
    falconsign_samplerz_top #(.RNG_DATA_W(256)) u_sz(
        .clk(clk),.rst_n(rst_n),
        .cmd_valid(sz_cmd_valid),.cmd_ready(sz_cmd_ready),
        .cmd_mu(sz_cmd_mu),.cmd_sigma_inv(sz_cmd_sigma_inv),
        .cmd_sigma_min(sz_cmd_sigma_min),.cmd_pair_mode(sz_cmd_pair),
        .rsp_valid(sz_rsp_valid),.rsp_ready(1'b1),.rsp_z0(sz_rsp_z0),.rsp_z1(sz_rsp_z1),
        .rsp_accept(sz_rsp_accept),.rsp_status(),
        .fpu_req_valid(sz_fpu_req_valid),.fpu_req_ready(sz_fpu_req_ready),
        .fpu_req_op(sz_fpu_req_op),.fpu_req_a(sz_fpu_req_a),
        .fpu_req_b(sz_fpu_req_b),.fpu_req_c(sz_fpu_req_c),
        .fpu_req_fmt(),.fpu_req_rm(),.fpu_req_fcvt_op(),
        .fpu_rsp_valid(sz_fpu_rsp_valid),.fpu_rsp_ready(),.fpu_rsp_result(sz_fpu_rsp_result),
        .rng_req(sz_rng_req),.rng_ack(sz_rng_ack),.rng_data(sz_rng_data),
        .busy(sz_busy),.done(sz_done),.fail(sz_fail));

    // ─── ffSampling EXU (SPLIT/MERGE/ADJUST) ───
    falcon_f64_ffsampling_exu #(.ADDR_W(ADDR_W)) u_fe(
        .clk(clk),.rst_n(rst_n),
        .task_valid(fe_task_valid),.task_ready(fe_task_ready),
        .task_word(fe_task_word),.task_done(fe_task_done),.task_fail(fe_task_fail),
        .task_status(fe_task_status),
        .mem_rd_en(fe_mem_rd_en),.mem_rd_addr(fe_mem_rd_addr),
        .mem_rd_data(fe_mem_rd_data),.mem_wr_en(fe_mem_wr_en),
        .mem_wr_addr(fe_mem_wr_addr),.mem_wr_data(fe_mem_wr_data),
        .twiddle_addr(fe_twiddle_addr),.twiddle_re(fe_twiddle_re),.twiddle_im(fe_twiddle_im),
        .fpu_req_valid(fe_fpu_req_valid),.fpu_req_ready(fe_fpu_req_ready),
        .fpu_req_op(fe_fpu_req_op),.fpu_req_a(fe_fpu_req_a),
        .fpu_req_b(fe_fpu_req_b),.fpu_req_c(fe_fpu_req_c),
        .fpu_rsp_valid(fe_fpu_rsp_valid),.fpu_rsp_result(fe_fpu_rsp_result),
        .sz_cmd_valid(fe_sz_cmd_valid),.sz_cmd_ready(fe_sz_cmd_ready),
        .sz_cmd_mu(fe_sz_cmd_mu),.sz_cmd_sigma_inv(fe_sz_cmd_sigma_inv),
        .sz_cmd_pair(fe_sz_cmd_pair),.sz_rsp_valid(sz_rsp_valid),
        .sz_rsp_z0(sz_rsp_z0),.sz_rsp_z1(sz_rsp_z1));

    // Real B_hat second-component multiply:
    // s2_fft = (t0 - z0) * b01 + (t1 - z1) * b11.
    wire        vd_start, vd_start_ready, vd_done, vd_fail;
    wire [7:0]  vd_status;
    wire        vd_mem_rd_en, vd_mem_wr_en;
    wire [ADDR_W-1:0] vd_mem_rd_addr, vd_mem_wr_addr;
    wire [255:0] vd_mem_wr_data;

    falcon_f64_bhat_mul_exu #(.ADDR_W(ADDR_W)) u_vd (
        .clk(clk),
        .rst_n(rst_n),
        .start(vd_start),
        .start_ready(vd_start_ready),
        .identity_mode(1'b0),
        .t_base(LAYOUT_T0_BASE),
        .z_base(LAYOUT_Z0_BASE),
        .b00_base(LAYOUT_B00_BASE),
        .b01_base(LAYOUT_B01_BASE),
        .b10_base(LAYOUT_B10_BASE),
        .b11_base(LAYOUT_B11_BASE),
        .s2_fft_base(LAYOUT_Z0_BASE),
        .word_count(FALCON_N_WORDS),
        .mem_rd_en(vd_mem_rd_en),
        .mem_rd_addr(vd_mem_rd_addr),
        .mem_rd_data(mem_b_rd_data),
        .mem_wr_en(vd_mem_wr_en),
        .mem_wr_addr(vd_mem_wr_addr),
        .mem_wr_data(vd_mem_wr_data),
        .fpu_req_valid(vd_fpu_req_valid),
        .fpu_req_ready(vd_fpu_req_ready),
        .fpu_req_op(vd_fpu_req_op),
        .fpu_req_a(vd_fpu_req_a),
        .fpu_req_b(vd_fpu_req_b),
        .fpu_req_c(vd_fpu_req_c),
        .fpu_rsp_valid(vd_fpu_rsp_valid),
        .fpu_rsp_result(vd_fpu_rsp_result),
        .done(vd_done),
        .fail(vd_fail),
        .status(vd_status)
    );

    // ─── Task Scheduler ───
    wire ts_start,ts_start_ready; wire[LEVEL_W-1:0] ts_cfg_depth; wire ts_cfg_dynamic;
    wire[ADDR_W-1:0] ts_t_base,ts_tree_base,ts_z_base,ts_tmp_base;
    wire ts_task_valid,ts_task_ready; wire[67:0] ts_task_word;
    wire ts_task_done,ts_task_fail,ts_busy,ts_done,ts_fail;
    wire [7:0] ts_task_status,ts_status;
    falconsign_ffsampling_task_update #(.LEVEL_W(LEVEL_W),.INDEX_W(INDEX_W),.ADDR_W(ADDR_W))
    u_ts(.clk(clk),.rst_n(rst_n),
        .start(ts_start),.start_ready(ts_start_ready),
        .cfg_depth(ts_cfg_depth),.cfg_dynamic_tree(ts_cfg_dynamic),
        .cfg_t_base(ts_t_base),.cfg_tree_base(ts_tree_base),.cfg_z_base(ts_z_base),
        .cfg_tmp_base(ts_tmp_base),
        .task_valid(ts_task_valid),.task_ready(ts_task_ready),
        .task_word(ts_task_word),.task_done(ts_task_done),.task_fail(ts_task_fail),
        .task_status(ts_task_status),.busy(ts_busy),.done(ts_done),.fail(ts_fail),.status(ts_status),
        .dbg_level(),.dbg_index(),.dbg_state());

    // ─── Task routing: scheduler → ffSampling EXU ───
    assign sz_cmd_valid    = fe_sz_cmd_valid;
    assign sz_cmd_mu       = fe_sz_cmd_mu;
    assign sz_cmd_sigma_inv = fe_sz_cmd_sigma_inv;
    assign sz_cmd_pair     = fe_sz_cmd_pair;
    assign fe_sz_cmd_ready = sz_cmd_ready;

    // ─── RNG ───
    wire rng_seed_valid,rng_seed_ready,rng_valid,rng_ready; wire[255:0] rng_seed_key;
    wire[95:0] rng_seed_nonce; wire[511:0] rng_block;
    falconsign_chacha20_rng u_rng(.clk(clk),.rst_n(rst_n),
        .seed_valid(rng_seed_valid),.seed_ready(rng_seed_ready),
        .seed_key(rng_seed_key),.seed_nonce(rng_seed_nonce),
        .rng_valid(rng_valid),.rng_ready(rng_ready),
        .rng_block(rng_block),.busy());

    // ─── Task routing: scheduler → ffSampling EXU ───
    assign fe_task_valid = ts_task_valid;
    assign fe_task_word  = ts_task_word;
    assign ts_task_ready = fe_task_ready;
    assign ts_task_done  = fe_task_done;
    assign ts_task_fail  = fe_task_fail;
    assign ts_task_status = fe_task_status;

    // ─── Memory Port B: mux between HP-write and ffSampling EXU ───
    wire        fi_start, fi_start_ready, fi_done, fi_fail;
    wire [7:0]  fi_status;
    wire        fi_mem_rd_en, fi_mem_wr_en;
    wire [ADDR_W-1:0] fi_mem_rd_addr, fi_mem_wr_addr;
    wire [255:0] fi_mem_wr_data;

    falconsign_fpr_to_int16 #(.ADDR_W(ADDR_W)) u_fpr_to_i16 (
        .clk(clk),
        .rst_n(rst_n),
        .start(fi_start),
        .start_ready(fi_start_ready),
        .src_base(LAYOUT_Z_BASE),
        .dst_base(LAYOUT_SIG_BASE),
        .coeff_count(FALCON_N_WORDS),
        .mem_rd_en(fi_mem_rd_en),
        .mem_rd_addr(fi_mem_rd_addr),
        .mem_rd_data(mem_b_rd_data),
        .mem_wr_en(fi_mem_wr_en),
        .mem_wr_addr(fi_mem_wr_addr),
        .mem_wr_data(fi_mem_wr_data),
        .done(fi_done),
        .fail(fi_fail),
        .status(fi_status)
    );

    wire use_fe_portb   = (st == FS);
    wire use_vd_portb   = (st == VD);
    wire use_fi_portb   = (st == FI);
    wire use_norm_portb = (st == RC);
    wire use_ntt_portb  = (st == N1);
    assign fe_mem_rd_data = mem_b_rd_data;
    assign mem_b_rd_en    = use_fe_portb   ? fe_mem_rd_en   :
                             use_vd_portb   ? vd_mem_rd_en   :
                             use_fi_portb   ? fi_mem_rd_en   :
                             use_norm_portb ? norm_mem_rd_en :
                             use_ntt_portb  ? ntt_mem_rd_en  : 1'b0;
    assign mem_b_rd_addr  = use_fe_portb   ? fe_mem_rd_addr   :
                             use_vd_portb   ? vd_mem_rd_addr   :
                             use_fi_portb   ? fi_mem_rd_addr   :
                             use_norm_portb ? norm_mem_rd_addr :
                             use_ntt_portb  ? ntt_mem_rd_addr  : {ADDR_W{1'b0}};
    assign mem_b_wr_en    = use_fe_portb   ? fe_mem_wr_en   :
                             use_vd_portb   ? vd_mem_wr_en   :
                             use_fi_portb   ? fi_mem_wr_en   :
                             use_norm_portb ? 1'b0           :
                             use_ntt_portb  ? ntt_mem_wr_en  : hp_wr_en;
    assign mem_b_wr_addr  = use_fe_portb   ? fe_mem_wr_addr   :
                             use_vd_portb   ? vd_mem_wr_addr   :
                             use_fi_portb   ? fi_mem_wr_addr   :
                             use_ntt_portb  ? ntt_mem_wr_addr  : hp_wr_addr;
    assign mem_b_wr_data  = use_fe_portb   ? fe_mem_wr_data   :
                             use_vd_portb   ? vd_mem_wr_data   :
                             use_fi_portb   ? fi_mem_wr_data   :
                             use_ntt_portb  ? ntt_mem_wr_data  : hp_wr_data;

    // ─── Port A: FFT ───
    wire use_hp_cint_porta = (st == HP);
    wire [ADDR_W-1:0] hp_cint_porta_addr = hp_cint_wr_en ? (hp_cint_wr_addr - 1'b1) : hp_cint_wr_addr;
    assign mem_a_rd_en    = use_hp_cint_porta ? 1'b0 : fft_busy;
    assign mem_a_rd_addr0 = fft_mem_base + fft_rd_addr0;
    assign mem_a_rd_addr1 = fft_mem_base + fft_rd_addr1;
    assign mem_a_wr_en    = use_hp_cint_porta ? hp_cint_wr_en : fft_wr_en;
    assign mem_a_wr_addr0 = use_hp_cint_porta ? hp_cint_porta_addr : (fft_mem_base + fft_wr_addr0);
    assign mem_a_wr_addr1 = use_hp_cint_porta ? hp_cint_porta_addr : (fft_mem_base + fft_wr_addr1);
    assign fft_rd_data0_re = mem_a_rd_data0[63:0];
    assign fft_rd_data0_im = mem_a_rd_data0[127:64];
    assign fft_rd_data1_re = mem_a_rd_data1[63:0];
    assign fft_rd_data1_im = mem_a_rd_data1[127:64];
    assign mem_a_wr_data0 = use_hp_cint_porta ? hp_cint_wr_data : {128'd0, fft_wr_data0_im, fft_wr_data0_re};
    assign mem_a_wr_data1 = use_hp_cint_porta ? hp_cint_wr_data : {128'd0, fft_wr_data1_im, fft_wr_data1_re};

    // ─── Twiddle ROM ───
    wire [7:0] twiddle_rom_addr = (st == FS) ? fe_twiddle_addr[7:0] : fft_twiddle_addr[7:0];
    wire [63:0] twiddle_rom_re, twiddle_rom_im;
    wire [63:0] gm_rom_re, gm_rom_im;
    falconsign_twiddle_rom #(.ADDR_W(8),.DEPTH(256)) u_tw(
        .clk(clk),.addr(twiddle_rom_addr),
        .twiddle_re(twiddle_rom_re),.twiddle_im(twiddle_rom_im));
    falconsign_gm_rom #(.ADDR_W(8),.DEPTH(255)) u_gm(
        .clk(clk),
        .addr(fe_twiddle_addr[7:0]),
        .gm_re(gm_rom_re),.gm_im(gm_rom_im));
    assign fft_twiddle_re = twiddle_rom_re;
    assign fft_twiddle_im = twiddle_rom_im;
    assign fe_twiddle_re  = gm_rom_re;
    assign fe_twiddle_im  = gm_rom_im;

    // ─── NTT (s1 = c - s2*h mod q) ───
    wire        ntt_start, ntt_ready, ntt_done, ntt_fail;
    wire [7:0]  ntt_status;
    wire        ntt_mem_rd_en, ntt_mem_wr_en;
    wire [ADDR_W-1:0] ntt_mem_rd_addr, ntt_mem_wr_addr;
    wire [255:0] ntt_mem_wr_data;
    wire [8:0]  ntt_twiddle_rom_addr;
    wire [13:0] ntt_twiddle_rom_data;
    wire [9:0]  ntt_psi_rom_addr;
    wire [13:0] ntt_psi_rom_data;

    // NTT twiddle ROM (512 x 14)
    falconsign_ntt_twiddle_rom #(.ADDR_W(9)) u_ntt_tw (
        .clk(clk), .addr(ntt_twiddle_rom_addr), .data(ntt_twiddle_rom_data));

    // NTT psi table ROM (1024 x 14)
    falconsign_ntt_psi_rom #(.ADDR_W(10)) u_ntt_psi (
        .clk(clk), .addr(ntt_psi_rom_addr), .data(ntt_psi_rom_data));

    // NTT EXU
    falconsign_ntt_exu #(.ADDR_W(ADDR_W)) u_ntt (
        .clk(clk), .rst_n(rst_n),
        .start(ntt_start), .start_ready(ntt_ready),
        .done(ntt_done), .fail(ntt_fail), .status(ntt_status),
        .cfg_h_base(LAYOUT_H_BASE),
        .cfg_h_work_base(LAYOUT_H_WORK_BASE),
        .cfg_s2_base(LAYOUT_SIG_BASE),
        .cfg_s2_work_base(LAYOUT_Z1_BASE),
        .cfg_c_base(LAYOUT_C_INT_BASE),
        .cfg_dst_base(LAYOUT_S1_BASE),
        .mem_rd_en(ntt_mem_rd_en), .mem_rd_addr(ntt_mem_rd_addr),
        .mem_rd_data(mem_b_rd_data),
        .mem_wr_en(ntt_mem_wr_en), .mem_wr_addr(ntt_mem_wr_addr),
        .mem_wr_data(ntt_mem_wr_data),
        .twiddle_rom_addr(ntt_twiddle_rom_addr),
        .twiddle_rom_data(ntt_twiddle_rom_data),
        .psi_rom_addr(ntt_psi_rom_addr),
        .psi_rom_data(ntt_psi_rom_data));

    assign ntt_start = (st == FI) && (sn == N1);

    // ─── Norm check / rejection check ───
    wire        norm_start, norm_start_ready, norm_done, norm_accept, norm_fail;
    wire [7:0]  norm_status;
    wire        norm_mem_rd_en;
    wire [ADDR_W-1:0] norm_mem_rd_addr;
    wire [63:0] norm_sq;
    localparam [63:0] FALCON512_BOUND_SQ = 64'd34034726;
    localparam [7:0]  MAX_RESTARTS = 8'd3;

    falconsign_norm_i16_sig_check #(.ADDR_W(ADDR_W)) u_norm (
        .clk(clk),
        .rst_n(rst_n),
        .start(norm_start),
        .start_ready(norm_start_ready),
        .s2_base(LAYOUT_SIG_BASE),
        .s1_base(LAYOUT_S1_BASE),
        .word_count(LAYOUT_NORM_WORDS),
        .bound_sq(FALCON512_BOUND_SQ),
        .mem_rd_en(norm_mem_rd_en),
        .mem_rd_addr(norm_mem_rd_addr),
        .mem_rd_data(mem_b_rd_data),
        .done(norm_done),
        .accept(norm_accept),
        .fail(norm_fail),
        .status(norm_status),
        .norm_sq(norm_sq)
    );

    // ─── Restart / rejection ───
    reg [7:0] salt_cnt;      // incremented on each restart (RC reject)
    reg       rc_fail;       // RC rejection flag

    // ─── Config ───
    assign ts_cfg_depth   = FALCON_LOGN[LEVEL_W-1:0];
    assign ts_cfg_dynamic = cfg_dynamic_tree;
    assign ts_t_base      = LAYOUT_T_BASE;
    assign ts_tree_base   = LAYOUT_TREE_BASE;
    assign ts_z_base      = LAYOUT_Z_BASE;
    assign ts_tmp_base    = LAYOUT_TMP_BASE;
    assign ts_start       = (st != FS) && (sn == FS);
`ifndef SYNTHESIS
    integer debug_rng_nonce_base;
    initial begin
        debug_rng_nonce_base = 0;
        if (!$value$plusargs("RNG_NONCE=%d", debug_rng_nonce_base)) begin
            debug_rng_nonce_base = 0;
        end
    end
    wire [7:0] rng_nonce_lo = salt_cnt + debug_rng_nonce_base[7:0];
`else
    wire [7:0] rng_nonce_lo = salt_cnt;
`endif
    assign rng_seed_valid = (st == SH) || ((st == SI) && (sn == FS));
    assign rng_seed_key   = 256'h0123456789ABCDEF0123456789ABCDEF0123456789ABCDEF0123456789ABCDEF;
    assign rng_seed_nonce = {88'd0, rng_nonce_lo};
    assign sz_rng_data    = rng_block[255:0];
    assign sz_rng_ack     = rng_valid && sz_rng_req;
    assign rng_ready      = sz_rng_req;
    assign sz_cmd_sigma_min = 64'h3FF47201BF1F7A75; // fpr_sigma_min[9]
    assign busy           = (st != SI) && (st != SD);
    wire bus_is_reg = (bus_addr == REG_CR) ||
                      (bus_addr == REG_SR) ||
                      (bus_addr == REG_CFG) ||
                      (bus_addr == REG_MEM_HI);

    assign mem_c_en       = bus_cs && !bus_is_reg;
    assign mem_c_wr       = bus_wr;
    assign mem_c_addr     = {mem_addr_hi, bus_addr};
    assign mem_c_wr_data  = bus_wdata;

    // ─── phase control signals ───
    wire sh_done;  // SH: SHAKE absorb complete
    wire hp_done_sig; // HP: all N coeffs written

    // ─── FFT command ───
    assign fft_cmd_valid  = (st == FC) || (st == IV);
    assign fft_cmd_opcode = (st == IV) ? 3'd2 : 3'd0;  // FWD / Falcon half-complex INV
    assign fft_cmd_logn   = FALCON_LOGN;

    // ─── SHAKE256 absorb control (SH phase) ───
    // One-cycle pulse on SI→SH or RC→SH (restart) transition
    assign shake_start = ((st == SI) && (sn == SH)) || ((st == RC) && (sn == SH));

    // ─── HashToPoint control (HP phase) ───
    // One-cycle pulse on SH→HP transition
    assign htp_start = (st == SH) && (sn == HP);
    assign vd_start = ((st == FS) && (sn == VD)) ||
                      (cfg_bypass_fs && (st == FC) && (sn == VD)) ||
                      (cfg_start_at_fs && cfg_bypass_fs && (st == SI) && (sn == VD));
    assign fi_start = (st == IV) && (sn == FI);
    assign norm_start = (st == N1) && (sn == RC);

    // ─── SH / HP done detection ───
    // SH done: absorbed all 4 words AND SHAKE has processed them (back in ready state after last permutation)
    // We detect SH done when: word_idx == 4 (all words sent) AND shake is back in absorb-ready state
    // Simple approach: done when word_idx == 4 and shake_ready (it finishes padding permute)
    assign sh_done = (st == SH) && (sh_word_idx == 3'd4) && shake_ready;

    // HP done: all 32 writes completed (32 × 16 = 512 coefficients)
    assign hp_done_sig = (st == HP) && (hp_wr_addr == (LAYOUT_C_BASE + HTP_C_WORDS)) && !hp_wr_en;

    // ─── Bus & FSM ───
    reg bus_pend, bus_pend_wr; reg[15:0] bus_pend_addr; reg[31:0] bus_pend_data;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st <= SI; cr_start <= 0; bus_rdata <= 0; bus_ready <= 0; bus_irq <= 0;
            done <= 0; fail <= 0; status <= 0; bus_pend <= 0; bus_pend_wr <= 0;
            bus_pend_addr <= 0; bus_pend_data <= 0;
            cfg_bypass_fs <= 1'b0;
            cfg_force_accept <= 1'b0;
            cfg_start_at_fs <= 1'b0;
            cfg_dynamic_tree <= 1'b0;
            mem_addr_hi <= 2'd0;
            sh_word_idx <= 0;
            shake_absorb <= 0; shake_din <= 0; shake_din_last <= 0;
            hp_coeff_cnt <= 0; hp_coeff_buf <= 0;
            hp_wr_addr <= LAYOUT_C_BASE; hp_wr_en <= 0; hp_wr_data <= 0;
            hp_cint_wr_addr <= LAYOUT_C_INT_BASE; hp_cint_wr_en <= 0; hp_cint_wr_data <= 0;
            salt_cnt <= 0;
            rc_fail <= 0;
        end else begin
            st <= sn; bus_ready <= 0;
            if (bus_cs && !bus_pend) begin
                if (bus_is_reg) begin
                    bus_ready <= 1'b1;
                    if (bus_wr) begin
                        case (bus_addr)
                            REG_CR: begin
                                cr_start <= bus_wdata[0];
                                if (st == SD) bus_irq <= 1'b0;
                            end
                            REG_CFG: begin
                                cfg_bypass_fs    <= bus_wdata[0];
                                cfg_force_accept <= bus_wdata[1];
                                cfg_start_at_fs  <= bus_wdata[2];
                                cfg_dynamic_tree <= bus_wdata[3];
                            end
                            REG_MEM_HI: begin
                                mem_addr_hi <= bus_wdata[1:0];
                            end
                            default: begin
                            end
                        endcase
                    end else begin
                        case (bus_addr)
                            REG_CR:     bus_rdata <= {31'd0, cr_start};
                            REG_SR:     bus_rdata <= {16'd0, status, 4'd0, fail, done, bus_irq, busy};
                            REG_CFG:    bus_rdata <= {28'd0, cfg_dynamic_tree, cfg_start_at_fs,
                                                       cfg_force_accept, cfg_bypass_fs};
                            REG_MEM_HI: bus_rdata <= {30'd0, mem_addr_hi};
                            default:    bus_rdata <= 32'd0;
                        endcase
                    end
                end else begin
                    bus_pend <= 1; bus_pend_wr <= bus_wr;
                    bus_pend_addr <= bus_addr; bus_pend_data <= bus_wdata;
                end
            end
            if (bus_pend && mem_c_ready) begin
                bus_pend <= 0; bus_ready <= 1;
                if (!bus_pend_wr) bus_rdata <= mem_c_rd_data;
            end
            case (st)
                SI: begin done<=0; fail<=0; bus_irq<=0; if (sn != SI) cr_start<=0; end
                SD: begin done<=1; bus_irq<=1;
                    if (bus_cs && bus_wr && bus_addr==REG_CR) bus_irq<=0; end
            endcase

            // ─── SH phase: absorb test message into SHAKE256 ───
            if (st == SH && sn == SH) begin
                // Feed one 64-bit word per cycle
                if (sh_word_idx < 3'd4 && shake_ready) begin
                    shake_absorb <= 1;
                    shake_din_last <= (sh_word_idx == 3'd3); // last word
                    case (sh_word_idx)
                        3'd0: shake_din <= TEST_MSG_W0;
                        3'd1: shake_din <= TEST_MSG_W1;
                        3'd2: shake_din <= TEST_MSG_W2;
                        3'd3: shake_din <= TEST_MSG_W3;
                        default: shake_din <= 64'd0;
                    endcase
                    sh_word_idx <= sh_word_idx + 3'd1;
                end else if (sh_word_idx == 3'd4) begin
                    // All words fed; wait for SHAKE padding + permutation
                    shake_absorb <= 0;
                end
            end

            // HP phase: squeeze SHAKE -> HashToPoint -> write FFT-ready FP64 complex words.
            if (st == HP && sn == HP) begin
                shake_absorb <= 0;

                hp_wr_en <= 0;
                hp_cint_wr_en <= 0;
                if (htp_coeff_valid) begin
                    hp_wr_en   <= 1'b1;
                    hp_wr_data <= {128'd0, 64'd0, u16_to_f64(htp_coeff)};
                    hp_wr_addr <= hp_wr_addr + 1'b1;
                    if (hp_coeff_cnt == 4'd15) begin
                        hp_cint_wr_en   <= 1'b1;
                        hp_cint_wr_data <= {htp_coeff, hp_coeff_buf[239:0]};
                        hp_cint_wr_addr <= hp_cint_wr_addr + 1'b1;
                        hp_coeff_cnt    <= 4'd0;
                        hp_coeff_buf    <= 256'd0;
                    end else begin
                        hp_coeff_buf[(hp_coeff_cnt * 16) +: 16] <= htp_coeff;
                        hp_coeff_cnt <= hp_coeff_cnt + 1'b1;
                    end
                end
            end

            // ─── RC phase: determine pass/fail ───
            if (st == RC && norm_done) begin
                rc_fail <= cfg_force_accept ? 1'b0 : !norm_accept;
                status  <= cfg_force_accept ? 8'h00 : norm_status;
                if (!cfg_force_accept && !norm_accept && (salt_cnt >= MAX_RESTARTS)) begin
                    fail <= 1'b1;
                    status <= 8'h21;
                end
            end
            if (st == VD && vd_done && vd_fail) begin
                fail   <= 1'b1;
                status <= vd_status;
            end
            if (st == FS && ts_fail) begin
                fail   <= 1'b1;
                status <= ts_status;
            end
            if (st == FI && fi_done && fi_fail) begin
                fail   <= 1'b1;
                status <= fi_status;
            end

            // Reset HP counters on entry to SH (new signing operation or restart)
            if ((st == SI && sn == SH) || (st == RC && sn == SH)) begin
                hp_wr_addr   <= LAYOUT_C_BASE;
                hp_cint_wr_addr <= LAYOUT_C_INT_BASE;
                hp_coeff_cnt <= 0;
                hp_wr_en     <= 0;
                hp_cint_wr_en <= 0;
                hp_coeff_buf <= 0;
                sh_word_idx  <= 0;
                shake_absorb <= 0;
                if (st == RC && sn == SH) salt_cnt <= salt_cnt + 1;
            end
        end
    end

    // ─── Phase FSM (next state) ───
    always @(*) begin
        sn = st;
        case (st)
            SI: if (cr_start) sn = cfg_start_at_fs ? (cfg_bypass_fs ? VD : FS) : SH;
            SH: if (sh_done)  sn = HP;
            HP: if (hp_done_sig) sn = FC;
            FC: if (fft_rsp_done) sn = cfg_bypass_fs ? VD : FS;
            FS: if (ts_fail)     sn = SD;
                else if (ts_done) sn = VD;
            VD: if (vd_done)     sn = vd_fail ? SD : IV;
            IV: if (fft_rsp_done) sn = FI;
            FI: if (fi_done)     sn = fi_fail ? SD : N1;
            N1: if (ntt_done)    sn = ntt_fail ? SD : RC;
            RC: begin
                if (norm_done) begin
                    if (cfg_force_accept) begin
                        sn = CN;
                    end else if (!norm_accept && (salt_cnt >= MAX_RESTARTS)) begin
                        sn = SD;
                    end else if (!norm_accept) begin
                        sn = SH;  // restart with new salt
                    end else begin
                        sn = CN;
                    end
                end
            end
            CN: if (skel_timer > 8'd5) sn = EN;
            EN: if (skel_timer > 8'd5) sn = OU;
            OU: if (skel_timer > 8'd3) sn = SD;
            SD: if (bus_cs && bus_wr && (bus_addr == REG_CR)) sn = SI;
            default: sn = SI;
        endcase
    end

    reg [7:0] skel_timer;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) skel_timer <= 0;
        else if (st != sn) skel_timer <= 0;
        else skel_timer <= skel_timer + 1;
    end

endmodule
