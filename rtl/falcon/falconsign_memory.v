`timescale 1ns/1ps

module falconsign_memory #(
    parameter ADDR_W = 10,
    parameter BANK_W = 256,
    parameter DEPTH  = 1024
) (
    input  wire        clk,
    input  wire        rst_n,

    // Port A: BFU/FFT access (2-read, 2-write per cycle)
    input  wire        port_a_rd_en,
    input  wire [ADDR_W-1:0] port_a_rd_addr0,
    input  wire [ADDR_W-1:0] port_a_rd_addr1,
    output wire [BANK_W-1:0] port_a_rd_data0,
    output wire [BANK_W-1:0] port_a_rd_data1,

    input  wire        port_a_wr_en,
    input  wire [ADDR_W-1:0] port_a_wr_addr0,
    input  wire [ADDR_W-1:0] port_a_wr_addr1,
    input  wire [BANK_W-1:0] port_a_wr_data0,
    input  wire [BANK_W-1:0] port_a_wr_data1,

    // Port B: SamplerZ / HashToPoint / Encoder access (1-read, 1-write per cycle)
    input  wire        port_b_rd_en,
    input  wire [ADDR_W-1:0] port_b_rd_addr,
    output wire [BANK_W-1:0] port_b_rd_data,

    input  wire        port_b_wr_en,
    input  wire [ADDR_W-1:0] port_b_wr_addr,
    input  wire [BANK_W-1:0] port_b_wr_data,

    // Port C: Bus interface (32-bit access, byte-addressable)
    input  wire        port_c_en,
    input  wire        port_c_wr,
    input  wire [ADDR_W+4:0] port_c_addr,
    input  wire [31:0] port_c_wr_data,
    output wire [31:0] port_c_rd_data,
    output wire        port_c_ready
);

    localparam BANK_SEL_W = 2;
    localparam BANK_ADDR_W = ADDR_W - BANK_SEL_W;

    // 4 banks × (DEPTH/4) entries × 256 bits
    reg [BANK_W-1:0] bank0 [0:(DEPTH/4)-1];
    reg [BANK_W-1:0] bank1 [0:(DEPTH/4)-1];
    reg [BANK_W-1:0] bank2 [0:(DEPTH/4)-1];
    reg [BANK_W-1:0] bank3 [0:(DEPTH/4)-1];

    integer init_i;
    initial begin
        for (init_i = 0; init_i < (DEPTH/4); init_i = init_i + 1) begin
            bank0[init_i] = {BANK_W{1'b0}};
            bank1[init_i] = {BANK_W{1'b0}};
            bank2[init_i] = {BANK_W{1'b0}};
            bank3[init_i] = {BANK_W{1'b0}};
        end
    end

    // Port A: full 256-bit, 4-bank parallel access
    wire [1:0] a_bank0_sel = port_a_rd_addr0[1:0];
    wire [1:0] a_bank1_sel = port_a_rd_addr1[1:0];
    wire [BANK_ADDR_W-1:0] a_addr0 = port_a_rd_addr0[ADDR_W-1:2];
    wire [BANK_ADDR_W-1:0] a_addr1 = port_a_rd_addr1[ADDR_W-1:2];

    // Port B: full 256-bit, single bank access
    wire [1:0] b_bank_sel = port_b_rd_addr[1:0];
    wire [BANK_ADDR_W-1:0] b_addr = port_b_rd_addr[ADDR_W-1:2];

    // Port C: 32-bit byte lane access
    wire [1:0] c_bank_sel = port_c_addr[ADDR_W+2:ADDR_W+1];
    wire [BANK_ADDR_W-1:0] c_addr = port_c_addr[ADDR_W-1:2];
    wire [2:0] c_byte_lane = port_c_addr[4:2];  // which 32-bit word in 256-bit bank

    // ─── Port A read (combinational, 1 cycle) ───
    reg [BANK_W-1:0] a_rd0, a_rd1;
    always @(*) begin
        case (a_bank0_sel)
            2'd0: a_rd0 = bank0[a_addr0];
            2'd1: a_rd0 = bank1[a_addr0];
            2'd2: a_rd0 = bank2[a_addr0];
            2'd3: a_rd0 = bank3[a_addr0];
        endcase
        case (a_bank1_sel)
            2'd0: a_rd1 = bank0[a_addr1];
            2'd1: a_rd1 = bank1[a_addr1];
            2'd2: a_rd1 = bank2[a_addr1];
            2'd3: a_rd1 = bank3[a_addr1];
        endcase
    end
    assign port_a_rd_data0 = a_rd0;
    assign port_a_rd_data1 = a_rd1;

    // Port A write
    wire [1:0] a_wr_bank0 = port_a_wr_addr0[1:0];
    wire [1:0] a_wr_bank1 = port_a_wr_addr1[1:0];
    wire [BANK_ADDR_W-1:0] a_wr_addr0 = port_a_wr_addr0[ADDR_W-1:2];
    wire [BANK_ADDR_W-1:0] a_wr_addr1 = port_a_wr_addr1[ADDR_W-1:2];
    always @(posedge clk) begin
        if (port_a_wr_en) begin
            case (a_wr_bank0)
                2'd0: bank0[a_wr_addr0] <= port_a_wr_data0;
                2'd1: bank1[a_wr_addr0] <= port_a_wr_data0;
                2'd2: bank2[a_wr_addr0] <= port_a_wr_data0;
                2'd3: bank3[a_wr_addr0] <= port_a_wr_data0;
            endcase
            case (a_wr_bank1)
                2'd0: bank0[a_wr_addr1] <= port_a_wr_data1;
                2'd1: bank1[a_wr_addr1] <= port_a_wr_data1;
                2'd2: bank2[a_wr_addr1] <= port_a_wr_data1;
                2'd3: bank3[a_wr_addr1] <= port_a_wr_data1;
            endcase
        end
    end

    // ─── Port B read ───
    reg [BANK_W-1:0] b_rd;
    initial b_rd = {BANK_W{1'b0}};
    always @(posedge clk) begin
        if (port_b_rd_en) begin
            case (b_bank_sel)
                2'd0: b_rd <= bank0[b_addr];
                2'd1: b_rd <= bank1[b_addr];
                2'd2: b_rd <= bank2[b_addr];
                2'd3: b_rd <= bank3[b_addr];
            endcase
        end
    end
    assign port_b_rd_data = b_rd;

    // Port B write
    wire [1:0] b_wr_bank = port_b_wr_addr[1:0];
    wire [BANK_ADDR_W-1:0] b_wr_addr = port_b_wr_addr[ADDR_W-1:2];
    always @(posedge clk) begin
        if (port_b_wr_en) begin
            case (b_wr_bank)
                2'd0: bank0[b_wr_addr] <= port_b_wr_data;
                2'd1: bank1[b_wr_addr] <= port_b_wr_data;
                2'd2: bank2[b_wr_addr] <= port_b_wr_data;
                2'd3: bank3[b_wr_addr] <= port_b_wr_data;
            endcase
        end
    end

    // ─── Port C: 32-bit bus access ───
    reg [31:0] c_rd_data;
    reg        c_ready;
    initial begin
        c_rd_data = 32'd0;
        c_ready = 1'b0;
    end
    always @(posedge clk) begin
        c_ready <= port_c_en;
        if (port_c_en) begin
            if (port_c_wr) begin
                case (c_bank_sel)
                    2'd0: bank0[c_addr][c_byte_lane*32 +: 32] <= port_c_wr_data;
                    2'd1: bank1[c_addr][c_byte_lane*32 +: 32] <= port_c_wr_data;
                    2'd2: bank2[c_addr][c_byte_lane*32 +: 32] <= port_c_wr_data;
                    2'd3: bank3[c_addr][c_byte_lane*32 +: 32] <= port_c_wr_data;
                endcase
            end else begin
                case (c_bank_sel)
                    2'd0: c_rd_data <= bank0[c_addr][c_byte_lane*32 +: 32];
                    2'd1: c_rd_data <= bank1[c_addr][c_byte_lane*32 +: 32];
                    2'd2: c_rd_data <= bank2[c_addr][c_byte_lane*32 +: 32];
                    2'd3: c_rd_data <= bank3[c_addr][c_byte_lane*32 +: 32];
                endcase
            end
        end
    end
    assign port_c_rd_data = c_rd_data;
    assign port_c_ready = c_ready;

endmodule
