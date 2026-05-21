`timescale 1ns/1ps

module tb_falconsign_norm_official_c;
    localparam ADDR_W = 11;
    localparam N_WORDS = 32;
    localparam S2_BASE = 0;
    localparam S1_BASE = 32;

    reg clk;
    reg rst_n;
    reg start;
    wire start_ready;
    wire mem_rd_en;
    wire [ADDR_W-1:0] mem_rd_addr;
    reg [255:0] mem_rd_data;
    wire done;
    wire accept;
    wire fail;
    wire [7:0] status;
    wire [63:0] norm_sq;

    reg [255:0] mem [0:63];
    reg [15:0] coeff [0:511];
    integer i;
    integer lane;
    integer errors;
    reg [63:0] expected_norm;
    reg signed [31:0] centered;
    reg [31:0] abs_centered;

    falconsign_norm_i16_sig_check #(.ADDR_W(ADDR_W)) dut (
        .clk(clk),
        .rst_n(rst_n),
        .start(start),
        .start_ready(start_ready),
        .s2_base(S2_BASE[ADDR_W-1:0]),
        .s1_base(S1_BASE[ADDR_W-1:0]),
        .word_count(N_WORDS[ADDR_W-1:0]),
        .bound_sq(64'hFFFF_FFFF_FFFF_FFFF),
        .mem_rd_en(mem_rd_en),
        .mem_rd_addr(mem_rd_addr),
        .mem_rd_data(mem_rd_data),
        .done(done),
        .accept(accept),
        .fail(fail),
        .status(status),
        .norm_sq(norm_sq)
    );

    always #5 clk = ~clk;

    always @(posedge clk) begin
        if (mem_rd_en)
            mem_rd_data <= mem[mem_rd_addr];
    end

    initial begin
        clk = 1'b0;
        rst_n = 1'b0;
        start = 1'b0;
        mem_rd_data = 256'd0;
        errors = 0;
        expected_norm = 64'd0;

        $readmemh("SRC/tb/falcon512_kat0_htp_expected.hex", coeff);

        for (i = 0; i < 64; i = i + 1)
            mem[i] = 256'd0;

        for (i = 0; i < N_WORDS; i = i + 1) begin
            mem[S2_BASE + i] = 256'd0;
            mem[S1_BASE + i] = 256'd0;
            for (lane = 0; lane < 16; lane = lane + 1) begin
                mem[S1_BASE + i][lane*16 +: 16] = coeff[i*16 + lane];
                centered = coeff[i*16 + lane];
                if (centered > 6144)
                    centered = centered - 12289;
                abs_centered = centered < 0 ? -centered : centered;
                expected_norm = expected_norm + (abs_centered * abs_centered);
            end
        end

        repeat (5) @(posedge clk);
        rst_n = 1'b1;
        repeat (5) @(posedge clk);

        if (!start_ready) begin
            $display("NORM OFFICIAL C FAILED: start_ready=0");
            $finish;
        end

        start <= 1'b1;
        @(posedge clk);
        start <= 1'b0;

        wait(done || fail);

        if (fail) begin
            $display("NORM OFFICIAL C FAILED: status=%02x", status);
            $finish;
        end

        if (norm_sq !== expected_norm) begin
            $display("NORM OFFICIAL C mismatch: got=%0d expected=%0d", norm_sq, expected_norm);
            errors = errors + 1;
        end
        if (!accept) begin
            $display("NORM OFFICIAL C mismatch: accept=0 with max bound");
            errors = errors + 1;
        end

        if (errors == 0)
            $display("NORM OFFICIAL C PASSED: centered norm_sq=%0d for official c, s2=0", norm_sq);
        else
            $display("NORM OFFICIAL C FAILED: %0d errors", errors);

        #20;
        $finish;
    end

    initial begin
        #1000000;
        $display("NORM OFFICIAL C TIMEOUT");
        $finish;
    end
endmodule
