// Board-specific PG2L100H DDR3 IP adapter. The user interface is in core_clk.
// Addresses and burst units are those of the generated ddr3_test IP, not AXI4.
module ddr3_bridge (
    input  wire        clk_p,
    input  wire        clk_n,
    input  wire        reset_n,
    output wire        core_clk,
    output wire        init_done,
    input  wire [27:0] awaddr,
    input  wire [3:0]  awlen,
    input  wire        awvalid,
    output wire        awready,
    input  wire [255:0] wdata,
    input  wire [31:0] wstrb,
    output wire        wready,
    output wire        wlast,
    input  wire [27:0] araddr,
    input  wire [3:0]  arlen,
    input  wire        arvalid,
    output wire        arready,
    output wire [255:0] rdata,
    output wire        rvalid,
    output wire        rlast,
    output wire        mem_rst_n, mem_ck, mem_ck_n, mem_cke,
    output wire        mem_cs_n, mem_ras_n, mem_cas_n, mem_we_n, mem_odt,
    output wire [14:0] mem_a,
    output wire [2:0]  mem_ba,
    inout  wire [3:0]  mem_dqs, mem_dqs_n,
    inout  wire [31:0] mem_dq,
    output wire [3:0]  mem_dm
);
    wire clk_125m;
    GTP_INBUFGDS #(.IOSTANDARD("DEFAULT"), .TERM_DIFF("ON")) u_refbuf (
        .I(clk_p), .IB(clk_n), .O(clk_125m)
    );
    ddr3_test u_ddr (
        .ref_clk(clk_125m), .resetn(reset_n), .ddr_init_done(init_done),
        .core_clk(core_clk),
        .axi_awaddr(awaddr), .axi_awuser_ap(1'b0), .axi_awuser_id(4'd0),
        .axi_awlen(awlen), .axi_awvalid(awvalid), .axi_awready(awready),
        .axi_wdata(wdata), .axi_wstrb(wstrb), .axi_wready(wready),
        .axi_wusero_last(wlast),
        .axi_araddr(araddr), .axi_aruser_ap(1'b0), .axi_aruser_id(4'd0),
        .axi_arlen(arlen), .axi_arvalid(arvalid), .axi_arready(arready),
        .axi_rdata(rdata), .axi_rvalid(rvalid), .axi_rlast(rlast),
        .apb_clk(1'b0), .apb_rst_n(1'b1), .apb_sel(1'b0),
        .apb_enable(1'b0), .apb_addr(8'd0), .apb_write(1'b0),
        .apb_wdata(16'd0),
        .mem_rst_n(mem_rst_n), .mem_ck(mem_ck), .mem_ck_n(mem_ck_n),
        .mem_cke(mem_cke), .mem_cs_n(mem_cs_n), .mem_ras_n(mem_ras_n),
        .mem_cas_n(mem_cas_n), .mem_we_n(mem_we_n), .mem_odt(mem_odt),
        .mem_a(mem_a), .mem_ba(mem_ba), .mem_dqs(mem_dqs),
        .mem_dqs_n(mem_dqs_n), .mem_dq(mem_dq), .mem_dm(mem_dm),
        .dbg_gate_start(1'b0), .dbg_cpd_start(1'b0), .dbg_ddrphy_rst_n(1'b1),
        .dbg_gpll_scan_rst(1'b0), .samp_position_dyn_adj(1'b0),
        .init_samp_position_even(32'd0), .init_samp_position_odd(32'd0),
        .wrcal_position_dyn_adj(1'b0), .init_wrcal_position(32'd0),
        .force_read_clk_ctrl(1'b0), .init_slip_step(16'd0),
        .init_read_clk_ctrl(12'd0), .debug_cpd_offset_adj(1'b0),
        .debug_cpd_offset_dir(1'b0), .debug_cpd_offset(10'd0),
        .ck_dly_en(1'b0), .init_ck_dly_step(8'd0)
    );
endmodule
