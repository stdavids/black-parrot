`include "bp_common_defines.svh"
`include "bp_me_defines.svh"
`include "bsg_cache.vh"
`include "bsg_noc_links.vh"
`include "bp_top_defines.svh"

module bp_dma_engine
    import bp_be_pkg::*;
    import bp_common_pkg::*;
    import bp_me_pkg::*;
    import bp_top_pkg::*;
    import bsg_cache_pkg::*;
    import bsg_noc_pkg::*;
    import bsg_wormhole_router_pkg::*;

#(  parameter bp_params_e bp_params_p = e_bp_default_cfg
    `declare_bp_proc_params(bp_params_p)
    `declare_bp_bedrock_if_widths(paddr_width_p, lce_id_width_p, cce_id_width_p, did_width_p, lce_assoc_p)
)

(   input  logic                                clk_i
,   input  logic                                reset_i

,   input  logic [mem_fwd_header_width_lp-1:0]  dev_fwd_header_i
,   input  logic [bedrock_fill_width_p-1:0]     dev_fwd_data_i
,   input  logic                                dev_fwd_v_i
,   output logic                                dev_fwd_ready_and_o

,   output logic [mem_rev_header_width_lp-1:0]  dev_rev_header_o
,   output logic [bedrock_fill_width_p-1:0]     dev_rev_data_o
,   output logic                                dev_rev_v_o
,   input  logic                                dev_rev_ready_and_i

,   input  logic [lce_id_width_p-1:0]           proc_rd_lce_id_i
,   output logic [mem_fwd_header_width_lp-1:0]  proc_rd_fwd_header_o
,   output logic [bedrock_fill_width_p-1:0]     proc_rd_fwd_data_o
,   output logic                                proc_rd_fwd_v_o
,   input                                       proc_rd_fwd_ready_and_i

,   input [mem_rev_header_width_lp-1:0]         proc_rd_rev_header_i
,   input [bedrock_fill_width_p-1:0]            proc_rd_rev_data_i
,   input                                       proc_rd_rev_v_i
,   output logic                                proc_rd_rev_ready_and_o

,   input  logic [lce_id_width_p-1:0]           proc_wr_lce_id_i
,   output logic [mem_fwd_header_width_lp-1:0]  proc_wr_fwd_header_o
,   output logic [bedrock_fill_width_p-1:0]     proc_wr_fwd_data_o
,   output logic                                proc_wr_fwd_v_o
,   input                                       proc_wr_fwd_ready_and_i

,   input [mem_rev_header_width_lp-1:0]         proc_wr_rev_header_i
,   input [bedrock_fill_width_p-1:0]            proc_wr_rev_data_i
,   input                                       proc_wr_rev_v_i
,   output logic                                proc_wr_rev_ready_and_o
);

    `declare_bp_bedrock_if(paddr_width_p, lce_id_width_p, cce_id_width_p, did_width_p, lce_assoc_p);

    localparam block_size_in_fill_lp = dcache_block_width_p / bedrock_fill_width_p;
    localparam fill_cnt_width_lp = `BSG_SAFE_CLOG2(block_size_in_fill_lp);

    logic [dev_addr_width_gp-1:0] csr_addr_lo;
    logic [63:0] csr_data_lo;
    logic [63:0] csr_data_li;
    logic csr_w_v_lo;
    logic csr_r_v_lo;

    logic [paddr_width_p-1:0] c_rd_addr_lo;
    logic c_rd_v_lo;

    logic [bedrock_fill_width_p-1:0] c_rd_data_li;
    logic c_rd_v_li;

    logic [paddr_width_p-1:0] c_wr_addr_lo;
    logic [bedrock_fill_width_p-1:0] c_wr_data_lo;
    logic c_wr_v_lo;

    bp_bedrock_mem_fwd_header_s fsm_rd_fwd_header_li;
    bp_bedrock_mem_fwd_header_s fsm_wr_fwd_header_li;
    bp_bedrock_mem_rev_header_s fsm_rd_rev_header_lo;

    logic fsm_rd_fwd_ready_lo;
    logic fsm_wr_fwd_ready_lo;
    logic fsm_wr_rev_v_lo;

    always_comb begin
        fsm_rd_fwd_header_li = '0;
        fsm_rd_fwd_header_li.msg_type       = e_bedrock_mem_uc_rd;
        fsm_rd_fwd_header_li.addr           = c_rd_addr_lo;
        fsm_rd_fwd_header_li.size           = e_bedrock_msg_size_16; // 128b
        fsm_rd_fwd_header_li.payload.lce_id = proc_rd_lce_id_i;
    end

    always_comb begin
        fsm_wr_fwd_header_li = '0;
        fsm_wr_fwd_header_li.msg_type       = e_bedrock_mem_uc_wr;
        fsm_wr_fwd_header_li.addr           = c_wr_addr_lo;
        fsm_wr_fwd_header_li.size           = e_bedrock_msg_size_16; // 128b
        fsm_wr_fwd_header_li.payload.lce_id = proc_wr_lce_id_i;
        fsm_wr_fwd_header_li.subop          = e_bedrock_store;
    end

    bp_me_bedrock_register #
        (.bp_params_p(bp_params_p)
        ,.els_p(1)
        ,.reg_addr_width_p(dev_addr_width_gp)
        ,.base_addr_p(dev_addr_width_gp'('h?_????)))
    register
        (.clk_i(clk_i)
        ,.reset_i(reset_i)

        ,.mem_fwd_header_i(dev_fwd_header_i)
        ,.mem_fwd_data_i(dev_fwd_data_i)
        ,.mem_fwd_v_i(dev_fwd_v_i)
        ,.mem_fwd_ready_and_o(dev_fwd_ready_and_o)

        ,.mem_rev_header_o(dev_rev_header_o)
        ,.mem_rev_data_o(dev_rev_data_o)
        ,.mem_rev_v_o(dev_rev_v_o)
        ,.mem_rev_ready_and_i(dev_rev_ready_and_i)

        ,.addr_o(csr_addr_lo)
        ,.size_o()
        ,.data_o(csr_data_lo)
        ,.w_v_o(csr_w_v_lo)
        ,.r_v_o(csr_r_v_lo)

        ,.data_i(csr_data_li)
        );

    bsg_mla_dma_controller #
        (.p_addr_width_p(dev_addr_width_gp)
        ,.p_data_width_p(64)
        ,.c_addr_width_p(paddr_width_p)
        ,.c_data_width_p(bedrock_fill_width_p)
        ,.c_mask_width_p(bedrock_fill_width_p / 8)
        ,.out_of_order_p(1)
        ,.st_fwd_fifo_els_p(16))
    dma_unit
        (.clk_i(clk_i)
        ,.reset_i(reset_i)

        ,.p_addr_i(csr_addr_lo)
        ,.p_data_i(csr_data_lo)
        ,.p_w_i(csr_w_v_lo)
        ,.p_v_i(csr_w_v_lo | csr_r_v_lo)
        ,.p_yumi_o()

        ,.p_data_o(csr_data_li)
        ,.p_v_o()

        ,.c_rd_addr_o(c_rd_addr_lo)
        ,.c_rd_v_o(c_rd_v_lo)
        ,.c_rd_yumi_i(c_rd_v_lo & fsm_rd_fwd_ready_lo)

        ,.c_rd_addr_i(fsm_rd_rev_header_lo.addr)
        ,.c_rd_data_i(c_rd_data_li)
        ,.c_rd_v_i(c_rd_v_li)

        ,.c_wr_addr_o(c_wr_addr_lo)
        ,.c_wr_data_o(c_wr_data_lo)
        ,.c_wr_mask_o()
        ,.c_wr_v_o(c_wr_v_lo)
        ,.c_wr_yumi_i(c_wr_v_lo & fsm_wr_fwd_ready_lo)

        ,.c_wr_ack_i(fsm_wr_rev_v_lo)

        ,.interrupt_o()
        );

    bp_me_stream_pump_out #
        (.bp_params_p(bp_params_p)
        ,.fsm_data_width_p(bedrock_fill_width_p)
        ,.block_width_p(dcache_block_width_p)
        ,.payload_width_p(mem_fwd_payload_width_lp)
        ,.msg_stream_mask_p(mem_fwd_stream_mask_gp)
        ,.fsm_stream_mask_p(mem_fwd_stream_mask_gp))
    rd_pump_out
        (.clk_i(clk_i)
        ,.reset_i(reset_i)

        ,.msg_header_o(proc_rd_fwd_header_o)
        ,.msg_data_o(proc_rd_fwd_data_o)
        ,.msg_v_o(proc_rd_fwd_v_o)
        ,.msg_ready_and_i(proc_rd_fwd_ready_and_i)

        ,.fsm_header_i(fsm_rd_fwd_header_li)
        ,.fsm_data_i('0)
        ,.fsm_addr_o()
        ,.fsm_v_i(c_rd_v_lo)
        ,.fsm_ready_and_o(fsm_rd_fwd_ready_lo)

        ,.fsm_new_o()
        ,.fsm_critical_o()
        ,.fsm_last_o()
        );

    bp_me_stream_pump_in #
        (.bp_params_p(bp_params_p)
        ,.fsm_data_width_p(bedrock_fill_width_p)
        ,.block_width_p(dcache_block_width_p)
        ,.payload_width_p(mem_rev_payload_width_lp)
        ,.msg_stream_mask_p(mem_rev_stream_mask_gp)
        ,.fsm_stream_mask_p(mem_rev_stream_mask_gp))
    rd_pump_in
        (.clk_i(clk_i)
        ,.reset_i(reset_i)

        ,.msg_header_i(proc_rd_rev_header_i)
        ,.msg_data_i(proc_rd_rev_data_i)
        ,.msg_v_i(proc_rd_rev_v_i)
        ,.msg_ready_and_o(proc_rd_rev_ready_and_o)

        ,.fsm_header_o(fsm_rd_rev_header_lo)
        ,.fsm_addr_o()
        ,.fsm_data_o(c_rd_data_li)
        ,.fsm_v_o(c_rd_v_li)
        ,.fsm_yumi_i(c_rd_v_li)

        ,.fsm_new_o()
        ,.fsm_critical_o()
        ,.fsm_last_o()
        );

    bp_me_stream_pump_out #
        (.bp_params_p(bp_params_p)
        ,.fsm_data_width_p(bedrock_fill_width_p)
        ,.block_width_p(dcache_block_width_p)
        ,.payload_width_p(mem_fwd_payload_width_lp)
        ,.msg_stream_mask_p(mem_fwd_stream_mask_gp)
        ,.fsm_stream_mask_p(mem_fwd_stream_mask_gp))
    wr_pump_out
        (.clk_i(clk_i)
        ,.reset_i(reset_i)

        ,.msg_header_o(proc_wr_fwd_header_o)
        ,.msg_data_o(proc_wr_fwd_data_o)
        ,.msg_v_o(proc_wr_fwd_v_o)
        ,.msg_ready_and_i(proc_wr_fwd_ready_and_i)

        ,.fsm_header_i(fsm_wr_fwd_header_li)
        ,.fsm_data_i(c_wr_data_lo)
        ,.fsm_addr_o()
        ,.fsm_v_i(c_wr_v_lo)
        ,.fsm_ready_and_o(fsm_wr_fwd_ready_lo)

        ,.fsm_new_o()
        ,.fsm_critical_o()
        ,.fsm_last_o()
        );

    bp_me_stream_pump_in #
        (.bp_params_p(bp_params_p)
        ,.fsm_data_width_p(bedrock_fill_width_p)
        ,.block_width_p(dcache_block_width_p)
        ,.payload_width_p(mem_rev_payload_width_lp)
        ,.msg_stream_mask_p(mem_rev_stream_mask_gp)
        ,.fsm_stream_mask_p(mem_rev_stream_mask_gp))
    wr_pump_in
        (.clk_i(clk_i)
        ,.reset_i(reset_i)

        ,.msg_header_i(proc_wr_rev_header_i)
        ,.msg_data_i(proc_wr_rev_data_i)
        ,.msg_v_i(proc_wr_rev_v_i)
        ,.msg_ready_and_o(proc_wr_rev_ready_and_o)

        ,.fsm_header_o()
        ,.fsm_addr_o()
        ,.fsm_data_o()
        ,.fsm_v_o(fsm_wr_rev_v_lo)
        ,.fsm_yumi_i(fsm_wr_rev_v_lo)

        ,.fsm_new_o()
        ,.fsm_critical_o()
        ,.fsm_last_o()
        );

endmodule // bsg_dma_engine
