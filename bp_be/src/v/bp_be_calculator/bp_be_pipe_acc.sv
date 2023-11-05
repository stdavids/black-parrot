`include "bp_common_defines.svh"
`include "bp_be_defines.svh"
`include "bp_me_defines.svh"
`include "bsg_mla_csr_pkg.svh"

module bp_be_pipe_acc
    import bp_common_pkg::*;
    import bp_be_pkg::*;
    import bp_me_pkg::*;
    import bsg_mla_csr_pkg::*;

#(  parameter bp_params_e bp_params_p = e_bp_default_cfg

    `declare_bp_proc_params(bp_params_p)
    `declare_bp_bedrock_if_widths(paddr_width_p, lce_id_width_p, cce_id_width_p, did_width_p, lce_assoc_p)

,   localparam dispatch_pkt_width_lp = `bp_be_dispatch_pkt_width(vaddr_width_p)
,   localparam commit_pkt_width_lp   = `bp_be_commit_pkt_width(vaddr_width_p, paddr_width_p)
)

(   input                                       clk_i
,   input                                       reset_i

,   input  logic [instr_width_gp-1:0]           instr_i
,   input  logic [dpath_width_gp-1:0]           data_i
,   input  logic                                v_i

,   input [dcache_block_width_p-1:0]            wide_data_i
,   input                                       wide_v_i

,   output logic                                busy_o
,   output logic                                panic_o

,   input [lce_id_width_p-1:0]                  lce_id_i

,   output logic [mem_fwd_header_width_lp-1:0]  mem_fwd_header_o
,   output logic [bedrock_fill_width_p-1:0]     mem_fwd_data_o
,   output logic                                mem_fwd_v_o
,   input                                       mem_fwd_ready_and_i

,   input [mem_rev_header_width_lp-1:0]         mem_rev_header_i
,   input [bedrock_fill_width_p-1:0]            mem_rev_data_i
,   input                                       mem_rev_v_i
,   output logic                                mem_rev_ready_and_o
);

    `declare_bp_bedrock_if(paddr_width_p, lce_id_width_p, cce_id_width_p, did_width_p, lce_assoc_p);

    bp_bedrock_mem_fwd_header_s fsm_fwd_header_li;
    logic [3:0][31:0] fsm_fwd_data_li;
    logic fsm_fwd_v_li, fsm_fwd_ready_and_lo;
    logic fsm_rev_v_lo;
    logic [dpath_width_gp-1:0] dest_r;
    logic dest_fifo_ready_lo;

    logic fsm_last_lo;

    logic [1:0] dpu_op;

    wire rv64_instr_itype_s instr = instr_i;

    typedef struct packed {
        logic wide_op;
        logic act_not_wt_op;
        logic last_op;
        logic set_csr;
        logic get_csr;
    } bp_be_pipe_acc_decode_s;

    bp_be_pipe_acc_decode_s decode;

    always_comb begin
        decode = '0;
        if (v_i) begin
            unique casez (instr)
                `RV64_TENSOR_ACLD0: begin
                    decode.wide_op = 1'b1;
                    decode.act_not_wt_op = 1'b1;
                end
                `RV64_TENSOR_ACLD1: begin
                    decode.wide_op = 1'b1;
                    decode.act_not_wt_op = 1'b1;
                    decode.last_op = 1'b1;
                end
                `RV64_TENSOR_WTLD0: begin
                    decode.wide_op = 1'b1;
                    decode.act_not_wt_op = 1'b0;
                end
                `RV64_TENSOR_WTLD1: begin
                    decode.wide_op = 1'b1;
                    decode.act_not_wt_op = 1'b0;
                    decode.last_op = 1'b1;
                end
                `RV64_TENSOR_CSRLD: begin
                    decode.get_csr = 1'b1;
                end
                `RV64_TENSOR_CSRST: begin
                    decode.set_csr = 1'b1;
                    // instr.imm12
                end
                default: begin
                end
            endcase
        end
    end

    /**
     * DESTINATION CSR
     */

    bsg_fifo_1r1w_small #(.width_p(dpath_width_gp),.els_p(2))
      dest_fifo
        (.clk_i(clk_i)
        ,.reset_i(reset_i)

        ,.data_i(data_i)
        ,.v_i(decode.set_csr)
        ,.ready_o(dest_fifo_ready_lo)

        ,.data_o(dest_r)
        ,.v_o(dest_v_lo)
        ,.yumi_i(fsm_fwd_v_li & fsm_fwd_ready_and_lo & fsm_last_lo)
        );

    /**
     * WIDE DATA HANDLING
     */

    logic wide_miss_queue_ready_lo, wide_miss_queue_v_lo, wide_miss_queue_last_op;
    logic wide_hit_under_miss_queue_ready_lo, wide_hit_under_miss_queue_v_lo, wide_hit_under_miss_queue_last_op;
    logic [511:0] wide_hit_under_miss_queue_data;

    wire wide_hit  = v_i & decode.wide_op & wide_v_i;
    wire wide_miss = v_i & decode.wide_op & ~wide_v_i;

    wire wide_hit_under_miss = wide_hit &  wide_miss_queue_v_lo;
    wire wide_hit_no_miss    = wide_hit & ~wide_miss_queue_v_lo;
    wire wide_late           = wide_v_i & ~(v_i & decode.wide_op);

    wire hit_under_miss_reordered = ~wide_miss_queue_v_lo & wide_hit_under_miss_queue_v_lo;

    bsg_fifo_1r1w_small #(.width_p(2),.els_p(32))
      wide_miss_queue
        (.clk_i(clk_i)
        ,.reset_i(reset_i)

        ,.data_i({decode.act_not_wt_op, decode.last_op})
        ,.v_i(wide_miss)
        ,.ready_o(wide_miss_queue_ready_lo)

        ,.data_o({wide_miss_queue_act_not_wt_op, wide_miss_queue_last_op})
        ,.v_o(wide_miss_queue_v_lo)
        ,.yumi_i(wide_miss_queue_v_lo & wide_late)
        );

    // synopsys translate_off
    always_ff @(posedge clk_i) begin
        if (wide_hit_under_miss & ~wide_hit_under_miss_queue_ready_lo)
            $display("ERROR: hit under miss queue overflow");
    end
    // synopsys translate_on

    bsg_fifo_1r1w_small #(.width_p(2 + 512),.els_p(4))
      wide_hit_under_miss_queue
        (.clk_i(clk_i)
        ,.reset_i(reset_i)

        ,.data_i({decode.act_not_wt_op, decode.last_op, wide_data_i})
        ,.v_i(wide_hit_under_miss)
        ,.ready_o(wide_hit_under_miss_queue_ready_lo)

        ,.data_o({wide_hit_under_miss_queue_act_not_wt_op, wide_hit_under_miss_queue_last_op, wide_hit_under_miss_queue_data})
        ,.v_o(wide_hit_under_miss_queue_v_lo)
        ,.yumi_i(hit_under_miss_reordered)
        );

    // synopsys translate_off
    always_ff @(posedge clk_i) begin
        if (wide_miss & ~wide_miss_queue_ready_lo)
            $display("ERROR: miss queue overflow");
    end
    // synopsys translate_on

    wire wide_act_not_wt_n = hit_under_miss_reordered ? wide_hit_under_miss_queue_act_not_wt_op
                           :                wide_late ? wide_miss_queue_act_not_wt_op
                           :                            decode.act_not_wt_op;

    wire wide_last_n = hit_under_miss_reordered ? wide_hit_under_miss_queue_last_op
                     :                wide_late ? wide_miss_queue_last_op
                     :                            decode.last_op;

    wire [511:0] wide_data_n = hit_under_miss_reordered ? wide_hit_under_miss_queue_data
                             :                            wide_data_i;

    wire wide_data_v = hit_under_miss_reordered | wide_late | wide_hit_no_miss;

    logic busy_fifo_ready_lo;
    logic v_lo, act_not_wt_n, last_n;
    logic [511:0] wdata_n;
    logic dpu_v_lo, dpu_yumi_lo;

    bsg_fifo_1r1w_small #(.width_p(2 + 512),.els_p(4))
      busy_fifo
        (.clk_i(clk_i)
        ,.reset_i(reset_i)

        ,.data_i({wide_act_not_wt_n, wide_last_n, wide_data_n})
        ,.v_i(wide_data_v)
        ,.ready_o(busy_fifo_ready_lo)

        ,.data_o({act_not_wt_n, last_n, wdata_n})
        ,.v_o(v_lo)
        ,.yumi_i(dpu_yumi_lo)
        );

    assign busy_o = wide_hit_under_miss_queue_v_lo | (v_lo & ~dpu_yumi_lo);
    //assign busy_o = 1'b0;

    assign panic_o = 1'b0;

    // synopsys translate_off
    always_ff @(posedge clk_i) begin
        if (wide_data_v & ~busy_fifo_ready_lo)
            $display("ERROR: busy fifo overflow");
    end
    // synopsys translate_on


    bsg_mla_os_dpu_bp_neo
      dpu
        (.clk_i(clk_i)
        ,.reset_i(reset_i)

        ,.data_i(wdata_n)
        ,.act_not_wt_i(act_not_wt_n)
        ,.last_i(last_n)
        ,.v_i(v_lo)
        ,.yumi_o(dpu_yumi_lo)

        ,.data_o(fsm_fwd_data_li)
        ,.v_o(dpu_v_lo)
        ,.yumi_i(fsm_fwd_v_li & fsm_fwd_ready_and_lo)
        );

    assign fsm_fwd_v_li = dpu_v_lo & dest_v_lo;


    always_comb begin
        fsm_fwd_header_li                = '0;
        fsm_fwd_header_li.msg_type       = e_bedrock_mem_uc_wr;
        fsm_fwd_header_li.addr           = dest_r;
        fsm_fwd_header_li.size           = e_bedrock_msg_size_64;
        fsm_fwd_header_li.payload.lce_id = lce_id_i;
        fsm_fwd_header_li.subop          = e_bedrock_store;
    end

    bp_me_stream_pump_out #(.bp_params_p(bp_params_p)
                           ,.fsm_data_width_p(bedrock_fill_width_p)
                           ,.block_width_p(dcache_block_width_p)
                           ,.payload_width_p(mem_fwd_payload_width_lp)
                           ,.msg_stream_mask_p(mem_fwd_stream_mask_gp)
                           ,.fsm_stream_mask_p(mem_fwd_stream_mask_gp))
      pump_out
        (.clk_i(clk_i)
        ,.reset_i(reset_i)

        ,.msg_header_o(mem_fwd_header_o)
        ,.msg_data_o(mem_fwd_data_o)
        ,.msg_v_o(mem_fwd_v_o)
        ,.msg_ready_and_i(mem_fwd_ready_and_i)

        ,.fsm_header_i(fsm_fwd_header_li)
        ,.fsm_data_i(fsm_fwd_data_li)
        ,.fsm_addr_o()
        ,.fsm_v_i(fsm_fwd_v_li)
        ,.fsm_ready_and_o(fsm_fwd_ready_and_lo)

        ,.fsm_new_o()
        ,.fsm_critical_o()
        ,.fsm_last_o(fsm_last_lo)
        );

    bp_me_stream_pump_in #(.bp_params_p(bp_params_p)
                          ,.fsm_data_width_p(bedrock_fill_width_p)
                          ,.block_width_p(dcache_block_width_p)
                          ,.payload_width_p(mem_rev_payload_width_lp)
                          ,.msg_stream_mask_p(mem_rev_stream_mask_gp)
                          ,.fsm_stream_mask_p(mem_rev_stream_mask_gp))
      pump_in
        (.clk_i(clk_i)
        ,.reset_i(reset_i)

        ,.msg_header_i(mem_rev_header_i)
        ,.msg_data_i(mem_rev_data_i)
        ,.msg_v_i(mem_rev_v_i)
        ,.msg_ready_and_o(mem_rev_ready_and_o)

        ,.fsm_header_o()
        ,.fsm_addr_o()
        ,.fsm_data_o()
        ,.fsm_v_o(fsm_rev_v_lo)
        ,.fsm_yumi_i(fsm_rev_v_lo)

        ,.fsm_new_o()
        ,.fsm_critical_o()
        ,.fsm_last_o()
        );

endmodule // bp_be_pipe_acc
