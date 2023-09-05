`include "bp_common_defines.svh"
`include "bp_be_defines.svh"
`include "bp_me_defines.svh"

module bp_be_pipe_accel
    import bp_common_pkg::*;
    import bp_be_pkg::*;
#(  parameter bp_params_e bp_params_p = e_bp_default_cfg
    `declare_bp_proc_params(bp_params_p)
    `declare_bp_bedrock_mem_if_widths(paddr_width_p, did_width_p, lce_id_width_p, lce_assoc_p)
,   localparam dispatch_pkt_width_lp = `bp_be_dispatch_pkt_width(vaddr_width_p)
)

(   input                                       clk_i
,   input                                       reset_i

,   input [dispatch_pkt_width_lp-1:0]           reservation_i
,   output logic                                busy_o
,   output logic                                panic_o

,   output logic [dpath_width_gp-1:0]           csr_data_o
,   output logic                                csr_v_o

,   input [31:0]                                commit_instr_i
,   input                                       commit_instr_v_i

,   input [dpath_width_gp-1:0]                  cache_incr_data_i
,   input                                       cache_incr_v_i

,   input [dpath_width_gp-1:0]                  cache_early_data_i
,   input                                       cache_early_v_i

,   input [dpath_width_gp-1:0]                  cache_final_data_i
,   input                                       cache_final_v_i

,   input [dcache_block_width_p-1:0]            cache_wide_data_i
,   input                                       cache_wide_v_i

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

    `declare_bp_be_internal_if_structs(vaddr_width_p, paddr_width_p, asid_width_p, branch_metadata_fwd_width_p);
    `declare_bp_bedrock_mem_if(paddr_width_p, did_width_p, lce_id_width_p, lce_assoc_p);

    localparam block_size_in_fill_lp = dcache_block_width_p / bedrock_fill_width_p;
    localparam fill_cnt_width_lp = `BSG_SAFE_CLOG2(block_size_in_fill_lp);

    localparam csr_dest_lp = 0;
    localparam num_csrs_lp = csr_dest_lp + 1;

    bp_be_dispatch_pkt_s reservation;
    rv64_instr_itype_s reserve_instr;
    rv64_instr_itype_s commit_instr;

    bp_bedrock_mem_fwd_header_s fsm_fwd_header_li;
    logic [bedrock_fill_width_p-1:0] core_data_lo, fsm_fwd_data_li;
    logic fsm_fwd_v_li, fsm_fwd_ready_and_lo, core_v_lo, panic_fifo_ready_lo;

    logic fsm_rev_v_lo;

    logic [1:0] ws_op_n;
    logic ws_op_v;

    logic [1:0] core_op_n;
    logic core_op_v;

    logic [dcache_block_width_p-1:0] core_data_n;
    logic core_data_v;

    assign reservation   = reservation_i;
    assign reserve_instr = reservation.instr;
    assign commit_instr  = commit_instr_i;

    logic [num_csrs_lp-1:0][63:0] csr_n, csr_r;
    logic [num_csrs_lp-1:0]       csr_en;

    always_comb begin
        csr_n   = '0;
        csr_en  = '0;
        csr_v_o = 1'b0;
        if (reservation.v) begin
            unique casez (reserve_instr)
                `RV64_TENSOR_CSRST: begin
                    csr_n [reserve_instr.imm12] = reservation.rs1[0+:64];
                    csr_en[reserve_instr.imm12] = 1'b1;
                end
                `RV64_TENSOR_CSRLD: begin
                    csr_v_o = 1'b1;
                end
                default: begin
                end
            endcase
        end
        if (fsm_fwd_v_li & fsm_fwd_ready_and_lo) begin
            csr_n[csr_dest_lp] = csr_r[csr_dest_lp] + 64;
            csr_en[csr_dest_lp] = 1'b1;
        end
    end

    for (genvar i = 0; i < num_csrs_lp; i++) begin
        always_ff @(posedge clk_i) begin
            if (reset_i) begin
                csr_r[i] <= '0;
            end else if (csr_en[i]) begin
                csr_r[i] <= csr_n[i];
            end
        end
    end

    assign csr_data_o = {{dpath_width_gp-64{1'b0}}, csr_r[reserve_instr.imm12]};


    always_comb begin
        ws_op_n = '0;
        ws_op_v = 1'b0;
        if (commit_instr_v_i) begin
            unique casez (commit_instr)
                `RV64_TENSOR_ACLD0: begin
                    ws_op_n = 2'b00;
                    ws_op_v = 1'b1;
                end
                `RV64_TENSOR_ACLD1: begin
                    ws_op_n = 2'b01;
                    ws_op_v = 1'b1;
                end
                `RV64_TENSOR_WTLD0: begin
                    ws_op_n = 2'b10;
                    ws_op_v = 1'b1;
                end
                `RV64_TENSOR_WTLD1: begin
                    ws_op_n = 2'b11;
                    ws_op_v = 1'b1;
                end
                default: begin
                end
            endcase
        end
    end

    bsg_two_fifo #
        (.width_p(2))
    op_fifo
        (.clk_i(clk_i)
        ,.reset_i(reset_i)

        ,.data_i(ws_op_n)
        ,.v_i(ws_op_v)
        ,.ready_o()

        ,.data_o(core_op_n)
        ,.v_o(core_op_v)
        ,.yumi_i(cache_wide_v_i)
        );

    // bsg_fifo_1r1w_small #
    //     (.width_p(dcache_block_width_p)
    //     ,.els_p(2))
    // data_fifo
    //     (.clk_i(clk_i)
    //     ,.reset_i(reset_i)

    //     ,.data_i(cache_wide_data_i)
    //     ,.v_i(cache_wide_v_i)
    //     ,.ready_o()

    //     ,.data_o(core_data_n)
    //     ,.v_o(core_data_v)
    //     ,.yumi_i(core_data_v & core_op_v)
    //     );

    bsg_ws_systolic_array_xor
    xor_core
        (.clk_i(clk_i)
        ,.reset_i(reset_i)

        ,.op_i(core_op_n)
        ,.data_i(cache_wide_data_i)
        ,.v_i(cache_wide_v_i)

        ,.data_o(core_data_lo)
        ,.v_o(core_v_lo)
        ,.yumi_i(core_v_lo & panic_fifo_ready_lo)
        );

    bsg_two_fifo #(.width_p(bedrock_fill_width_p))
      panic_twofer
        (.clk_i(clk_i)
        ,.reset_i(reset_i)

        ,.data_i(core_data_lo)
        ,.v_i(core_v_lo)
        ,.ready_o(panic_fifo_ready_lo)

        ,.data_o(fsm_fwd_data_li)
        ,.v_o(fsm_fwd_v_li)
        ,.yumi_i(fsm_fwd_v_li & fsm_fwd_ready_and_lo)
        );

    assign busy_o = ~panic_fifo_ready_lo;
    assign panic_o = core_v_lo & fsm_fwd_v_li & ~fsm_fwd_ready_and_lo;

    always_comb begin
        fsm_fwd_header_li                = '0;
        fsm_fwd_header_li.msg_type       = e_bedrock_mem_uc_wr;
        fsm_fwd_header_li.addr           = csr_r[csr_dest_lp];
        fsm_fwd_header_li.size           = e_bedrock_msg_size_16; // 128b
        fsm_fwd_header_li.payload.lce_id = lce_id_i;
        fsm_fwd_header_li.subop          = e_bedrock_store;
    end

    bp_me_stream_pump_out #
        (.bp_params_p(bp_params_p)
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
        ,.fsm_last_o()
        );

    bp_me_stream_pump_in #
        (.bp_params_p(bp_params_p)
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

endmodule
