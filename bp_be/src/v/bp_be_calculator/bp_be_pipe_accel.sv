`include "bp_common_defines.svh"
`include "bp_be_defines.svh"
`include "bp_me_defines.svh"
`include "bsg_mla_csr_pkg.svh"

module bp_be_pipe_accel
    import bp_common_pkg::*;
    import bp_be_pkg::*;
    import bsg_mla_csr_pkg::*;

#(  parameter bp_params_e bp_params_p = e_bp_default_cfg
    `declare_bp_proc_params(bp_params_p)
    `declare_bp_bedrock_mem_if_widths(paddr_width_p, did_width_p, lce_id_width_p, lce_assoc_p)
,   localparam dispatch_pkt_width_lp = `bp_be_dispatch_pkt_width(vaddr_width_p)
,   localparam commit_pkt_width_lp   = `bp_be_commit_pkt_width(vaddr_width_p, paddr_width_p)
)

(   input                                       clk_i
,   input                                       reset_i

,   output logic                                busy_o
,   output logic                                panic_o

,   input [dispatch_pkt_width_lp-1:0]           reservation_i

,   output logic [dpath_width_gp-1:0]           csr_data_o
,   output logic                                csr_v_o

,   input [commit_pkt_width_lp-1:0]             commit_pkt_i

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

    wire bp_be_dispatch_pkt_s reservation  = reservation_i;
    wire bp_be_commit_pkt_s   commit_pkt   = commit_pkt_i;

    wire rv64_instr_itype_s   commit_instr = commit_pkt.instr;

    ///////////////////////////////////////////////////////////////////////////
    //
    // COMMIT INSTRUCTION DECODE
    //

    typedef struct packed {
        logic wl_op;
        logic al_op;
    } bp_be_pipe_accel_decode_s;

    bp_be_pipe_accel_decode_s commit_decode;

    always_comb
        begin
            commit_decode = '0;
            unique casez (commit_instr)
                `RV64_TENSOR_ACLD0: begin
                    commit_decode.al_op = 1'b1;
                end
                `RV64_TENSOR_ACLD1: begin
                    commit_decode.al_op = 1'b1;
                end
                `RV64_TENSOR_WTLD0: begin
                    commit_decode.wl_op = 1'b1;
                end
                `RV64_TENSOR_WTLD1: begin
                    commit_decode.wl_op = 1'b1;
                end
                default: begin
                end
            endcase
        end

    ///////////////////////////////////////////////////////////////////////////
    //
    // READ / WRITE CSRS
    //

    localparam num_csrs_lp = 32;

    typedef enum logic [4:0] {
        eDEST0 = 5'd0,
        eDEST1 = 5'd1,

        ePERF_WL_STALLS = 5'd2,
        ePERF_AL_STALLS = 5'd3,
        ePANIC_STALLS   = 5'd4,
        eRUNNING_STALLS = 5'd5,
        eIDLE_STALLS = 5'd6
    } bp_be_pipe_accel_csrs_e;

    wire [dpath_width_gp-1:0] csr_mem_data_li = reservation.rs1;

    logic [num_csrs_lp-1:0] csr_mem_wen_li;
    logic [num_csrs_lp-1:0][dpath_width_gp-1:0] csr_mem_data_lo;

    `bsg_mla_csr_create(dest0,          eDEST0,          dpath_width_gp, dpath_width_gp);
    `bsg_mla_csr_create(dest1,          eDEST1,          dpath_width_gp, dpath_width_gp);
    `bsg_mla_csr_create(perf_wl_stalls, ePERF_WL_STALLS, dpath_width_gp, dpath_width_gp);
    `bsg_mla_csr_create(perf_al_stalls, ePERF_AL_STALLS, dpath_width_gp, dpath_width_gp);
    `bsg_mla_csr_create(panic_stalls,   ePANIC_STALLS,   dpath_width_gp, dpath_width_gp);
    `bsg_mla_csr_create(running_stalls, eRUNNING_STALLS,    dpath_width_gp, dpath_width_gp);
    `bsg_mla_csr_create(idle_stalls,    eIDLE_STALLS,    dpath_width_gp, dpath_width_gp);
        // Create the CSR registers and connects to the csr_mem_* logics created before and creates
        // 3 new logics for each CSR:
        //      1. csr_core_``name``_n
        //      2. csr_core_``name``_r
        //      3. csr_core_``name``_wen

    // No core-side writes for these csrs
    assign csr_core_dest0_n = csr_core_dest0_r + 16;
    assign csr_core_dest1_n = csr_core_dest1_r + 16;
    assign csr_core_dest0_wen = 1'b0;
    assign csr_core_dest1_wen = 1'b0;

    always_comb begin
        csr_mem_wen_li = '0;
        csr_data_o     = '0;
        csr_v_o        = '0;
        if (reservation.v) begin
            unique casez (reservation.instr)
                `RV64_TENSOR_CSRST: begin
                    csr_mem_wen_li[reservation.imm] = 1'b1;
                end
                `RV64_TENSOR_CSRLD: begin
                    csr_data_o = csr_mem_data_lo[reservation.imm];
                    csr_v_o = 1'b1;
                end
                default: begin
                end
            endcase
        end
    end

    logic wl_stalls_running;
    bsg_mla_csr_perf_ctr #
        (.core_width_p(dpath_width_gp))
    perf_wl_stals
        (.clk_i(clk_i)
        ,.reset_i(reset_i)
        ,.start_i(commit_pkt.dcache_replay & commit_decode.wl_op)
        ,.stop_i(cache_wide_v_i)

        ,.core_data_r_i(csr_core_perf_wl_stalls_r)
        ,.core_data_n_o(csr_core_perf_wl_stalls_n)
        ,.core_wen_o(csr_core_perf_wl_stalls_wen)

        ,.running_o(wl_stalls_running)
        );

    logic al_stalls_running;
    bsg_mla_csr_perf_ctr #
        (.core_width_p(dpath_width_gp))
    perf_al_stals
        (.clk_i(clk_i)
        ,.reset_i(reset_i)
        ,.start_i(commit_pkt.dcache_replay & commit_decode.al_op)
        ,.stop_i(cache_wide_v_i)

        ,.core_data_r_i(csr_core_perf_al_stalls_r)
        ,.core_data_n_o(csr_core_perf_al_stalls_n)
        ,.core_wen_o(csr_core_perf_al_stalls_wen)

        ,.running_o(al_stalls_running)
        );

    logic panic_stalls_running;
    bsg_mla_csr_perf_ctr #
        (.core_width_p(dpath_width_gp))
    panic_stalls
        (.clk_i(clk_i)
        ,.reset_i(reset_i)
        ,.start_i(panic_o)
        ,.stop_i(~panic_o)

        ,.core_data_r_i(csr_core_panic_stalls_r)
        ,.core_data_n_o(csr_core_panic_stalls_n)
        ,.core_wen_o(csr_core_panic_stalls_wen)

        ,.running_o(panic_stalls_running)
        );

    logic core_running_n, core_stalls_running;

    bsg_mla_csr_perf_ctr #
        (.core_width_p(dpath_width_gp))
    running_stalls
        (.clk_i(clk_i)
        ,.reset_i(reset_i)
        ,.start_i(core_running_n)
        ,.stop_i(~core_running_n)

        ,.core_data_r_i(csr_core_running_stalls_r)
        ,.core_data_n_o(csr_core_running_stalls_n)
        ,.core_wen_o(csr_core_running_stalls_wen)

        ,.running_o(core_stalls_running)
        );

    wire idle_stop = core_stalls_running | panic_stalls_running | al_stalls_running | wl_stalls_running;

    bsg_mla_csr_perf_ctr #
        (.core_width_p(dpath_width_gp))
    idle_stalls
        (.clk_i(clk_i)
        ,.reset_i(reset_i)
        ,.start_i(~idle_stop)
        ,.stop_i(idle_stop)

        ,.core_data_r_i(csr_core_idle_stalls_r)
        ,.core_data_n_o(csr_core_idle_stalls_n)
        ,.core_wen_o(csr_core_idle_stalls_wen)

        ,.running_o()
        );

    ///////////////////////////////////////////////////////////////////////////
    //
    // MAIN TENSOR-CORE OPERATIONS
    //

    logic [bedrock_fill_width_p-1:0] core_data_lo, fsm_fwd_data_li;
    logic fsm_fwd_v_li, fsm_fwd_ready_and_lo, core_v_lo, panic_fifo_ready_lo;
    logic fsm_fwd_buf_li, core_buf_lo;

    logic [1:0] ws_op_n;
    logic       ws_op_v;

    logic [1:0] core_op_n;
    logic       core_op_v;

    logic [dcache_block_width_p-1:0] core_data_n;
    logic core_data_v;

    always_comb begin
        ws_op_n = '0;
        ws_op_v = 1'b0;
        if (commit_pkt.instret) begin
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

    bsg_fifo_1r1w_small #
        (.width_p(2)
        ,.els_p(2))
    op_fifo
        (.clk_i(clk_i)
        ,.reset_i(reset_i)

        ,.data_i(ws_op_n)
        ,.v_i(ws_op_v)
        ,.ready_o()

        ,.data_o(core_op_n)
        ,.v_o(core_op_v)
        ,.yumi_i(core_op_v & core_data_v)
        );

    bsg_fifo_1r1w_small #
        (.width_p(dcache_block_width_p)
        ,.els_p(2))
    data_fifo
        (.clk_i(clk_i)
        ,.reset_i(reset_i)

        ,.data_i(cache_wide_data_i)
        ,.v_i(cache_wide_v_i & ~panic_o)
        ,.ready_o()

        ,.data_o(core_data_n)
        ,.v_o(core_data_v)
        ,.yumi_i(core_data_v & core_op_v)
        );

    logic [dpath_width_gp-1:0] addr_lo;
    logic last_lo;

    bsg_mla_ws_dpu_bp_neo
    xor_core
        (.clk_i(clk_i)
        ,.reset_i(reset_i)
        ,.running_o(core_running_n)

        ,.op_i(core_op_n)
        ,.data_i(core_data_n)
        ,.addr_i(csr_core_dest0_r)
        ,.v_i(core_data_v & core_op_v)

        ,.data_o(core_data_lo)
        ,.buf_o(core_buf_lo)
        ,.addr_o(addr_lo)
        ,.v_o(core_v_lo)
        ,.yumi_i(core_v_lo & panic_fifo_ready_lo)
        ,.addr_yumi_i(last_lo & fsm_fwd_v_li & fsm_fwd_ready_and_lo)
        );

    bsg_two_fifo #
        (.width_p(1+bedrock_fill_width_p))
    panic_twofer
        (.clk_i(clk_i)
        ,.reset_i(reset_i)

        ,.data_i({core_buf_lo, core_data_lo})
        ,.v_i(core_v_lo)
        ,.ready_o(panic_fifo_ready_lo)

        ,.data_o({fsm_fwd_buf_li, fsm_fwd_data_li})
        ,.v_o(fsm_fwd_v_li)
        ,.yumi_i(fsm_fwd_v_li & fsm_fwd_ready_and_lo)
        );

    assign busy_o = ~panic_fifo_ready_lo;
    assign panic_o = core_v_lo & fsm_fwd_v_li & ~fsm_fwd_ready_and_lo;

    bp_bedrock_mem_fwd_header_s fsm_fwd_header_li;
    always_comb
        begin
            fsm_fwd_header_li                = '0;
            fsm_fwd_header_li.msg_type       = e_bedrock_mem_uc_wr;
            fsm_fwd_header_li.addr           = addr_lo;
            fsm_fwd_header_li.size           = e_bedrock_msg_size_64; // 128b
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
        ,.fsm_last_o(last_lo)
        );

    //
    // simply need to capture returns for our writes... nothing is really done here.
    //

    logic fsm_rev_v_lo;

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

endmodule // bp_be_pipe_accel
