/**
 *
 * bp_common_aviary_defines.svh
 *
 */

`ifndef BP_COMMON_AVIARY_DEFINES_SVH
`define BP_COMMON_AVIARY_DEFINES_SVH

  `define declare_bp_proc_params(bp_params_e_mp) \
    , localparam bp_proc_param_s proc_param_lp = all_cfgs_gp[bp_params_e_mp]                       \
                                                                                                   \
    , localparam cc_x_dim_p  = proc_param_lp.cc_x_dim                                              \
    , localparam cc_y_dim_p  = proc_param_lp.cc_y_dim                                              \
                                                                                                   \
    , localparam ic_x_dim_p = cc_x_dim_p                                                           \
    , localparam ic_y_dim_p = proc_param_lp.ic_y_dim                                               \
    , localparam mc_x_dim_p = cc_x_dim_p                                                           \
    , localparam mc_y_dim_p = proc_param_lp.mc_y_dim                                               \
    , localparam cac_x_dim_p = proc_param_lp.cac_x_dim                                             \
    , localparam cac_y_dim_p = cc_y_dim_p                                                          \
    , localparam sac_x_dim_p = proc_param_lp.sac_x_dim                                             \
    , localparam sac_y_dim_p = cc_y_dim_p                                                          \
    , localparam cacc_type_p = proc_param_lp.cacc_type                                             \
    , localparam sacc_type_p = proc_param_lp.sacc_type                                             \
                                                                                                   \
    , localparam num_core_p  = cc_x_dim_p * cc_y_dim_p                                             \
    , localparam num_io_p    = ic_x_dim_p * ic_y_dim_p                                             \
    , localparam num_l2e_p   = mc_x_dim_p * mc_y_dim_p                                             \
    , localparam num_cacc_p  = cac_x_dim_p * cac_y_dim_p                                           \
    , localparam num_sacc_p  = sac_x_dim_p * sac_y_dim_p                                           \
                                                                                                   \
    , localparam num_cce_p = proc_param_lp.num_cce                                                 \
    , localparam num_lce_p = proc_param_lp.num_lce                                                 \
                                                                                                   \
    , localparam num_pseudo_cce_p = num_core_p+num_io_p+num_l2e_p+num_cacc_p+num_sacc_p+1          \
    , localparam num_pseudo_lce_p = 2*num_core_p+num_io_p+num_l2e_p+num_cacc_p+num_sacc_p+1        \
                                                                                                   \
    , localparam core_id_width_p = `BSG_SAFE_CLOG2(num_core_p)                                     \
    , localparam cce_id_width_p  = `BSG_SAFE_CLOG2(num_pseudo_cce_p)                               \
    , localparam lce_id_width_p  = `BSG_SAFE_CLOG2(num_pseudo_lce_p)                               \
                                                                                                   \
    , localparam vaddr_width_p   = proc_param_lp.vaddr_width                                       \
    , localparam paddr_width_p   = proc_param_lp.paddr_width                                       \
    , localparam daddr_width_p   = proc_param_lp.daddr_width                                       \
    , localparam caddr_width_p   = proc_param_lp.caddr_width                                       \
    , localparam asid_width_p    = proc_param_lp.asid_width                                        \
    , localparam hio_width_p     = paddr_width_p - daddr_width_p                                   \
                                                                                                   \
    , localparam branch_metadata_fwd_width_p = proc_param_lp.branch_metadata_fwd_width             \
    , localparam ras_idx_width_p             = proc_param_lp.ras_idx_width                         \
    , localparam btb_tag_width_p             = proc_param_lp.btb_tag_width                         \
    , localparam btb_idx_width_p             = proc_param_lp.btb_idx_width                         \
    , localparam bht_idx_width_p             = proc_param_lp.bht_idx_width                         \
    , localparam bht_row_els_p               = proc_param_lp.bht_row_els                           \
    , localparam ghist_width_p               = proc_param_lp.ghist_width                           \
    , localparam bht_row_width_p             = 2*bht_row_els_p                                     \
    , localparam bht_offset_width_p          = `BSG_SAFE_CLOG2(bht_row_els_p)                      \
                                                                                                   \
    , localparam itlb_els_4k_p              = proc_param_lp.itlb_els_4k                            \
    , localparam itlb_els_1g_p              = proc_param_lp.itlb_els_1g                            \
    , localparam dtlb_els_4k_p              = proc_param_lp.dtlb_els_4k                            \
    , localparam dtlb_els_1g_p              = proc_param_lp.dtlb_els_1g                            \
                                                                                                   \
    , localparam dcache_features_p          = proc_param_lp.dcache_features                        \
    , localparam dcache_sets_p              = proc_param_lp.dcache_sets                            \
    , localparam dcache_assoc_p             = proc_param_lp.dcache_assoc                           \
    , localparam dcache_block_width_p       = proc_param_lp.dcache_block_width                     \
    , localparam dcache_fill_width_p        = proc_param_lp.dcache_fill_width                      \
    , localparam icache_features_p          = proc_param_lp.icache_features                        \
    , localparam icache_sets_p              = proc_param_lp.icache_sets                            \
    , localparam icache_assoc_p             = proc_param_lp.icache_assoc                           \
    , localparam icache_block_width_p       = proc_param_lp.icache_block_width                     \
    , localparam icache_fill_width_p        = proc_param_lp.icache_fill_width                      \
    , localparam acache_features_p          = proc_param_lp.acache_features                        \
    , localparam acache_sets_p              = proc_param_lp.acache_sets                            \
    , localparam acache_assoc_p             = proc_param_lp.acache_assoc                           \
    , localparam acache_block_width_p       = proc_param_lp.acache_block_width                     \
    , localparam acache_fill_width_p        = proc_param_lp.acache_fill_width                      \
    , localparam lce_assoc_p                =                                                      \
        `BSG_MAX(dcache_assoc_p, `BSG_MAX(icache_assoc_p, num_cacc_p ? acache_assoc_p : '0))       \
    , localparam lce_assoc_width_p          = `BSG_SAFE_CLOG2(lce_assoc_p)                         \
    , localparam lce_sets_p                 =                                                      \
        `BSG_MAX(dcache_sets_p, `BSG_MAX(icache_sets_p, num_cacc_p ? acache_sets_p : '0))          \
    , localparam lce_sets_width_p           = `BSG_SAFE_CLOG2(lce_sets_p)                          \
                                                                                                   \
    , localparam cce_type_p                 = proc_param_lp.cce_type                               \
    , localparam cce_pc_width_p             = proc_param_lp.cce_pc_width                           \
    , localparam bedrock_block_width_p      = proc_param_lp.bedrock_block_width                    \
    , localparam bedrock_fill_width_p       = proc_param_lp.bedrock_fill_width                     \
    , localparam num_cce_instr_ram_els_p    = 2**cce_pc_width_p                                    \
    , localparam cce_way_groups_p           =                                                      \
        `BSG_MIN(dcache_sets_p, `BSG_MIN(icache_sets_p, num_cacc_p ? acache_sets_p : icache_sets_p)) \
                                                                                                   \
    /* L2 enable turns the L2 into a small write buffer, which is minimal size                  */ \
    , localparam l2_features_p         = proc_param_lp.l2_features                                 \
    , localparam l2_banks_p            = l2_features_p[e_cfg_enabled] ? proc_param_lp.l2_banks : 1 \
    , localparam l2_sets_p             = l2_features_p[e_cfg_enabled] ? proc_param_lp.l2_sets  : 4 \
    , localparam l2_assoc_p            = l2_features_p[e_cfg_enabled] ? proc_param_lp.l2_assoc : 2 \
    , localparam l2_block_width_p      = proc_param_lp.l2_block_width                              \
    , localparam l2_fill_width_p       = proc_param_lp.l2_fill_width                               \
    , localparam l2_data_width_p       = proc_param_lp.l2_data_width                               \
    , localparam l2_outstanding_reqs_p = proc_param_lp.l2_outstanding_reqs                         \
                                                                                                   \
    , localparam l2_block_size_in_words_p = l2_block_width_p / l2_data_width_p                     \
    , localparam l2_block_size_in_fill_p  = l2_block_width_p / l2_fill_width_p                     \
                                                                                                   \
    , localparam fe_queue_fifo_els_p  = proc_param_lp.fe_queue_fifo_els                            \
    , localparam fe_cmd_fifo_els_p    = proc_param_lp.fe_cmd_fifo_els                              \
    , localparam integer_support_p    = proc_param_lp.integer_support                              \
    , localparam muldiv_support_p     = proc_param_lp.muldiv_support                               \
    , localparam fpu_support_p        = proc_param_lp.fpu_support                                  \
    , localparam compressed_support_p = proc_param_lp.compressed_support                           \
                                                                                                   \
    , localparam async_coh_clk_p        = proc_param_lp.async_coh_clk                              \
    , localparam coh_noc_max_credits_p  = proc_param_lp.coh_noc_max_credits                        \
    , localparam coh_noc_flit_width_p   = proc_param_lp.coh_noc_flit_width                         \
    , localparam coh_noc_cid_width_p    = proc_param_lp.coh_noc_cid_width                          \
    , localparam coh_noc_len_width_p    = proc_param_lp.coh_noc_len_width                          \
    , localparam coh_noc_y_cord_width_p = `BSG_SAFE_CLOG2(ic_y_dim_p+cc_y_dim_p+mc_y_dim_p+1)      \
    , localparam coh_noc_x_cord_width_p = `BSG_SAFE_CLOG2(sac_x_dim_p+cc_x_dim_p+cac_x_dim_p+1)    \
    , localparam coh_noc_dims_p         = 2                                                        \
    , localparam coh_noc_dirs_p         = coh_noc_dims_p*2 + 1                                     \
    , localparam coh_noc_trans_p        = 0                                                        \
    , localparam int coh_noc_cord_markers_pos_p[coh_noc_dims_p:0] = coh_noc_trans_p                \
        ? '{coh_noc_x_cord_width_p+coh_noc_y_cord_width_p, coh_noc_y_cord_width_p, 0}              \
        : '{coh_noc_y_cord_width_p+coh_noc_x_cord_width_p, coh_noc_x_cord_width_p, 0}              \
    , localparam coh_noc_cord_width_p   = coh_noc_cord_markers_pos_p[coh_noc_dims_p]               \
                                                                                                   \
    , localparam async_mem_clk_p           = proc_param_lp.async_mem_clk                           \
    , localparam mem_noc_max_credits_p     = proc_param_lp.mem_noc_max_credits                     \
    , localparam mem_noc_did_width_p       = proc_param_lp.mem_noc_did_width                       \
    , localparam mem_noc_flit_width_p      = proc_param_lp.mem_noc_flit_width                      \
    , localparam mem_noc_cid_width_p       = proc_param_lp.mem_noc_cid_width                       \
    , localparam mem_noc_len_width_p       = proc_param_lp.mem_noc_len_width                       \
    , localparam mem_noc_y_cord_width_p    = 0                                                     \
    , localparam mem_noc_x_cord_width_p    = mem_noc_did_width_p                                   \
    , localparam mem_noc_dims_p            = 1                                                     \
    , localparam mem_noc_cord_dims_p       = 2                                                     \
    , localparam mem_noc_dirs_p            = mem_noc_cord_dims_p*2 + 1                             \
    , localparam mem_noc_trans_p           = 0                                                     \
    , localparam int mem_noc_cord_markers_pos_p[mem_noc_cord_dims_p:0] = mem_noc_trans_p           \
        ? '{mem_noc_x_cord_width_p+mem_noc_y_cord_width_p, mem_noc_y_cord_width_p, 0}              \
        : '{mem_noc_y_cord_width_p+mem_noc_x_cord_width_p, mem_noc_x_cord_width_p, 0}              \
    , localparam mem_noc_cord_width_p      = mem_noc_cord_markers_pos_p[mem_noc_dims_p]            \
                                                                                                   \
    , localparam async_dma_clk_p           = proc_param_lp.async_dma_clk                           \
    , localparam dma_noc_max_credits_p     = proc_param_lp.dma_noc_max_credits                     \
    , localparam dma_noc_flit_width_p      = proc_param_lp.dma_noc_flit_width                      \
    , localparam dma_noc_cid_width_p       = proc_param_lp.dma_noc_cid_width                       \
    , localparam dma_noc_len_width_p       = proc_param_lp.dma_noc_len_width                       \
    , localparam dma_noc_y_cord_width_p    = `BSG_SAFE_CLOG2(ic_y_dim_p+cc_y_dim_p+mc_y_dim_p+1)   \
    , localparam dma_noc_x_cord_width_p    = 0                                                     \
    , localparam dma_noc_dims_p            = 1                                                     \
    , localparam dma_noc_cord_dims_p       = 2                                                     \
    , localparam dma_noc_dirs_p            = dma_noc_dims_p*2 + 1                                  \
    , localparam dma_noc_trans_p           = 1                                                     \
    , localparam int dma_noc_cord_markers_pos_p[dma_noc_cord_dims_p:0] = dma_noc_trans_p           \
        ? '{dma_noc_x_cord_width_p+dma_noc_y_cord_width_p, dma_noc_y_cord_width_p, 0}              \
        : '{dma_noc_y_cord_width_p+dma_noc_x_cord_width_p, dma_noc_x_cord_width_p, 0}              \
    , localparam dma_noc_cord_width_p      = dma_noc_cord_markers_pos_p[dma_noc_dims_p]            \
                                                                                                   \
    , localparam did_width_p = mem_noc_did_width_p                                                 \
                                                                                                   \
    , localparam etag_width_p  = dword_width_gp - page_offset_width_gp                             \
    , localparam vtag_width_p  = vaddr_width_p - page_offset_width_gp                              \
    , localparam ptag_width_p  = paddr_width_p - page_offset_width_gp                              \
    , localparam dtag_width_p  = daddr_width_p - page_offset_width_gp                              \
    , localparam icache_ctag_width_p = caddr_width_p -                                             \
        (`BSG_SAFE_CLOG2(icache_sets_p*icache_block_width_p/8))                                    \
    , localparam dcache_ctag_width_p = caddr_width_p -                                             \
        (`BSG_SAFE_CLOG2(dcache_sets_p*dcache_block_width_p/8))                                    \
    , localparam acache_ctag_width_p = caddr_width_p -                                             \
        (`BSG_SAFE_CLOG2(acache_sets_p*acache_block_width_p/8))

    `define bp_aviary_parameter_override(parameter_mp, override_cfg_mp, default_cfg_mp) \
      parameter_mp: (override_cfg_mp.``parameter_mp`` == "inv") \
                    ? default_cfg_mp.``parameter_mp``           \
                    : override_cfg_mp.``parameter_mp``          \

    `define bp_aviary_define_override(parameter_mp, define_mp, default_cfg_mp) \
      `ifdef define_mp                                          \
      parameter_mp: `define_mp                                  \
      `else                                                     \
      parameter_mp: default_cfg_mp.``parameter_mp``             \
      `endif

    `define bp_aviary_derive_cfg(cfg_name_mp, override_cfg_mp, default_cfg_mp) \
      localparam bp_proc_param_s cfg_name_mp =                                                     \
        '{`bp_aviary_parameter_override(cc_x_dim, override_cfg_mp, default_cfg_mp)                 \
          ,`bp_aviary_parameter_override(cc_y_dim, override_cfg_mp, default_cfg_mp)                \
          ,`bp_aviary_parameter_override(ic_y_dim, override_cfg_mp, default_cfg_mp)                \
          ,`bp_aviary_parameter_override(mc_y_dim, override_cfg_mp, default_cfg_mp)                \
          ,`bp_aviary_parameter_override(cac_x_dim, override_cfg_mp, default_cfg_mp)               \
          ,`bp_aviary_parameter_override(sac_x_dim, override_cfg_mp, default_cfg_mp)               \
          ,`bp_aviary_parameter_override(cacc_type, override_cfg_mp, default_cfg_mp)               \
          ,`bp_aviary_parameter_override(sacc_type, override_cfg_mp, default_cfg_mp)               \
                                                                                                   \
          ,`bp_aviary_parameter_override(num_cce, override_cfg_mp, default_cfg_mp)                 \
          ,`bp_aviary_parameter_override(num_lce, override_cfg_mp, default_cfg_mp)                 \
                                                                                                   \
          ,`bp_aviary_parameter_override(vaddr_width, override_cfg_mp, default_cfg_mp)             \
          ,`bp_aviary_parameter_override(paddr_width, override_cfg_mp, default_cfg_mp)             \
          ,`bp_aviary_parameter_override(daddr_width, override_cfg_mp, default_cfg_mp)             \
          ,`bp_aviary_parameter_override(caddr_width, override_cfg_mp, default_cfg_mp)             \
          ,`bp_aviary_parameter_override(asid_width, override_cfg_mp, default_cfg_mp)              \
                                                                                                   \
          ,`bp_aviary_parameter_override(fe_queue_fifo_els, override_cfg_mp, default_cfg_mp)       \
          ,`bp_aviary_parameter_override(fe_cmd_fifo_els, override_cfg_mp, default_cfg_mp)         \
          ,`bp_aviary_parameter_override(integer_support, override_cfg_mp, default_cfg_mp)         \
          ,`bp_aviary_parameter_override(muldiv_support, override_cfg_mp, default_cfg_mp)          \
          ,`bp_aviary_parameter_override(fpu_support, override_cfg_mp, default_cfg_mp)             \
          ,`bp_aviary_parameter_override(compressed_support, override_cfg_mp, default_cfg_mp)      \
                                                                                                   \
          ,`bp_aviary_parameter_override(branch_metadata_fwd_width, override_cfg_mp, default_cfg_mp) \
          ,`bp_aviary_parameter_override(ras_idx_width, override_cfg_mp, default_cfg_mp)           \
          ,`bp_aviary_parameter_override(btb_tag_width, override_cfg_mp, default_cfg_mp)           \
          ,`bp_aviary_parameter_override(btb_idx_width, override_cfg_mp, default_cfg_mp)           \
          ,`bp_aviary_parameter_override(bht_idx_width, override_cfg_mp, default_cfg_mp)           \
          ,`bp_aviary_parameter_override(bht_row_els, override_cfg_mp, default_cfg_mp)             \
          ,`bp_aviary_parameter_override(ghist_width, override_cfg_mp, default_cfg_mp)             \
                                                                                                   \
          ,`bp_aviary_parameter_override(itlb_els_4k, override_cfg_mp, default_cfg_mp)             \
          ,`bp_aviary_parameter_override(itlb_els_1g, override_cfg_mp, default_cfg_mp)             \
          ,`bp_aviary_parameter_override(dtlb_els_4k, override_cfg_mp, default_cfg_mp)             \
          ,`bp_aviary_parameter_override(dtlb_els_1g, override_cfg_mp, default_cfg_mp)             \
                                                                                                   \
          ,`bp_aviary_parameter_override(icache_features, override_cfg_mp, default_cfg_mp)         \
          ,`bp_aviary_parameter_override(icache_sets, override_cfg_mp, default_cfg_mp)             \
          ,`bp_aviary_parameter_override(icache_assoc, override_cfg_mp, default_cfg_mp)            \
          ,`bp_aviary_parameter_override(icache_block_width, override_cfg_mp, default_cfg_mp)      \
          ,`bp_aviary_parameter_override(icache_fill_width, override_cfg_mp, default_cfg_mp)       \
                                                                                                   \
          ,`bp_aviary_parameter_override(dcache_features, override_cfg_mp, default_cfg_mp)         \
          ,`bp_aviary_parameter_override(dcache_sets, override_cfg_mp, default_cfg_mp)             \
          ,`bp_aviary_parameter_override(dcache_assoc, override_cfg_mp, default_cfg_mp)            \
          ,`bp_aviary_parameter_override(dcache_block_width, override_cfg_mp, default_cfg_mp)      \
          ,`bp_aviary_parameter_override(dcache_fill_width, override_cfg_mp, default_cfg_mp)       \
                                                                                                   \
          ,`bp_aviary_parameter_override(acache_features, override_cfg_mp, default_cfg_mp)         \
          ,`bp_aviary_parameter_override(acache_sets, override_cfg_mp, default_cfg_mp)             \
          ,`bp_aviary_parameter_override(acache_assoc, override_cfg_mp, default_cfg_mp)            \
          ,`bp_aviary_parameter_override(acache_block_width, override_cfg_mp, default_cfg_mp)      \
          ,`bp_aviary_parameter_override(acache_fill_width, override_cfg_mp, default_cfg_mp)       \
                                                                                                   \
          ,`bp_aviary_parameter_override(cce_type, override_cfg_mp, default_cfg_mp)                \
          ,`bp_aviary_parameter_override(cce_pc_width, override_cfg_mp, default_cfg_mp)            \
          ,`bp_aviary_parameter_override(bedrock_block_width, override_cfg_mp, default_cfg_mp)     \
          ,`bp_aviary_parameter_override(bedrock_fill_width, override_cfg_mp, default_cfg_mp)      \
                                                                                                   \
          ,`bp_aviary_parameter_override(l2_features, override_cfg_mp, default_cfg_mp)             \
          ,`bp_aviary_parameter_override(l2_banks, override_cfg_mp, default_cfg_mp)                \
          ,`bp_aviary_parameter_override(l2_data_width, override_cfg_mp, default_cfg_mp)           \
          ,`bp_aviary_parameter_override(l2_sets, override_cfg_mp, default_cfg_mp)                 \
          ,`bp_aviary_parameter_override(l2_assoc, override_cfg_mp, default_cfg_mp)                \
          ,`bp_aviary_parameter_override(l2_block_width, override_cfg_mp, default_cfg_mp)          \
          ,`bp_aviary_parameter_override(l2_fill_width, override_cfg_mp, default_cfg_mp)           \
          ,`bp_aviary_parameter_override(l2_outstanding_reqs, override_cfg_mp, default_cfg_mp)     \
                                                                                                   \
          ,`bp_aviary_parameter_override(async_coh_clk, override_cfg_mp, default_cfg_mp)           \
          ,`bp_aviary_parameter_override(coh_noc_max_credits, override_cfg_mp, default_cfg_mp)     \
          ,`bp_aviary_parameter_override(coh_noc_flit_width, override_cfg_mp, default_cfg_mp)      \
          ,`bp_aviary_parameter_override(coh_noc_cid_width, override_cfg_mp, default_cfg_mp)       \
          ,`bp_aviary_parameter_override(coh_noc_len_width, override_cfg_mp, default_cfg_mp)       \
                                                                                                   \
          ,`bp_aviary_parameter_override(async_mem_clk, override_cfg_mp, default_cfg_mp)           \
          ,`bp_aviary_parameter_override(mem_noc_max_credits, override_cfg_mp, default_cfg_mp)     \
          ,`bp_aviary_parameter_override(mem_noc_flit_width, override_cfg_mp, default_cfg_mp)      \
          ,`bp_aviary_parameter_override(mem_noc_cid_width, override_cfg_mp, default_cfg_mp)       \
          ,`bp_aviary_parameter_override(mem_noc_did_width, override_cfg_mp, default_cfg_mp)       \
          ,`bp_aviary_parameter_override(mem_noc_len_width, override_cfg_mp, default_cfg_mp)       \
                                                                                                   \
          ,`bp_aviary_parameter_override(async_dma_clk, override_cfg_mp, default_cfg_mp)           \
          ,`bp_aviary_parameter_override(dma_noc_max_credits, override_cfg_mp, default_cfg_mp)     \
          ,`bp_aviary_parameter_override(dma_noc_flit_width, override_cfg_mp, default_cfg_mp)      \
          ,`bp_aviary_parameter_override(dma_noc_cid_width, override_cfg_mp, default_cfg_mp)       \
          ,`bp_aviary_parameter_override(dma_noc_len_width, override_cfg_mp, default_cfg_mp)       \
          }

`endif

