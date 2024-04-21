#ifndef RUNNER_D_CODE_ADDRESSES
#define RUNNER_D_CODE_ADDRESSES

#define runner_d_start_task_initialization_task		(0x14)
#define runner_d_initialization_task		(0x14)
#define runner_d_start_task_timer_scheduler_set		(0x1B84)
#define runner_d_timer_scheduler_set		(0x1B84)
#define runner_d_start_task_flow_cache_wakeup_request		(0xCAC)
#define runner_d_flow_cache_wakeup_request		(0xCAC)
#define runner_d_start_task_lan_dispatch_wakeup_request		(0x192C)
#define runner_d_lan_dispatch_wakeup_request		(0x192C)
#define runner_d_start_task_lan_cpu_wakeup_request		(0x154)
#define runner_d_lan_cpu_wakeup_request		(0x154)
#define runner_d_start_task_debug_routine		(0x6C)
#define runner_d_debug_routine		(0x6C)
#define runner_d_start_task_free_skb_index_wakeup_request		(0x1D00)
#define runner_d_free_skb_index_wakeup_request		(0x1D00)
#define runner_d_free_skb_index_tx_abs_done		(0x1D00)
#define runner_d_start_task_dhd_rx_complete_wakeup_request		(0x1DD4)
#define runner_d_dhd_rx_complete_wakeup_request		(0x1DD4)
#define runner_d_gpe_sop_push_replace_ddr_sram_32		(0x12B4)
#define runner_d_gpe_sop_push_replace_sram_32_64		(0x1328)
#define runner_d_gpe_sop_push_replace_sram_64		(0x133C)
#define runner_d_gpe_sop_push_replace_sram_64_32		(0x1350)
#define runner_d_gpe_sop_pull_replace_ddr_sram_32		(0x1364)
#define runner_d_gpe_sop_pull_replace_sram_32_64		(0x13D8)
#define runner_d_gpe_sop_pull_replace_sram_64		(0x1428)
#define runner_d_gpe_sop_pull_replace_sram_64_32		(0x1464)
#define runner_d_gpe_replace_pointer_32_ddr		(0x14B4)
#define runner_d_gpe_replace_pointer_32_sram		(0x14D8)
#define runner_d_gpe_replace_pointer_64_sram		(0x14FC)
#define runner_d_gpe_replace_16		(0x1520)
#define runner_d_gpe_replace_32		(0x1554)
#define runner_d_gpe_replace_bits_16		(0x1578)
#define runner_d_gpe_copy_add_16_cl		(0x15A4)
#define runner_d_gpe_copy_add_16_sram		(0x15B0)
#define runner_d_gpe_copy_bits_16_cl		(0x15F8)
#define runner_d_gpe_copy_bits_16_sram		(0x1604)
#define runner_d_gpe_insert_16		(0x164C)
#define runner_d_gpe_delete_16		(0x16B4)
#define runner_d_gpe_decrement_8		(0x16F4)
#define runner_d_gpe_apply_icsum_16		(0x1718)
#define runner_d_gpe_apply_icsum_nz_16		(0x173C)
#define runner_d_gpe_compute_csum_16_cl		(0x1778)
#define runner_d_gpe_compute_csum_16_sram		(0x1784)
#define runner_d_gpe_buffer_copy_16_sram		(0x17C4)
#define runner_d_gpe_buffer_copy_16_ddr		(0x17EC)
#define runner_d_gpe_replace_add_packet_length_cl		(0x1818)
#define runner_d_upstream_ingress_rate_limiter_budget_allocate		(0x1C04)
#define runner_d_upstream_quasi_budget_allocate		(0x1C84)
#define runner_d_schedule_free_skb_index		(0x1CE4)

#endif