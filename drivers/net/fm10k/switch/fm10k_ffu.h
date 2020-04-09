/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright 2019   Silicom Ltd. Connectivity Solutions
 */

#ifndef _FM10K_FFU_H_
#define _FM10K_FFU_H_

#include <stdint.h>

#define FM10K_FFU_EXT_PORT_RULE_INGRESS(i)	((i) * 2)
#define FM10K_FFU_EXT_PORT_RULE_EGRESS(i)	((i) * 2 + 1)
#define FM10K_FFU_RULE_MAX			FM10K_SW_FFU_RULE_MAX

struct fm10k_switch;
struct fm10k_dpdk_cfg;
struct fm10k_cfg_flow;

int fm10k_ffu_init(struct fm10k_switch *sw, struct fm10k_dpdk_cfg *cfg);

int fm10k_ffu_mirror_set(struct fm10k_switch *sw,
		uint16_t src_dpdk_port, u16 dest_dpdk_port, uint16_t vlan);
int fm10k_ffu_mirror_reset(struct fm10k_switch *sw, int src_dpdk_port);

void fm10k_ffu_flow_enable(struct fm10k_switch *sw,
		struct fm10k_cfg_flow *flow);
void fm10k_ffu_flow_disable(struct fm10k_switch *sw,
		struct fm10k_cfg_flow *flow);
void fm10k_ffu_flowset_switch(struct fm10k_switch *sw, const char *new_name);

#endif /* _FM10K_FFU_H */
