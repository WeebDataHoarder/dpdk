/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright 2019   Silicom Ltd. Connectivity Solutions
 */

#ifndef _FM10K_CONFIG_H_
#define _FM10K_CONFIG_H_

#include <stdint.h>
#include "fm10k_switch.h"

/* General configuration */
#define FM10K_CONFIG_TYPE_NULL			0
#define FM10K_CONFIG_BIND_PF_NUMBER		1
#define FM10K_CONFIG_EXT_PORT_SPEED		2

/* Internal redirect configuration */
#define FM10K_CONFIG_DPDK_PORT_MAP_PF		10
#define FM10K_CONFIG_EXT_PORT_MAP_PF		11

/* Debug configuration */
#define FM10K_CONFIG_DEBUG_START		50
#define FM10K_CONFIG_DEBUG_ENABLE		(FM10K_CONFIG_DEBUG_START + 0)
#define FM10K_CONFIG_DEBUG_CONFIG		(FM10K_CONFIG_DEBUG_START + 1)
#define FM10K_CONFIG_DEBUG_FFU_INIT		(FM10K_CONFIG_DEBUG_START + 2)
#define FM10K_CONFIG_DEBUG_FFU_REG		(FM10K_CONFIG_DEBUG_START + 3)
#define FM10K_CONFIG_DEBUG_FFU_RULE		(FM10K_CONFIG_DEBUG_START + 4)
#define FM10K_CONFIG_DEBUG_STATS_PORT		(FM10K_CONFIG_DEBUG_START + 5)
#define FM10K_CONFIG_DEBUG_STATS_QUEUE		(FM10K_CONFIG_DEBUG_START + 6)
#define FM10K_CONFIG_DEBUG_STATS_FFU		(FM10K_CONFIG_DEBUG_START + 7)
#define FM10K_CONFIG_DEBUG_STATS_MORE		(FM10K_CONFIG_DEBUG_START + 8)
#define FM10K_CONFIG_DEBUG_STATS_INTERVAL	(FM10K_CONFIG_DEBUG_START + 9)

/* external redirect configuration */
#define FM10K_CONFIG_FLOWSET_START		80
#define FM10K_CONFIG_FLOWSET_STOP		81
#define FM10K_CONFIG_FLOWSET_ENABLE		82

#define FM10K_CONFIG_FLOW_COND_START		100
#define FM10K_CONFIG_FLOW_COND_SRC_EXT_PORT	\
		(FM10K_CONFIG_FLOW_COND_START + 0)
#define FM10K_CONFIG_FLOW_COND_SRC_DPDK_PORT	\
		(FM10K_CONFIG_FLOW_COND_START + 1)
#define FM10K_CONFIG_FLOW_COND_VLAN		\
		(FM10K_CONFIG_FLOW_COND_START + 2)
#define FM10K_CONFIG_FLOW_COND_END		\
		(FM10K_CONFIG_FLOW_COND_START + 2)

#define FM10K_CONFIG_FLOW_ACT_START		200
#define FM10K_CONFIG_FLOW_ACT_FW_EXT_PORT	\
		(FM10K_CONFIG_FLOW_ACT_START + 0)
#define FM10K_CONFIG_FLOW_ACT_FW_DPDK_PORT	\
		(FM10K_CONFIG_FLOW_ACT_START + 1)
#define FM10K_CONFIG_FLOW_ACT_FW_VALN		\
		(FM10K_CONFIG_FLOW_ACT_START + 2)
#define FM10K_CONFIG_FLOW_ACT_END		\
		(FM10K_CONFIG_FLOW_ACT_START + 2)

#define FM10K_CONFIG_FLOW_NONE_PORT		0
#define FM10K_CONFIG_FLOW_EXT_PORT		1
#define FM10K_CONFIG_FLOW_DPDK_PORT		2

#define FM10K_CONFIG_VALUE_NULL			0
#define FM10K_CONFIG_VALUE_INT			1
#define FM10K_CONFIG_VALUE_STR			2

/* SWITCH config */
#define FM10K_CONFIG_PORT_MAP_NULL		0
#define FM10K_CONFIG_PORT_MAP_PF		1
#define FM10K_CONFIG_PORT_MAP_PFS		2
#define FM10K_CONFIG_PORT_MAP_PFSS		3

/* DPDK port */
#define	FM10K_CONFIG_DPDK_NULL			0
#define FM10K_CONFIG_DPDK_PF			1
#define FM10K_CONFIG_DPDK_VF			2
#define FM10K_CONFIG_DPDK_MAX			3

struct fm10k_cfg_port_pf_map {
	uint8_t type;
	uint8_t map_no[2];
};

/* Configuration read from file */
struct fm10k_cfg_config_item {
	uint8_t  type;
	uint8_t  val_type;
	uint16_t key_param;
	const char *describe;
	union {
		int64_t int64;
		char *str;
	} val;
};

struct fm10k_hw;

struct fm10k_dpdk_port {
	uint8_t type;
	struct fm10k_hw *hw;
	void *rte_dev;
	void *flow_list;
	uint16_t pf_no;
	uint8_t tx_queue_num;
	uint8_t rx_queue_num;
};

/* Flow configuration */
struct fm10k_cfg_port {
	uint8_t port_type;
	uint8_t port_no;
	uint16_t vlan_id;
};

struct fm10k_cfg_flow {
	/* set by configuration */
	struct fm10k_cfg_flow *prev;
	struct fm10k_cfg_flow *next;
	uint8_t flow_no; /* only configured flow has this NO. */
	struct fm10k_cfg_port src_port;
	struct fm10k_cfg_port fw_port[2];
	/* set by ffu rule add */
	uint16_t rule_id;
};

#define FM10K_CONFIG_FLOWSET_NAME_MAX	256
struct fm10k_cfg_flowset {
	char name[FM10K_CONFIG_FLOWSET_NAME_MAX];
	struct fm10k_cfg_flow flow_head;
	struct fm10k_cfg_flowset *next;
};

/* Configuration */
struct fm10k_dpdk_cfg {
	uint8_t pf_num;		/* configure by conf */
	uint8_t pf_bind;	/* initialize by dpdk */
	uint8_t pf_max;		/* set by card type */
	uint8_t ext_port_num;	/* configure by conf */
	uint8_t ext_port_speed;	/* configure by conf */
	uint32_t debug_cfg;	/* configure by conf */
	uint32_t stats_interval;/* configure by conf */

	struct fm10k_hw *master_hw;	/* initialize by dpdk */
	struct fm10k_hw *pf_hw[FM10K_SW_PEP_PORTS_MAX];	/* initialize by dpdk */

	/* initialize by dpdk */
	struct fm10k_dpdk_port ports[FM10K_SW_LOGICAL_PORTS_MAX];
	/* configure by conf or default*/
	struct fm10k_cfg_port_pf_map dpdk_port_map[FM10K_SW_LOGICAL_PORTS_MAX];
	/* configure by conf */
	struct fm10k_cfg_port_pf_map ext_port_map[FM10K_SW_EXT_PORTS_MAX];
	struct fm10k_cfg_config_item *config_list;	/* configure by conf */
	struct fm10k_cfg_flowset flowset_head;	/* transfer from conf file */
	struct fm10k_cfg_flowset *current;
};

static inline bool
fm10k_config_check_debug(struct fm10k_dpdk_cfg *cfg, uint16_t dbg)
{
	return cfg->debug_cfg & 1 << (dbg - FM10K_CONFIG_DEBUG_START) &&
		   cfg->debug_cfg & 1;
}

struct fm10k_cfg_flowset *fm10k_config_flowset_get(const char *name);
struct fm10k_cfg_flowset *fm10k_config_flowset_current_get(void);
void fm10k_config_flowset_current_set(struct fm10k_cfg_flowset *new);
void fm10k_config_flow_list_add_tail(struct fm10k_cfg_flowset *flowset,
				struct fm10k_cfg_flow *flow);
void fm10k_config_flow_list_delete(struct fm10k_cfg_flow *flow);
bool fm10k_config_flow_list_end(struct fm10k_cfg_flow *list,
				struct fm10k_cfg_flow *flow);

void fm10k_config_cfg_flowset_show(void);
struct fm10k_hw *fm10k_config_hw_get(int port_no);
int fm10k_config_init(struct fm10k_switch *sw, struct fm10k_hw *hw);

#endif /* _FM10K_CONFIG_H */
