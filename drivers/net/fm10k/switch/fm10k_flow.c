/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright 2019   Silicom Ltd. Connectivity Solutions
 */

#include <sys/queue.h>
#include <stdio.h>
#include <errno.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <stdarg.h>

#include <rte_ether.h>
#include <rte_ethdev.h>
#include <rte_log.h>
#include <rte_malloc.h>
#include <rte_eth_ctrl.h>
#include <rte_tailq.h>
#include <rte_flow_driver.h>

#include "fm10k_flow.h"
#include "fm10k_switch.h"
#include "fm10k_ffu.h"
#include "fm10k_config.h"

static int fm10k_flow_validate(struct rte_eth_dev *dev,
			      const struct rte_flow_attr *attr,
			      const struct rte_flow_item pattern[],
			      const struct rte_flow_action actions[],
			      struct rte_flow_error *error);
static struct rte_flow *fm10k_flow_create(struct rte_eth_dev *dev,
					 const struct rte_flow_attr *attr,
					 const struct rte_flow_item pattern[],
					 const struct rte_flow_action actions[],
					 struct rte_flow_error *error);
static int fm10k_flow_destroy(struct rte_eth_dev *dev,
			     struct rte_flow *flow,
			     struct rte_flow_error *error);
static int fm10k_flow_flush(struct rte_eth_dev *dev,
			   struct rte_flow_error *error);
static int fm10k_flow_parse_attr(const struct rte_flow_attr *attr,
				struct rte_flow_error *error);

const struct rte_flow_ops fm10k_flow_ops = {
	.validate = fm10k_flow_validate,
	.create = fm10k_flow_create,
	.destroy = fm10k_flow_destroy,
	.flush = fm10k_flow_flush,
};

union fm10k_filter_t cons_filter;
enum rte_filter_type fm10k_cons_filter_type = RTE_ETH_FILTER_NONE;

/**
 * MPLS filter configuration.
 */
enum fm10k_mpls_type {
	FM10K_MPLS_TYPE_UNI,
	FM10K_MPLS_TYPE_MULTI,
};

enum fm10k_mpls_action {
	FM10K_MPLS_ACTION_DROP,
	FM10K_MPLS_ACTION_QUEUE,
};

struct fm10k_mpls_filter_conf {
	enum fm10k_mpls_type mpls_type; /**< mandatory for MPLS */
	uint32_t mpls_header;     /**< MPLS header */
	uint32_t mpls_header_mask;      /**< MPLS header mask */
	enum fm10k_mpls_action mpls_action;
	uint16_t queue;
	uint8_t ffu_id;
	uint8_t ffu_prio;
};

/**
 * VLAN filter configuration.
 */
struct fm10k_vlan_filter_conf {
	int ffu_id;
	uint8_t ffu_prio;
	uint8_t is_ingress;
	uint8_t port;
	uint8_t in_ext_port;
	uint8_t out_ext_port;
	uint16_t in_vlan;
	uint16_t out_vlan;
};

union fm10k_filter_t {
	struct fm10k_mpls_filter_conf mpls_filter;
	struct fm10k_vlan_filter_conf vlan_filter;
};

typedef int (*parse_filter_t)(struct rte_eth_dev *dev,
			      const struct rte_flow_attr *attr,
			      const struct rte_flow_item pattern[],
			      const struct rte_flow_action actions[],
			      struct rte_flow_error *error,
			      union fm10k_filter_t *filter);

struct fm10k_valid_pattern {
	enum rte_flow_item_type *items;
	parse_filter_t parse_filter;
};

static enum rte_flow_item_type pattern_mpls_1[] = {
	RTE_FLOW_ITEM_TYPE_ETH,
	RTE_FLOW_ITEM_TYPE_MPLS,
	RTE_FLOW_ITEM_TYPE_END,
};

static enum rte_flow_item_type pattern_vlan_1[] = {
	RTE_FLOW_ITEM_TYPE_VLAN,
	RTE_FLOW_ITEM_TYPE_PHY_PORT,
	RTE_FLOW_ITEM_TYPE_END,
};

static enum rte_flow_item_type pattern_vlan_2[] = {
	RTE_FLOW_ITEM_TYPE_VLAN,
	RTE_FLOW_ITEM_TYPE_PHY_PORT,
	RTE_FLOW_ITEM_TYPE_PHY_PORT,
	RTE_FLOW_ITEM_TYPE_END,
};

static int fm10k_flow_parse_mpls_filter(struct rte_eth_dev *dev,
				    const struct rte_flow_attr *attr,
				    const struct rte_flow_item pattern[],
				    const struct rte_flow_action actions[],
				    struct rte_flow_error *error,
				    union fm10k_filter_t *filter);
static int
fm10k_flow_parse_vlan_filter(struct rte_eth_dev *dev,
			    const struct rte_flow_attr *attr,
			    const struct rte_flow_item pattern[],
			    const struct rte_flow_action actions[],
			    struct rte_flow_error *error,
			    union fm10k_filter_t *filter);

static struct fm10k_valid_pattern fm10k_supported_patterns[] = {
	/* MPLS */
	{ pattern_mpls_1, fm10k_flow_parse_mpls_filter },
	/* VLAN */
	{ pattern_vlan_1, fm10k_flow_parse_vlan_filter },
	/* VLAN */
	{ pattern_vlan_2, fm10k_flow_parse_vlan_filter },
};

static const struct rte_flow_action *
fm10k_next_item_of_action(const struct rte_flow_action *actions,
		uint32_t *index)
{
	static const struct rte_flow_action *act;

	act = actions + *index;
	while (act->type == RTE_FLOW_ACTION_TYPE_VOID) {
		(*index)++;
		act = actions + *index;
	}
	return act;
}

/* Find the first VOID or non-VOID item pointer */
static const struct rte_flow_item *
fm10k_find_first_item(const struct rte_flow_item *item, bool is_void)
{
	bool is_find;

	while (item->type != RTE_FLOW_ITEM_TYPE_END) {
		if (is_void)
			is_find = item->type == RTE_FLOW_ITEM_TYPE_VOID;
		else
			is_find = item->type != RTE_FLOW_ITEM_TYPE_VOID;
		if (is_find)
			break;
		item++;
	}
	return item;
}

/* Skip all VOID items of the pattern */
static void
fm10k_pattern_skip_void_item(struct rte_flow_item *items,
			    const struct rte_flow_item *pattern)
{
	uint32_t cpy_count = 0;
	const struct rte_flow_item *pb = pattern, *pe = pattern;

	for (;;) {
		/* Find a non-void item first */
		pb = fm10k_find_first_item(pb, false);
		if (pb->type == RTE_FLOW_ITEM_TYPE_END) {
			pe = pb;
			break;
		}

		/* Find a void item */
		pe = fm10k_find_first_item(pb + 1, true);

		cpy_count = pe - pb;
		rte_memcpy(items, pb, sizeof(struct rte_flow_item) * cpy_count);

		items += cpy_count;

		if (pe->type == RTE_FLOW_ITEM_TYPE_END) {
			pb = pe;
			break;
		}

		pb = pe + 1;
	}
	/* Copy the END item. */
	rte_memcpy(items, pe, sizeof(struct rte_flow_item));
}

/* Check if the pattern matches a supported item type array */
static bool
fm10k_match_pattern(enum rte_flow_item_type *item_array,
		   struct rte_flow_item *pattern)
{
	struct rte_flow_item *item = pattern;

	while ((*item_array == item->type) &&
	       (*item_array != RTE_FLOW_ITEM_TYPE_END)) {
		item_array++;
		item++;
	}

	return (*item_array == RTE_FLOW_ITEM_TYPE_END &&
		item->type == RTE_FLOW_ITEM_TYPE_END);
}

/* Find if there's parse filter function matched */
static parse_filter_t
fm10k_find_parse_filter_func(struct rte_flow_item *pattern, uint32_t *idx)
{
	parse_filter_t parse_filter = NULL;
	uint8_t i = *idx;

	for (; i < RTE_DIM(fm10k_supported_patterns); i++) {
		if (fm10k_match_pattern(fm10k_supported_patterns[i].items,
					pattern)) {
			parse_filter = fm10k_supported_patterns[i].parse_filter;
			break;
		}
	}

	*idx = ++i;

	return parse_filter;
}

/* Parse attributes */
static int
fm10k_flow_parse_attr(const struct rte_flow_attr *attr,
		     struct rte_flow_error *error)
{
	/* Not supported */
	if (attr->group) {
		rte_flow_error_set(error, EINVAL,
				   RTE_FLOW_ERROR_TYPE_ATTR_GROUP,
				   attr, "Not support group.");
		return -rte_errno;
	}

	return 0;
}

/*
 * MPLS
 */
/* 1. Last in item should be NULL as range is not supported.
 * 2. Supported filter types: MPLS label.
 * 3. Mask of fields which need to be matched should be
 *    filled with 1.
 * 4. Mask of fields which needn't to be matched should be
 *    filled with 0.
 */
static int
fm10k_flow_parse_mpls_pattern(__rte_unused struct rte_eth_dev *dev,
			     const struct rte_flow_item *pattern,
			     struct rte_flow_error *error,
			     struct fm10k_mpls_filter_conf *filter)
{
	const struct rte_flow_item *item = pattern;
	const struct rte_flow_item_mpls *mpls_spec;
	const struct rte_flow_item_mpls *mpls_mask;
	const struct rte_flow_item_eth *eth_spec;
	enum rte_flow_item_type item_type;
	const uint8_t label_mask[3] = {0xFF, 0xFF, 0xF0};
	uint32_t label_be = 0;
	uint32_t be_mask = 0;

	for (; item->type != RTE_FLOW_ITEM_TYPE_END; item++) {
		if (item->last) {
			rte_flow_error_set(error, EINVAL,
					   RTE_FLOW_ERROR_TYPE_ITEM,
					   item,
					   "Not support range");
			return -rte_errno;
		}
		item_type = item->type;
		switch (item_type) {
		case RTE_FLOW_ITEM_TYPE_ETH:
			if (!item->spec || item->mask) {
				rte_flow_error_set(error, EINVAL,
						   RTE_FLOW_ERROR_TYPE_ITEM,
						   item,
						   "Invalid ETH item");
				return -rte_errno;
			}
			eth_spec =
				(const struct rte_flow_item_eth *)item->spec;

			if (rte_be_to_cpu_16(eth_spec->type) == 0x8847) {
				filter->mpls_type = FM10K_MPLS_TYPE_UNI;
			} else if (rte_be_to_cpu_16(eth_spec->type) == 0x8848) {
				filter->mpls_type = FM10K_MPLS_TYPE_MULTI;
			} else {
				rte_flow_error_set(error, EINVAL,
					RTE_FLOW_ERROR_TYPE_ITEM,
					item, "Invalid ETH item");
				return -rte_errno;
			}
			break;
		case RTE_FLOW_ITEM_TYPE_MPLS:
			mpls_spec =
				(const struct rte_flow_item_mpls *)item->spec;
			mpls_mask =
				(const struct rte_flow_item_mpls *)item->mask;

			if (!mpls_spec || !mpls_mask) {
				rte_flow_error_set(error, EINVAL,
						   RTE_FLOW_ERROR_TYPE_ITEM,
						   item,
						   "Invalid MPLS item");
				return -rte_errno;
			}

			if (memcmp(mpls_mask->label_tc_s, label_mask, 3)) {
				rte_flow_error_set(error, EINVAL,
						   RTE_FLOW_ERROR_TYPE_ITEM,
						   item,
						   "Invalid MPLS label mask");
				return -rte_errno;
			}
			rte_memcpy(((uint8_t *)&label_be + 1),
					mpls_spec->label_tc_s, 3);
			rte_memcpy(((uint8_t *)&be_mask + 1),
					mpls_mask->label_tc_s, 3);
			filter->mpls_header =
					rte_be_to_cpu_32(label_be) >> 4;
			filter->mpls_header_mask =
					rte_be_to_cpu_32(be_mask) >> 4;

			fm10k_cons_filter_type = RTE_ETH_FILTER_TUNNEL;
			break;
		default:
			break;
		}
	}

	return 0;
}

/* MPLS action only supports QUEUE or DROP. */
static int
fm10k_flow_parse_mpls_action(const struct rte_flow_action *actions,
				 struct rte_flow_error *error,
				 struct fm10k_mpls_filter_conf *filter)
{
	const struct rte_flow_action *act;
	const struct rte_flow_action_queue *act_q;
	uint32_t index = 0;

	/* Check if the first non-void action is QUEUE or DROP. */
	act = fm10k_next_item_of_action(actions, &index);
	if (act->type != RTE_FLOW_ACTION_TYPE_QUEUE &&
	    act->type != RTE_FLOW_ACTION_TYPE_DROP) {
		rte_flow_error_set(error, EINVAL, RTE_FLOW_ERROR_TYPE_ACTION,
				   act, "Not supported action.");
		return -rte_errno;
	}

	if (act->type == RTE_FLOW_ACTION_TYPE_QUEUE) {
		act_q = (const struct rte_flow_action_queue *)act->conf;
		filter->mpls_action = FM10K_MPLS_ACTION_QUEUE;
		filter->queue = act_q->index;
	} else {
		filter->mpls_action = FM10K_MPLS_ACTION_DROP;
	}

	/* Check if the next non-void item is END */
	index++;
	act = fm10k_next_item_of_action(actions, &index);
	if (act->type != RTE_FLOW_ACTION_TYPE_END) {
		rte_flow_error_set(error, EINVAL, RTE_FLOW_ERROR_TYPE_ACTION,
				   act, "Not supported action.");
		return -rte_errno;
	}

	return 0;
}

static int
fm10k_flow_parse_mpls_filter(struct rte_eth_dev *dev,
			    const struct rte_flow_attr *attr,
			    const struct rte_flow_item pattern[],
			    const struct rte_flow_action actions[],
			    struct rte_flow_error *error,
			    union fm10k_filter_t *filter)
{
	struct fm10k_mpls_filter_conf *mpls_filter =
		&filter->mpls_filter;
	int ret;

	ret = fm10k_flow_parse_mpls_pattern(dev, pattern,
					   error, mpls_filter);
	if (ret)
		return ret;

	ret = fm10k_flow_parse_mpls_action(actions, error, mpls_filter);
	if (ret)
		return ret;

	ret = fm10k_flow_parse_attr(attr, error);
	return ret;
}

/*
 * VLAN
 */
static int
fm10k_flow_parse_vlan_pattern(__rte_unused struct rte_eth_dev *dev,
			     const struct rte_flow_item *pattern,
			     struct rte_flow_error *error,
			     struct fm10k_vlan_filter_conf *filter)
{
	const struct rte_flow_item *item = pattern;
	const struct rte_flow_item_vlan *vlan_spec;
	const struct rte_flow_item_phy_port *pp_spec;
	enum rte_flow_item_type item_type;

	PMD_INIT_LOG(DEBUG, "Parse vlan pattern ffu id %d", filter->ffu_id);

	for (; item->type != RTE_FLOW_ITEM_TYPE_END; item++) {
		if (item->last) {
			rte_flow_error_set(error, EINVAL,
					   RTE_FLOW_ERROR_TYPE_ITEM,
					   item,
					   "Not support range");
			return -rte_errno;
		}
		item_type = item->type;
		switch (item_type) {
		case RTE_FLOW_ITEM_TYPE_VLAN:
			vlan_spec =
				(const struct rte_flow_item_vlan *)item->spec;

			if (!vlan_spec) {
				rte_flow_error_set(error, EINVAL,
						   RTE_FLOW_ERROR_TYPE_ITEM,
						   item,
						   "Invalid VLAN item");
				return -rte_errno;
			}
			break;

		case RTE_FLOW_ITEM_TYPE_PHY_PORT:
			pp_spec =
			(const struct rte_flow_item_phy_port *)item->spec;
			if (!pp_spec) {
				rte_flow_error_set(error, EINVAL,
						   RTE_FLOW_ERROR_TYPE_ITEM,
						   item,
						   "Invalid PHY PORT item");
				return -rte_errno;
			}
			break;

		default:
			break;
		}
	}

	return 0;
}

static int
fm10k_flow_parse_vlan_action(struct rte_eth_dev *dev,
				 const struct rte_flow_action *actions,
				 struct rte_flow_error *error,
				 struct fm10k_vlan_filter_conf *filter)
{
	const struct rte_flow_action *act;
	uint32_t index = 0;

	PMD_INIT_LOG(DEBUG, "Parse vlan action name %s ffu id %d",
			dev->device->name, filter->ffu_id);

	/* Check if the first non-void action is QUEUE or DROP. */
	act = fm10k_next_item_of_action(actions, &index);
	if (act->type != RTE_FLOW_ACTION_TYPE_MARK) {
		rte_flow_error_set(error, EINVAL, RTE_FLOW_ERROR_TYPE_ACTION,
				   act, "Not supported action.");
		return -rte_errno;
	}

	index++;
	act = fm10k_next_item_of_action(actions, &index);
	if (act->type != RTE_FLOW_ACTION_TYPE_MARK &&
		act->type != RTE_FLOW_ACTION_TYPE_END) {
		rte_flow_error_set(error, EINVAL, RTE_FLOW_ERROR_TYPE_ACTION,
				   act, "Not supported action.");
		return -rte_errno;
	}
	if (act->type == RTE_FLOW_ACTION_TYPE_END)
		return 0;

	index++;
	/* Check if the next non-void item is END */
	act = fm10k_next_item_of_action(actions, &index);
	if (act->type != RTE_FLOW_ACTION_TYPE_END) {
		rte_flow_error_set(error, EINVAL, RTE_FLOW_ERROR_TYPE_ACTION,
				   act, "Not supported action.");
		return -rte_errno;
	}

	return 0;
}

static int
fm10k_flow_parse_vlan_filter(struct rte_eth_dev *dev,
			    const struct rte_flow_attr *attr,
			    const struct rte_flow_item pattern[],
			    const struct rte_flow_action actions[],
			    struct rte_flow_error *error,
			    union fm10k_filter_t *filter)
{
	int ret;
	struct fm10k_vlan_filter_conf *vlan_filter =
		&filter->vlan_filter;

	ret = fm10k_flow_parse_vlan_pattern(dev, pattern,
					   error, vlan_filter);
	if (ret)
		return ret;

	ret = fm10k_flow_parse_vlan_action(dev, actions, error, vlan_filter);
	if (ret)
		return ret;

	if (attr->ingress)
		vlan_filter->is_ingress = 1;
	else if (attr->egress)
		vlan_filter->is_ingress = 0;
	vlan_filter->ffu_prio = attr->priority;

	ret = fm10k_flow_parse_attr(attr, error);
	return ret;
}

/*
 *
 */
static int
fm10k_flow_validate(struct rte_eth_dev *dev,
		   const struct rte_flow_attr *attr,
		   const struct rte_flow_item pattern[],
		   const struct rte_flow_action actions[],
		   struct rte_flow_error *error)
{
	struct rte_flow_item *items; /* internal pattern w/o VOID items */
	parse_filter_t parse_filter;
	uint32_t item_num = 0; /* non-void item number of pattern*/
	uint32_t i = 0;
	bool flag = false;
	int ret = -1;

	if (!pattern) {
		rte_flow_error_set(error, EINVAL, RTE_FLOW_ERROR_TYPE_ITEM_NUM,
				   NULL, "NULL pattern.");
		return -rte_errno;
	}

	if (!actions) {
		rte_flow_error_set(error, EINVAL,
				   RTE_FLOW_ERROR_TYPE_ACTION_NUM,
				   NULL, "NULL action.");
		return -rte_errno;
	}

	if (!attr) {
		rte_flow_error_set(error, EINVAL,
				   RTE_FLOW_ERROR_TYPE_ATTR,
				   NULL, "NULL attribute.");
		return -rte_errno;
	}

	/* Get the non-void item number of pattern */
	while ((pattern + i)->type != RTE_FLOW_ITEM_TYPE_END) {
		if ((pattern + i)->type != RTE_FLOW_ITEM_TYPE_VOID)
			item_num++;
		i++;
	}
	item_num++;

	items = rte_zmalloc("fm10k_pattern",
			    item_num * sizeof(struct rte_flow_item), 0);
	if (!items) {
		rte_flow_error_set(error, ENOMEM, RTE_FLOW_ERROR_TYPE_ITEM_NUM,
				   NULL, "No memory for PMD internal items.");
		return -ENOMEM;
	}

	fm10k_pattern_skip_void_item(items, pattern);

	i = 0;
	do {
		parse_filter = fm10k_find_parse_filter_func(items, &i);
		if (!parse_filter && !flag) {
			rte_flow_error_set(error, EINVAL,
					   RTE_FLOW_ERROR_TYPE_ITEM,
					   pattern, "Unsupported pattern");
			rte_free(items);
			return -rte_errno;
		}
		if (parse_filter)
			ret = parse_filter(dev, attr, items, actions,
					   error, &cons_filter);
		flag = true;
	} while ((ret < 0) && (i < RTE_DIM(fm10k_supported_patterns)));

	rte_free(items);

	return ret;
}

static struct fm10k_cfg_flow *
fm10k_flow_cfg_transfer(struct rte_eth_dev *dev,
		 const struct rte_flow_attr *attr,
		 const struct rte_flow_item pattern[],
		 const struct rte_flow_action actions[],
		 struct rte_flow_error *error)
{
	int i;
	u8 port_id;
	int set_port_num = 0, set_vlan_num = 0;
	u16 fw_port_id = 0, bp_port_id = 0;
	u16 filter_vlan_id = 0, fw_vlan_id = 0, bp_vlan_id = 0;
	struct fm10k_hw *hw = FM10K_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	struct fm10k_cfg_flow *cf;

	cf = rte_zmalloc("fm10k_rule", sizeof(struct fm10k_cfg_flow), 0);
	if (!cf) {
		rte_flow_error_set(error, ENOMEM,
				   RTE_FLOW_ERROR_TYPE_HANDLE, NULL,
				   "Failed to allocate memory");
		return NULL;
	}
	memset(cf, 0, sizeof(struct fm10k_cfg_flow));

	port_id = fm10k_switch_dpdk_port_no_get(hw);
	for (i = 0; i < 4; i++)	{
		if (pattern[i].type == RTE_FLOW_ITEM_TYPE_VLAN) {
			filter_vlan_id =
				rte_be_to_cpu_16
				(((const struct rte_flow_item_vlan *)
				pattern[i].spec)->tci);
		} else if (pattern[i].type == RTE_FLOW_ITEM_TYPE_PHY_PORT) {
			if (set_port_num)
				bp_port_id =
					((const struct rte_flow_item_phy_port *)
					pattern[i].spec)->index;
			else
				fw_port_id =
					((const struct rte_flow_item_phy_port *)
					pattern[i].spec)->index;
			set_port_num++;
		} else if (pattern[i].type == RTE_FLOW_ITEM_TYPE_END) {
			break;
		}
	}

	for (i = 0; i < 3; i++)	{
		if (actions[i].type == RTE_FLOW_ACTION_TYPE_MARK) {
			if (set_vlan_num)
				bp_vlan_id =
					((const struct rte_flow_action_mark *)
					actions[i].conf)->id;
			else
				fw_vlan_id =
					((const struct rte_flow_action_mark *)
					actions[i].conf)->id;
			set_vlan_num++;
		} else if (actions[i].type == RTE_FLOW_ACTION_TYPE_END) {
			break;
		}
	}

	if (attr->ingress && !attr->egress)	{
		/* this port is DPDK port and it is destination port */
		cf->src_port.port_type = FM10K_CONFIG_FLOW_EXT_PORT;
		cf->src_port.port_no = fw_port_id;
		cf->src_port.vlan_id = filter_vlan_id;
		cf->fw_port[0].port_type = FM10K_CONFIG_FLOW_DPDK_PORT;
		cf->fw_port[0].port_no = port_id;
		cf->fw_port[0].vlan_id = fw_vlan_id;
	} else if (!attr->ingress && attr->egress) {
		/* this port is DPDK port and it is source port */
		cf->src_port.port_type = FM10K_CONFIG_FLOW_DPDK_PORT;
		cf->src_port.port_no = port_id;
		cf->src_port.vlan_id = filter_vlan_id;
		cf->fw_port[0].port_type = FM10K_CONFIG_FLOW_EXT_PORT;
		cf->fw_port[0].port_no = fw_port_id;
		cf->fw_port[0].vlan_id = fw_vlan_id;
	} else if (!attr->ingress && !attr->egress) {
		/* two ports are external port */
		cf->src_port.port_type = FM10K_CONFIG_FLOW_EXT_PORT;
		cf->src_port.port_no = port_id;
		cf->src_port.vlan_id = filter_vlan_id;
		cf->fw_port[0].port_type = FM10K_CONFIG_FLOW_EXT_PORT;
		cf->fw_port[0].port_no = fw_port_id;
		cf->fw_port[0].vlan_id = fw_vlan_id;
	} else {
		/* two ports are DPDK port */
		cf->src_port.port_type = FM10K_CONFIG_FLOW_DPDK_PORT;
		cf->src_port.port_no = port_id;
		cf->src_port.vlan_id = filter_vlan_id;
		cf->fw_port[0].port_type = FM10K_CONFIG_FLOW_DPDK_PORT;
		cf->fw_port[0].port_no = fw_port_id;
		cf->fw_port[0].vlan_id = fw_vlan_id;
	}

	if (set_port_num == 2 && set_vlan_num == 2)	{
		cf->fw_port[1].port_type = FM10K_CONFIG_FLOW_EXT_PORT;
		cf->fw_port[1].port_no = bp_port_id;
		cf->fw_port[1].vlan_id = bp_vlan_id;
	}

	return cf;
}

static struct rte_flow *
fm10k_flow_create(struct rte_eth_dev *dev,
		 const struct rte_flow_attr *attr,
		 const struct rte_flow_item pattern[],
		 const struct rte_flow_action actions[],
		 struct rte_flow_error *error)
{
	struct fm10k_hw *hw = FM10K_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	struct rte_flow *flow;
	struct fm10k_switch *sw = fm10k_switch_get();
	struct fm10k_cfg_flow *cf;
	int ret;

	flow = rte_zmalloc("fm10k_flow", sizeof(struct rte_flow), 0);
	if (!flow) {
		rte_flow_error_set(error, ENOMEM,
				   RTE_FLOW_ERROR_TYPE_HANDLE, NULL,
				   "Failed to allocate memory");
		return flow;
	}

	ret = fm10k_flow_validate(dev, attr, pattern, actions, error);
	if (ret < 0)
		return NULL;

	cf = fm10k_flow_cfg_transfer(dev, attr, pattern, actions, error);
	if (!cf)
		goto free_flow;

	flow->rule = cf;
	fm10k_ffu_flow_enable(sw, cf);
	fm10k_config_flow_list_add_tail(fm10k_config_flowset_current_get(), cf);

	TAILQ_INSERT_TAIL((struct fm10k_flow_list *)
			fm10k_switch_dpdk_port_flow_list_get(hw), flow, node);
	return flow;

free_flow:
	rte_flow_error_set(error, -ret,
			   RTE_FLOW_ERROR_TYPE_HANDLE, NULL,
			   "Failed to create flow.");
	rte_free(flow);
	return NULL;
}

static int
fm10k_flow_destroy(struct rte_eth_dev *dev,
		  struct rte_flow *flow,
		  struct rte_flow_error *error)
{
	struct fm10k_hw *hw = FM10K_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	int ret = 0;
	struct fm10k_switch *sw = fm10k_switch_get();

	if (flow->rule)	{
		fm10k_config_flow_list_delete
			((struct fm10k_cfg_flow *)flow->rule);
		fm10k_ffu_flow_disable(sw,
			(struct fm10k_cfg_flow *)flow->rule);
	}

	if (!ret) {
		TAILQ_REMOVE((struct fm10k_flow_list *)
				fm10k_switch_dpdk_port_flow_list_get(hw),
				flow, node);
		rte_free(flow->rule);
		rte_free(flow);
	} else {
		rte_flow_error_set(error, -ret,
				   RTE_FLOW_ERROR_TYPE_HANDLE, NULL,
				   "Failed to destroy flow.");
	}
	return ret;
}

static int
fm10k_flow_flush(struct rte_eth_dev *dev, struct rte_flow_error *error)
{
	struct fm10k_hw *hw = FM10K_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	struct rte_flow *flow;
	void *temp;
	int ret = 0;
	struct fm10k_cfg_flow *cf;
	struct fm10k_switch *sw = fm10k_switch_get();

	/* Delete flows in flow list. */
	TAILQ_FOREACH_SAFE(flow,
			(struct fm10k_flow_list *)
			fm10k_switch_dpdk_port_flow_list_get(hw),
			node, temp) {
		cf = flow->rule;
		if (cf) {
			fm10k_config_flow_list_delete(cf);
			fm10k_ffu_flow_disable(sw, cf);
		} else {
			rte_flow_error_set(error, -ret,
					   RTE_FLOW_ERROR_TYPE_HANDLE, NULL,
					   "No such rule in flow.");
		}
		TAILQ_REMOVE((struct fm10k_flow_list *)
				fm10k_switch_dpdk_port_flow_list_get(hw),
				flow, node);
		rte_free(flow);
	}

	return ret;
}

void
fm10k_flow_list_init(void *flow_list)
{
	TAILQ_INIT((struct fm10k_flow_list *)flow_list);
}

/* Flow operations */
const struct rte_flow_ops *
fm10k_flow_ops_get(void)
{
	return &fm10k_flow_ops;
}
