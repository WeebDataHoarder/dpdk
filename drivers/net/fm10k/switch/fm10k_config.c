/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright 2019   Silicom Ltd. Connectivity Solutions
 */

#include <unistd.h>
#include <rte_time.h>
#include <rte_kvargs.h>
#include <rte_hash.h>
#include <rte_flow.h>
#include <rte_flow_driver.h>
#include <rte_tm_driver.h>

#include "../base/fm10k_type.h"
#include "../base/fm10k_osdep.h"
#include "../fm10k.h"
#include "../fm10k_logs.h"
#include "fm10k_debug.h"
#include "fm10k_regs.h"
#include "fm10k_switch.h"
#include "fm10k_config.h"

#define FM10K_CONFIG_WORD_MAX			10
#define FM10K_CONFIG_WORD_LEN			20
#define FM10K_CONFIG_STR_MAX			100

/* used for store the key strings */
struct fm10k_cfg_key_item {
	const char *key_str[FM10K_CONFIG_WORD_MAX];
	uint8_t type;
};

/* configuration file path */
static const char *fm10k_config_dpdk_conf_file = "/etc/dpdk_fm10k.conf";
/* configuration structure */
static struct fm10k_dpdk_cfg fm10k_config_dpdk_cfg;

/* configuration key strings */
static struct fm10k_cfg_key_item fm10k_config_key_items[] = {
		/* enable debug print */
		{ {"debug", "print", "enable"},
			FM10K_CONFIG_DEBUG_ENABLE},
		{ {"debug", "print", "config"},
			FM10K_CONFIG_DEBUG_CONFIG},
		{ {"debug", "print", "ffu", "init"},
			FM10K_CONFIG_DEBUG_FFU_INIT},
		{ {"debug", "print", "ffu", "register"},
			FM10K_CONFIG_DEBUG_FFU_REG},
		{ {"debug", "print", "ffu", "rule"},
			FM10K_CONFIG_DEBUG_FFU_RULE},
		{ {"debug", "print", "stats", "port"},
			FM10K_CONFIG_DEBUG_STATS_PORT},
		{ {"debug", "print", "stats", "queue"},
			FM10K_CONFIG_DEBUG_STATS_QUEUE},
		{ {"debug", "print", "stats", "rule"},
			FM10K_CONFIG_DEBUG_STATS_FFU},
		{ {"debug", "print", "stats", "detail"},
			FM10K_CONFIG_DEBUG_STATS_MORE},
		{ {"debug", "print", "stats", "interval"},
			FM10K_CONFIG_DEBUG_STATS_INTERVAL},
		/* general configuration */
		{ {"dpdk", "bind", "pf", "number"},
			FM10K_CONFIG_BIND_PF_NUMBER},
		{ {"extern", "port", "speed"},
			FM10K_CONFIG_EXT_PORT_SPEED},
		/* internal redirect configuration */
		{ {"dpdk", "port", "*", "map", "pf"},
			FM10K_CONFIG_DPDK_PORT_MAP_PF},
		{ {"extern", "port", "*", "map", "pf"},
			FM10K_CONFIG_EXT_PORT_MAP_PF},
		/* external redirect configuration */
		{ {"flowset", "start"},
			FM10K_CONFIG_FLOWSET_START},
		{ {"flowset", "stop"},
			FM10K_CONFIG_FLOWSET_STOP},
		{ {"flowset", "enable"},
			FM10K_CONFIG_FLOWSET_ENABLE},
		{ {"flow", "*", "condition", "source", "extern", "port"},
			FM10K_CONFIG_FLOW_COND_SRC_EXT_PORT},
		{ {"flow", "*", "condition", "source", "dpdk", "port"},
			FM10K_CONFIG_FLOW_COND_SRC_DPDK_PORT},
		{ {"flow", "*", "condition", "vlan"},
			FM10K_CONFIG_FLOW_COND_VLAN},
		{ {"flow", "*", "action", "forward", "extern", "port"},
			FM10K_CONFIG_FLOW_ACT_FW_EXT_PORT},
		{ {"flow", "*", "action", "forward", "dpdk", "port"},
			FM10K_CONFIG_FLOW_ACT_FW_DPDK_PORT},
		{ {"flow", "*", "action", "forward", "vlan"},
			FM10K_CONFIG_FLOW_ACT_FW_VALN},
};

static char default_flowset[10] = "default";

static struct fm10k_cfg_config_item fm10k_silc_nic_2ext_2pep[] = {
	{   FM10K_CONFIG_BIND_PF_NUMBER, FM10K_CONFIG_VALUE_INT, 0,
		"# Tell how many PF ports are bound in DPDK.\n"
		"# Driver will check the number when initializes.",
		.val.int64 = 2
	},
	{   FM10K_CONFIG_EXT_PORT_SPEED, FM10K_CONFIG_VALUE_INT, 0,
		"# Set external port speed, 40 means 40G, 100 means 100G.",
		.val.int64 = 100
	},
	{   FM10K_CONFIG_DPDK_PORT_MAP_PF, FM10K_CONFIG_VALUE_INT, 0,
		"# Map 1 or 2 PF ports to one DPDK port.\n"
		"# If mapped PF number is 2, traffic will be\n"
		"# load balance between the 2 PF.\n"
		"# And the DPDK port queue number will be configured\n"
		"# more than 2(each PF need at least 1 DPDK port queue).",
		.val.int64 = 0
	},
	{   FM10K_CONFIG_DPDK_PORT_MAP_PF, FM10K_CONFIG_VALUE_INT, 1,
		"",
		.val.int64 = 1
	},
	{   FM10K_CONFIG_EXT_PORT_MAP_PF, FM10K_CONFIG_VALUE_INT, 1,
		"# Map 1 or 2 PF to one external port. If mapped PF number is 2,\n"
		"# traffic will be load balance between the 2 PF. ",
		.val.int64 = 0
	},
	{   FM10K_CONFIG_EXT_PORT_MAP_PF, FM10K_CONFIG_VALUE_INT, 2,
		"",
		.val.int64 = 1
	},
	{   FM10K_CONFIG_FLOWSET_START, FM10K_CONFIG_VALUE_STR, 0,
		"# Define flow rule",
		.val.str = default_flowset
	},
	{   FM10K_CONFIG_FLOWSET_STOP, FM10K_CONFIG_VALUE_STR, 0,
		"",
		.val.str = default_flowset
	},
	{   FM10K_CONFIG_FLOWSET_ENABLE, FM10K_CONFIG_VALUE_STR, 0,
		"",
		.val.str = default_flowset
	},
	{   FM10K_CONFIG_TYPE_NULL, 0, 0,
		"",
		.val.int64 = 0
	},
};

static struct fm10k_cfg_config_item fm10k_silc_nic_2ext_4pep[] = {
	{   FM10K_CONFIG_BIND_PF_NUMBER, FM10K_CONFIG_VALUE_INT, 0,
		"# Tell how many PF ports are bound in DPDK.\n"
		"# Driver will check the  number when initializes.",
		.val.int64 = 4
	},

	{   FM10K_CONFIG_EXT_PORT_SPEED, FM10K_CONFIG_VALUE_INT, 0,
		"# Set external port speed, 40 means 40G, 100 means 100G.",
		.val.int64 = 100
	},
	{   FM10K_CONFIG_DPDK_PORT_MAP_PF, FM10K_CONFIG_VALUE_INT, 0,
		"# Map 1 or 2 PF ports to one DPDK port.\n"
		"# If mapped PF number is 2, traffic will be\n"
		"# load balance between the 2 PFs.\n"
		"# And the DPDK port queue number will be configured\n"
		"# more than 2(each PF need at least 1 DPDK port queue).",
		.val.int64 = 0
	},
	{   FM10K_CONFIG_DPDK_PORT_MAP_PF, FM10K_CONFIG_VALUE_INT, 0,
		"",
		.val.int64 = 2
	},
	{   FM10K_CONFIG_DPDK_PORT_MAP_PF, FM10K_CONFIG_VALUE_INT, 1,
		"",
		.val.int64 = 1
	},
	{   FM10K_CONFIG_DPDK_PORT_MAP_PF, FM10K_CONFIG_VALUE_INT, 1,
		"",
		.val.int64 = 3
	},
	{   FM10K_CONFIG_EXT_PORT_MAP_PF, FM10K_CONFIG_VALUE_INT, 1,
		"# Map 1 or 2 PF to one external port. If mapped PF number is 2,\n"
		"# traffic will be load balance between the 2 PF.",
		.val.int64 = 0
	},
	{   FM10K_CONFIG_EXT_PORT_MAP_PF, FM10K_CONFIG_VALUE_INT, 1,
		"",
		.val.int64 = 2
	},
	{   FM10K_CONFIG_EXT_PORT_MAP_PF, FM10K_CONFIG_VALUE_INT, 2,
		"",
		.val.int64 = 1
	},
	{   FM10K_CONFIG_EXT_PORT_MAP_PF, FM10K_CONFIG_VALUE_INT, 2,
		"",
		.val.int64 = 3
	},
	{   FM10K_CONFIG_FLOWSET_START, FM10K_CONFIG_VALUE_STR, 0,
		"# Define flow rule",
		.val.str = default_flowset
	},
	{   FM10K_CONFIG_FLOWSET_STOP, FM10K_CONFIG_VALUE_STR, 0,
		"",
		.val.str = default_flowset
	},
	{   FM10K_CONFIG_FLOWSET_ENABLE, FM10K_CONFIG_VALUE_STR, 0,
		"",
		.val.str = default_flowset
	},
	{   FM10K_CONFIG_TYPE_NULL, 0, 0,
		"",
		.val.int64 = 0
	},
};

static int
fm10k_config_conf_line_parser(char *buff, struct fm10k_cfg_config_item *item)
{
	char *p;
	char *endptr;
	const char *cfg_delims = { " " };
	const char *key_delims = { "." };
	struct fm10k_cfg_key_item *key;
	char cfgs[3][FM10K_CONFIG_STR_MAX];
	char cmp_keys[FM10K_CONFIG_WORD_MAX][FM10K_CONFIG_WORD_LEN];
	long key_param = 0;
	uint16_t i, j;
	uint8_t val_type = FM10K_CONFIG_VALUE_NULL;

	for (i = 0; i < 3; i++) {
		if (i == 0)
			p = strtok(buff, cfg_delims);
		else
			p = strtok(NULL, cfg_delims);
		if (p == NULL)
			return -1;
		strncpy(cfgs[i], p, FM10K_CONFIG_STR_MAX);
	}

	p = strtok(NULL, cfg_delims);
	if (p)
		return -1;

	memset(cmp_keys, 0, sizeof(cmp_keys));
	for (i = 0; i < FM10K_CONFIG_WORD_MAX; i++) {
		if (i == 0)
			p = strtok(cfgs[0], key_delims);
		else
			p = strtok(NULL, key_delims);
		if (p == NULL)
			break;
		strncpy(cmp_keys[i], p, FM10K_CONFIG_WORD_LEN);
	}

	if (strcmp(cfgs[1], "int") == 0)
		val_type = FM10K_CONFIG_VALUE_INT;
	else if (strcmp(cfgs[1], "string") == 0)
		val_type = FM10K_CONFIG_VALUE_STR;
	else
		return -1;

	for (i = 0; i < sizeof(fm10k_config_key_items) /
			sizeof(fm10k_config_key_items[0]); i++) {
		key = &fm10k_config_key_items[i];
		for (j = 0; j < FM10K_CONFIG_WORD_MAX; j++)	{
			if (key->key_str[j] &&
					strlen(cmp_keys[j]) > 0) {
				if (strcmp(key->key_str[j], "*") == 0) {
					key_param =
					strtol(cmp_keys[j], &endptr, 10);
					if ((key_param == 0 &&
						endptr == cmp_keys[j]) ||
						(endptr != cmp_keys[j] &&
						strlen(endptr) != 0))
						break;
				} else if (strcmp(key->key_str[j],
							cmp_keys[j]) != 0) {
					break;
				}
			} else if (key->key_str[j] == 0 &&
					strlen(cmp_keys[j]) == 0) {
				if (val_type == FM10K_CONFIG_VALUE_STR) {
					item->val.str =
					malloc(strlen(cfgs[2]) + 1);
					if (item->val.str == NULL)
						return -1;
					strcpy(item->val.str, cfgs[2]);
				} else {
					item->val.int64 =
					strtol(cfgs[2], NULL, 10);
				}
				item->type = key->type;
				item->key_param = key_param;
				item->val_type = val_type;
				return 0;
			}
		}
	}
	return -1;
}

static bool
fm10k_config_blank_line_check(char *line)
{
	uint16_t i;

	for (i = 0; i < strlen(line); i++) {
		if (line[i] == ' ' ||
				line[i] == '\n' ||
				line[i] == '\t' || line[i] == 0x0d) /* 0d: CR */
			continue;
		else
			return false;
	}
	return true;
}

static int
fm10k_config_conf_file_load(void)
{
	int i = 0;
	FILE *fp;
	char buff[255];
	struct fm10k_cfg_config_item *item;

	fp = fopen(fm10k_config_dpdk_conf_file, "r");
	if (fp == NULL)
		return -1;

	fm10k_config_dpdk_cfg.config_list =	malloc
		(sizeof(struct fm10k_cfg_config_item) * FM10K_SW_CONFIG_MAX);
	if (fm10k_config_dpdk_cfg.config_list == NULL)
		return -1;
	memset(fm10k_config_dpdk_cfg.config_list, 0,
		sizeof(struct fm10k_cfg_config_item) * FM10K_SW_CONFIG_MAX);

	while (fgets(buff, sizeof(buff), fp)) {
		if (buff[0] == '#' || buff[0] == 0)
			continue;

		if (fm10k_config_blank_line_check(buff))
			continue;

		item = &fm10k_config_dpdk_cfg.config_list[i++];
		if (fm10k_config_conf_line_parser(buff, item) < 0) {
			FM10K_SW_ERR("Unknown configuration: %s", buff);
			return -1;
		}
	}

	return 0;
}

struct fm10k_hw*
fm10k_config_hw_get(int port_no)
{
	int hw_port;

	hw_port = fm10k_config_dpdk_cfg.dpdk_port_map[port_no].map_no[0];
	return fm10k_config_dpdk_cfg.pf_hw[hw_port];
}

struct fm10k_cfg_flowset *
fm10k_config_flowset_current_get(void)
{
	return fm10k_config_dpdk_cfg.current;
}

void
fm10k_config_flowset_current_set(struct fm10k_cfg_flowset *new)
{
	fm10k_config_dpdk_cfg.current = new;
}

bool
fm10k_config_flow_list_end(struct fm10k_cfg_flow *list,
		struct fm10k_cfg_flow *flow)
{
	return (list == flow);
}

static void
fm10k_config_flow_list_init(struct fm10k_cfg_flow *list)
{
	list->next = list;
	list->prev = list;
}

static void
fm10k_config_flow_list_add(struct fm10k_cfg_flow *new,
		struct fm10k_cfg_flow *list)
{
	struct fm10k_cfg_flow *next = list->next;
	next->prev = new;
	new->next = next;
	new->prev = list;
	list->next = new;
}

void
fm10k_config_flow_list_add_tail(struct fm10k_cfg_flowset *flowset,
		struct fm10k_cfg_flow *flow)
{
	fm10k_config_flow_list_add(flow, flowset->flow_head.prev);
}

void
fm10k_config_flow_list_delete(struct fm10k_cfg_flow *flow)
{
	flow->prev->next = flow->next;
	flow->next->prev = flow->prev;
	flow->prev = NULL;
	flow->next = NULL;
}

struct fm10k_cfg_flowset *
fm10k_config_flowset_get(const char *name)
{
	struct fm10k_cfg_flowset *flowset;

	flowset = fm10k_config_dpdk_cfg.flowset_head.next;
	while (flowset) {
		if (strcmp(flowset->name, name) == 0)
			break;
		flowset = flowset->next;
	}

	if (flowset == NULL) {
		flowset = malloc(sizeof(struct fm10k_cfg_flowset));
		if (flowset == NULL)
			return NULL;
		strcpy(flowset->name, name);
		fm10k_config_flow_list_init(&flowset->flow_head);
		flowset->next = fm10k_config_dpdk_cfg.flowset_head.next;
		fm10k_config_dpdk_cfg.flowset_head.next = flowset;
	}
	return flowset;
}

static int
fm10k_cfg_flow_item_set(struct fm10k_cfg_flowset *flowset,
		uint8_t flow_no, uint8_t type, int64_t val)
{
	struct fm10k_cfg_flow *tmp;
	struct fm10k_cfg_flow *flow = NULL;

	if (flowset == NULL)
		return -1;

	tmp = flowset->flow_head.next;
	while (!fm10k_config_flow_list_end(&flowset->flow_head, tmp)) {
		if (tmp->flow_no == flow_no) {
			flow = tmp;
			break;
		}
		tmp = tmp->next;
	}
	if (flow == NULL) {
		flow = malloc(sizeof(struct fm10k_cfg_flow));
		memset(flow, 0, sizeof(struct fm10k_cfg_flow));
		flow->flow_no = flow_no;
		fm10k_config_flow_list_add_tail(flowset, flow);
	}
	if (flow == NULL)
		return -1;

	switch (type) {
	case FM10K_CONFIG_FLOW_COND_SRC_EXT_PORT:
		flow->src_port.port_type = FM10K_CONFIG_FLOW_EXT_PORT;
		flow->src_port.port_no = val;
		break;
	case FM10K_CONFIG_FLOW_COND_SRC_DPDK_PORT:
		flow->src_port.port_type = FM10K_CONFIG_FLOW_DPDK_PORT;
		flow->src_port.port_no = val;
		break;
	case FM10K_CONFIG_FLOW_COND_VLAN:
		flow->src_port.vlan_id = val;
		break;
	case FM10K_CONFIG_FLOW_ACT_FW_EXT_PORT:
		if (flow->fw_port[0].port_type == FM10K_CONFIG_FLOW_NONE_PORT) {
			flow->fw_port[0].port_type = FM10K_CONFIG_FLOW_EXT_PORT;
			flow->fw_port[0].port_no = val;
		} else {
			flow->fw_port[1].port_type = FM10K_CONFIG_FLOW_EXT_PORT;
			flow->fw_port[1].port_no = val;
		}
		break;
	case FM10K_CONFIG_FLOW_ACT_FW_DPDK_PORT:
		if (flow->fw_port[0].port_type == FM10K_CONFIG_FLOW_NONE_PORT) {
			flow->fw_port[0].port_type =
					FM10K_CONFIG_FLOW_DPDK_PORT;
			flow->fw_port[0].port_no = val;
		} else {
			flow->fw_port[1].port_type =
					FM10K_CONFIG_FLOW_DPDK_PORT;
			flow->fw_port[1].port_no = val;
		}
		break;
	case FM10K_CONFIG_FLOW_ACT_FW_VALN:
		if (flow->fw_port[0].vlan_id == 0)
			flow->fw_port[0].vlan_id = val;
		else
			flow->fw_port[1].vlan_id = val;
		break;
	default:
		return -1;
	}

	return 0;
}

static int
fm10k_config_conf_file_transfer(void)
{
	int i;
	int offset;
	uint32_t tmp, data;
	struct fm10k_cfg_config_item *item;
	struct fm10k_cfg_flowset *flowset = NULL;
	struct fm10k_cfg_port_pf_map *map;

	for (i = 0; i < FM10K_SW_CONFIG_MAX; i++) {
		item = &fm10k_config_dpdk_cfg.config_list[i];
		if (item->type == FM10K_CONFIG_TYPE_NULL)
			break;

		switch (item->type)	{
		case FM10K_CONFIG_BIND_PF_NUMBER:
			fm10k_config_dpdk_cfg.pf_num = item->val.int64;
			break;
		case FM10K_CONFIG_EXT_PORT_SPEED:
			fm10k_config_dpdk_cfg.ext_port_speed = item->val.int64;
			break;
		case FM10K_CONFIG_DPDK_PORT_MAP_PF:
			map =
			&fm10k_config_dpdk_cfg.dpdk_port_map[item->key_param];
			if (map->type == FM10K_CONFIG_PORT_MAP_PF) {
				map->type = FM10K_CONFIG_PORT_MAP_PFS;
				map->map_no[1] = item->val.int64;
			} else {
				map->type = FM10K_CONFIG_PORT_MAP_PF;
				map->map_no[0] = item->val.int64;
			}
			break;
		case FM10K_CONFIG_EXT_PORT_MAP_PF:
			map =
				&fm10k_config_dpdk_cfg.ext_port_map
				[item->key_param - 1];
			if (map->type == FM10K_CONFIG_PORT_MAP_PF) {
				map->type = FM10K_CONFIG_PORT_MAP_PFS;
				map->map_no[1] = item->val.int64;
			} else {
				map->type = FM10K_CONFIG_PORT_MAP_PF;
				map->map_no[0] = item->val.int64;
			}
			break;

		case FM10K_CONFIG_DEBUG_ENABLE:
		case FM10K_CONFIG_DEBUG_CONFIG:
		case FM10K_CONFIG_DEBUG_FFU_INIT:
		case FM10K_CONFIG_DEBUG_FFU_REG:
		case FM10K_CONFIG_DEBUG_FFU_RULE:
		case FM10K_CONFIG_DEBUG_STATS_PORT:
		case FM10K_CONFIG_DEBUG_STATS_QUEUE:
		case FM10K_CONFIG_DEBUG_STATS_FFU:
		case FM10K_CONFIG_DEBUG_STATS_MORE:
			offset = item->type - FM10K_CONFIG_DEBUG_START;
			tmp = fm10k_config_dpdk_cfg.debug_cfg;
			data = 1;
			if (item->val.int64 != 1)
				fm10k_config_dpdk_cfg.debug_cfg =
						tmp & ~(data << offset);
			else
				fm10k_config_dpdk_cfg.debug_cfg |=
						(data << offset);
			break;
		case FM10K_CONFIG_DEBUG_STATS_INTERVAL:
			fm10k_config_dpdk_cfg.stats_interval = item->val.int64;
			break;

		case FM10K_CONFIG_FLOWSET_START:
			/* skip /n */
			item->val.str[strlen(item->val.str) - 1] = 0;
			flowset = fm10k_config_flowset_get(item->val.str);
			if (flowset == NULL)
				return -1;
			break;
		case FM10K_CONFIG_FLOWSET_STOP:
			flowset = NULL;
			break;
		case FM10K_CONFIG_FLOWSET_ENABLE:
			/* skip /n */
			item->val.str[strlen(item->val.str) - 1] = 0;
			fm10k_config_dpdk_cfg.current =
					fm10k_config_flowset_get(item->val.str);
			break;
		case FM10K_CONFIG_FLOW_COND_SRC_EXT_PORT:
		case FM10K_CONFIG_FLOW_COND_SRC_DPDK_PORT:
		case FM10K_CONFIG_FLOW_COND_VLAN:
		case FM10K_CONFIG_FLOW_ACT_FW_EXT_PORT:
		case FM10K_CONFIG_FLOW_ACT_FW_DPDK_PORT:
		case FM10K_CONFIG_FLOW_ACT_FW_VALN:
			if (flowset == NULL ||
					fm10k_cfg_flow_item_set(flowset,
					item->key_param, item->type,
					item->val.int64) != 0)
				return -1;
			break;
		default:
			return -1;
		}
	}
	return 0;
}

static bool
fm10k_config_conf_file_exist(void)
{
	if (access(fm10k_config_dpdk_conf_file, 0) == 0)
		return true;
	return false;
}

static int
fm10k_config_conf_file_item_create(FILE *fp, char *buff,
		struct fm10k_cfg_key_item *key,
		struct fm10k_cfg_config_item *item)
{
	int i;

	if (item->type != key->type)
		return 0;

	for (i = 0; i < FM10K_CONFIG_WORD_MAX; i++) {
		if (key->key_str[i] == 0)
			break;
		if (i == 0) {
			sprintf(buff + strlen(buff),
				"%s", key->key_str[i]);
		} else {
			if (strcmp(key->key_str[i], "*") == 0)
				sprintf(buff + strlen(buff),
						".%u", item->key_param);
			else
				sprintf(buff + strlen(buff),
						".%s", key->key_str[i]);
		}
	}
	if (item->val_type == FM10K_CONFIG_VALUE_INT)
		sprintf(buff + strlen(buff),
				" int %lld\n", (long long)item->val.int64);
	else if (item->val_type == FM10K_CONFIG_VALUE_STR)
		sprintf(buff + strlen(buff), " string %s\n", item->val.str);
	else
		return -1;
	fwrite(buff, strlen(buff), 1, fp);
	FM10K_SW_TRACE("[write] %s", buff);

	return 0;
}

static int
fm10k_config_conf_file_create(void)
{
	uint16_t i, j;
	struct fm10k_cfg_key_item *key;
	struct fm10k_cfg_config_item *item;
	FILE *fp;
	char buff[255] = "";

	fp = fopen(fm10k_config_dpdk_conf_file, "w");
	if (fp == NULL)
		return -1;

	for (i = 0; i < FM10K_SW_CONFIG_MAX; i++) {
		item = &fm10k_config_dpdk_cfg.config_list[i];
		if (item->type == FM10K_CONFIG_TYPE_NULL)
			break;
		buff[0] = 0;
		if (strlen(item->describe) > 0)
			sprintf(buff, "\n\n%s\n", item->describe);
		for (j = 0; j < sizeof(fm10k_config_key_items) /
				sizeof(fm10k_config_key_items[0]); j++) {
			key = &fm10k_config_key_items[j];
			if (fm10k_config_conf_file_item_create
					(fp, buff, key, item) != 0) {
				fclose(fp);
				return -1;
			}
		}
	}
	fclose(fp);
	return 0;
}

void
fm10k_config_cfg_flowset_show(void)
{
	struct fm10k_cfg_flowset *flowset;

	FM10K_SW_INFO("  FLOWSET ENABLE: %s\n",
			fm10k_config_dpdk_cfg.current->name);

	flowset = fm10k_config_dpdk_cfg.flowset_head.next;
	while (flowset) {
		FM10K_SW_INFO("\n  FLOWSET  : %s\n", flowset->name);
		struct fm10k_cfg_flow *flow = flowset->flow_head.next;
		while (!fm10k_config_flow_list_end(&flowset->flow_head, flow)) {
			const char *port_type[3] = { "NON", "EXT", "DPDK" };
			if (flow->fw_port[1].port_type !=
					FM10K_CONFIG_FLOW_NONE_PORT) {
				FM10K_SW_INFO("  FLOW %d : %4s PORT %d VLAN %4d --> "
					"%4s PORT %d VLAN %4d & %4s PORT %d VLAN %4d\n",
					flow->flow_no,
					port_type[flow->src_port.port_type],
					flow->src_port.port_no,
					flow->src_port.vlan_id,
					port_type[flow->fw_port[0].port_type],
					flow->fw_port[0].port_no,
					flow->fw_port[0].vlan_id,
					port_type[flow->fw_port[1].port_type],
					flow->fw_port[1].port_no,
					flow->fw_port[1].vlan_id);
			} else {
				FM10K_SW_INFO("  FLOW %d : %4s PORT %d VLAN %4d --> "
					"%4s PORT %d VLAN %4d\n",
					flow->flow_no,
					port_type[flow->src_port.port_type],
					flow->src_port.port_no,
					flow->src_port.vlan_id,
					port_type[flow->fw_port[0].port_type],
					flow->fw_port[0].port_no,
					flow->fw_port[0].vlan_id);
			}
			flow = flow->next;
		}
		flowset = flowset->next;
	}
}

static void
fm10k_config_cfg_describe(struct fm10k_switch *sw,
		struct fm10k_device_info *info)
{
	uint16_t i;
	struct fm10k_cfg_flowset *flowset;

	if (!fm10k_config_check_debug(sw->dpdk_cfg, FM10K_CONFIG_DEBUG_CONFIG))
		return;

	FM10K_SW_INFO("--- FM10K STATIC CONFIG ---\n");
	FM10K_SW_INFO("  Card Type: %s\n", info->desc);
	FM10K_SW_INFO("  PF Max   : %d\n", fm10k_config_dpdk_cfg.pf_max);
	FM10K_SW_INFO("  PF Bind  : %d\n", fm10k_config_dpdk_cfg.pf_num);
	FM10K_SW_INFO("  DEBUG    : %#x\n", fm10k_config_dpdk_cfg.debug_cfg);
	FM10K_SW_INFO("  STATS GAP: %d sec\n",
			fm10k_config_dpdk_cfg.stats_interval);
	FM10K_SW_INFO("  EXT PORT speed: %d Gbps\n",
			fm10k_config_dpdk_cfg.ext_port_speed);
	FM10K_SW_INFO("  FLOWSET ENABLE: %s\n",
			fm10k_config_dpdk_cfg.current->name);

	for (i = 0; i < fm10k_config_dpdk_cfg.ext_port_num; i++) {
		if (fm10k_config_dpdk_cfg.ext_port_map[i].type ==
				FM10K_CONFIG_PORT_MAP_NULL)
			continue;
		if (fm10k_config_dpdk_cfg.ext_port_map[i].type ==
				FM10K_CONFIG_PORT_MAP_PF)
			FM10K_SW_INFO("  EXT PORT[%d] MAP: PF%d\n", i + 1,
			fm10k_config_dpdk_cfg.ext_port_map[i].map_no[0]);
		else
			FM10K_SW_INFO("  EXT PORT[%d] MAP: PF%d PF%d\n", i + 1,
			fm10k_config_dpdk_cfg.ext_port_map[i].map_no[0],
			fm10k_config_dpdk_cfg.ext_port_map[i].map_no[1]);
	}

	for (i = 0; i < FM10K_SW_LOGICAL_PORTS_MAX; i++) {
		if (fm10k_config_dpdk_cfg.dpdk_port_map[i].type ==
				FM10K_CONFIG_PORT_MAP_NULL)
			continue;
		if (fm10k_config_dpdk_cfg.ext_port_map[i].type ==
				FM10K_CONFIG_PORT_MAP_PF)
			FM10K_SW_INFO("  DPDK PORT[%d] MAP: PF%d\n", i,
			fm10k_config_dpdk_cfg.dpdk_port_map[i].map_no[0]);
		else
			FM10K_SW_INFO("  DPDK PORT[%d] MAP: PF%d PF%d\n", i,
			fm10k_config_dpdk_cfg.dpdk_port_map[i].map_no[0],
			fm10k_config_dpdk_cfg.dpdk_port_map[i].map_no[1]);
	}

	flowset = fm10k_config_dpdk_cfg.flowset_head.next;
	while (flowset) {
		FM10K_SW_INFO("\n  FLOWSET  : %s\n", flowset->name);
		struct fm10k_cfg_flow *flow = flowset->flow_head.next;
		while (!fm10k_config_flow_list_end(&flowset->flow_head, flow)) {
			const char *port_type[3] = { "NON", "EXT", "DPDK" };
			if (flow->fw_port[1].port_type !=
					FM10K_CONFIG_FLOW_NONE_PORT) {
				FM10K_SW_INFO("  FLOW %d : %4s PORT %d VLAN %4d"
					" --> %4s PORT %d VLAN %4d & %4s PORT %d"
					" VLAN %4d\n",
					flow->flow_no,
					port_type[flow->src_port.port_type],
					flow->src_port.port_no,
					flow->src_port.vlan_id,
					port_type[flow->fw_port[0].port_type],
					flow->fw_port[0].port_no,
					flow->fw_port[0].vlan_id,
					port_type[flow->fw_port[1].port_type],
					flow->fw_port[1].port_no,
					flow->fw_port[1].vlan_id);
			} else {
				FM10K_SW_INFO("  FLOW %d : %4s PORT %d VLAN %4d"
					" --> %4s PORT %d VLAN %4d\n",
					flow->flow_no,
					port_type[flow->src_port.port_type],
					flow->src_port.port_no,
					flow->src_port.vlan_id,
					port_type[flow->fw_port[0].port_type],
					flow->fw_port[0].port_no,
					flow->fw_port[0].vlan_id);
			}
			flow = flow->next;
		}
		flowset = flowset->next;
	}
	FM10K_SW_INFO("\n");
}

int
fm10k_config_init(struct fm10k_switch *sw, struct fm10k_hw *hw)
{
	struct fm10k_device_info *info = fm10k_get_device_info(hw);

	fm10k_config_dpdk_cfg.stats_interval = 2;
	fm10k_config_dpdk_cfg.pf_max = info->num_peps;
	fm10k_config_dpdk_cfg.ext_port_num = info->num_ext_ports;

	if (!fm10k_config_conf_file_exist()) {
		if (info->num_epls == 2 && info->num_peps == 2)
			fm10k_config_dpdk_cfg.config_list =
					fm10k_silc_nic_2ext_2pep;
		else if (info->num_epls == 2 && info->num_peps == 4)
			fm10k_config_dpdk_cfg.config_list =
					fm10k_silc_nic_2ext_4pep;
		else
			return -1;

		fm10k_config_conf_file_create();
	} else {
		if (fm10k_config_conf_file_load() < 0)
			return -1;
	}

	if (fm10k_config_conf_file_transfer() < 0)
		return -1;
	sw->dpdk_cfg = &fm10k_config_dpdk_cfg;

	fm10k_config_cfg_describe(sw, info);

	return 0;
}
