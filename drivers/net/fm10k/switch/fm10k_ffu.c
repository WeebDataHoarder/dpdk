/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright 2019   Silicom Ltd. Connectivity Solutions
 */

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
#include "fm10k_ffu.h"
#include "fm10k_stats.h"

/*
 * one SLICE for 40bits SEL
 * SLICE 28   FOR SEL SGLORT(16bits) and VLAN(16bits)
 *            FOR ACT ROUTE
 * SLICE 29   FOR SEL ETHER_TYPE(16bits)
 *            FOR ACT MIRROR | SET_VLAN
 * SLICE 30   FOR SEL MPLS_LABEL0(32bits)
 * SLICE 31   FOR SEL MPLS_LABEL1(32bits)
 */
#define FM10K_FFU_SLICE_START			28
#define FM10K_FFU_SLICE_SGLORT_VID		28
#define FM10K_FFU_SLICE_ETYPE_VID2		29
#define FM10K_FFU_SLICE_MPLS_LABEL0		30
#define FM10K_FFU_SLICE_MPLS_LABEL1		31

#define FM10K_FFU_SLICE_SGLORT_VLAN_CFG		\
		(FM10K_SW_MAKE_REG_FIELD64(FFU_SLICE_CFG_SELECT_0, \
				FM10K_SW_FFU_MUX_SEL_SGLORT) | \
		FM10K_SW_MAKE_REG_FIELD64(FFU_SLICE_CFG_SELECT_1, \
				FM10K_SW_FFU_MUX_SEL_SGLORT) | \
		FM10K_SW_MAKE_REG_FIELD64(FFU_SLICE_CFG_SELECT_2, \
				FM10K_SW_FFU_MUX_SEL_VPRI_VID) | \
		FM10K_SW_MAKE_REG_FIELD64(FFU_SLICE_CFG_SELECT_3, \
				FM10K_SW_FFU_MUX_SEL_VPRI_VID) | \
		FM10K_SW_FFU_SLICE_CFG_START_ACTION | \
		FM10K_SW_FFU_SLICE_CFG_START_COMPARE | \
		FM10K_SW_FFU_SLICE_CFG_VALID_LOW)

#define FM10K_FFU_SLICE_ETHER_TYPE_CFG		\
		(FM10K_SW_MAKE_REG_FIELD64(FFU_SLICE_CFG_SELECT_0, \
				FM10K_SW_FFU_MUX_SEL_L2_TYPE) | \
		FM10K_SW_MAKE_REG_FIELD64(FFU_SLICE_CFG_SELECT_1, \
				FM10K_SW_FFU_MUX_SEL_L2_TYPE) | \
		FM10K_SW_MAKE_REG_FIELD64(FFU_SLICE_CFG_SELECT_2, \
				FM10K_SW_FFU_MUX_SEL_VPRI2_VID2) | \
		FM10K_SW_MAKE_REG_FIELD64(FFU_SLICE_CFG_SELECT_3, \
				FM10K_SW_FFU_MUX_SEL_VPRI2_VID2) | \
		FM10K_SW_FFU_SLICE_CFG_VALID_LOW)

#define FM10K_FFU_SLICE_MPLS_LABEL0_CFG		\
		(FM10K_SW_MAKE_REG_FIELD64(FFU_SLICE_CFG_SELECT_0, \
				FM10K_SW_FFU_MUX_SEL_L3_SIP_127_96) | \
		FM10K_SW_MAKE_REG_FIELD64(FFU_SLICE_CFG_SELECT_1, \
				FM10K_SW_FFU_MUX_SEL_L3_SIP_127_96) | \
		FM10K_SW_MAKE_REG_FIELD64(FFU_SLICE_CFG_SELECT_2, \
				FM10K_SW_FFU_MUX_SEL_L3_SIP_127_96) | \
		FM10K_SW_MAKE_REG_FIELD64(FFU_SLICE_CFG_SELECT_3, \
				FM10K_SW_FFU_MUX_SEL_L3_SIP_127_96) | \
		FM10K_SW_FFU_SLICE_CFG_START_COMPARE | \
		FM10K_SW_FFU_SLICE_CFG_VALID_LOW)

#define FM10K_FFU_SLICE_MPLS_LABEL1_CFG		\
		(FM10K_SW_MAKE_REG_FIELD64(FFU_SLICE_CFG_SELECT_0, \
				FM10K_SW_FFU_MUX_SEL_L3_SIP_95_64) | \
		FM10K_SW_MAKE_REG_FIELD64(FFU_SLICE_CFG_SELECT_1, \
				FM10K_SW_FFU_MUX_SEL_L3_SIP_95_64) | \
		FM10K_SW_MAKE_REG_FIELD64(FFU_SLICE_CFG_SELECT_2, \
				FM10K_SW_FFU_MUX_SEL_L3_SIP_95_64) | \
		FM10K_SW_MAKE_REG_FIELD64(FFU_SLICE_CFG_SELECT_3, \
				FM10K_SW_FFU_MUX_SEL_L3_SIP_95_64) | \
		FM10K_SW_FFU_SLICE_CFG_START_COMPARE | \
		FM10K_SW_FFU_SLICE_CFG_VALID_LOW)

static uint64_t fm10k_ffu_slice_cfgs[4] = {
		FM10K_FFU_SLICE_SGLORT_VLAN_CFG,
		FM10K_FFU_SLICE_ETHER_TYPE_CFG,
		FM10K_FFU_SLICE_MPLS_LABEL0_CFG,
		FM10K_FFU_SLICE_MPLS_LABEL1_CFG
};

#define FM10K_FFU_INIT_PRINT(cfg, ...)		\
do { \
	if (fm10k_config_check_debug(cfg, FM10K_CONFIG_DEBUG_FFU_INIT)) \
		FM10K_SW_INFO(__VA_ARGS__); \
} while (0)

#define FM10K_FFU_RULE_PRINT(cfg, ...)		\
do { \
	if (fm10k_config_check_debug(cfg, FM10K_CONFIG_DEBUG_FFU_RULE)) \
		FM10K_SW_INFO(__VA_ARGS__); \
} while (0)

#define FM10K_FFU_CNT_INDEX(x)		(FM10K_SW_FFU_CNT_START + (x))
#define FM10K_FFU_FLOW_START		10
#define FM10K_FFU_MIRROR_PROFILE	1
#define FM10K_FFU_SGLORT_TYPE_NULL	0
#define FM10K_FFU_SGLORT_TYPE_EPL	1
#define FM10K_FFU_SGLORT_TYPE_PF	2
#define FM10K_FFU_SGLORT_TYPE_PFS	3
#define FM10K_FFU_SGLORT_TYPE_DPDK	4

struct fm10k_ffu_rule_data {
	uint16_t sglort_type;
	uint16_t cond_sglort;
	uint16_t cond_vlan;
	uint16_t act_dglort;
	uint16_t act_vlan;
	uint16_t bypass_dglort;
	uint16_t bypass_vlan;
};

static uint8_t fm10k_ffu_bitmap[FM10K_FFU_RULE_MAX / 8];
static uint8_t
fm10k_ffu_rule_get_bit(int id)
{
	int num = id / 8;
	int offset = id % 8;

	return (fm10k_ffu_bitmap[num] >> offset) & 0x1;
}

static void
fm10k_ffu_rule_set_bit(int id, uint8_t bit)
{
	int num = id / 8;
	int offset = id % 8;
	uint8_t tmp = fm10k_ffu_bitmap[num];
	uint8_t data = 1;

	if (bit == 0)
		fm10k_ffu_bitmap[num] = tmp & ~(data << offset);
	else
		fm10k_ffu_bitmap[num] |= (data << offset);
}

static int
fm10k_ffu_rule_alloc(void)
{
	int i;

	for (i = FM10K_FFU_FLOW_START; i < FM10K_FFU_RULE_MAX; i++) {
		if (fm10k_ffu_rule_get_bit(i) != 0)
			continue;
		fm10k_ffu_rule_set_bit(i, 1);
		return i;
	}
	return -1;
}

static void
fm10k_ffu_rule_free(int id)
{
	fm10k_ffu_rule_set_bit(id, 0);
}

static void
fm10k_ffu_always_mismatch(struct fm10k_switch *sw,
		unsigned int slice, unsigned int idx)
{
	uint64_t temp64;

	temp64 =
	    FM10K_SW_MAKE_REG_FIELD64(FFU_SLICE_TCAM_KEY, ~0ULL) |
	    FM10K_SW_MAKE_REG_FIELD64(FFU_SLICE_TCAM_KEY_TOP, ~0ULL);
	fm10k_write_switch_reg128(sw, FM10K_SW_FFU_SLICE_TCAM(slice, idx),
	    temp64, temp64);
}

static void
fm10k_ffu_route_dglort(struct fm10k_switch *sw, unsigned int slice,
			unsigned int idx, uint16_t sglort, uint16_t dglort)
{
	uint64_t temp64, temp64_2;

	/*
	 * Set the key to exact match on the 16 SGLORT bits and
	 * always match everywhere else.
	 */
	temp64 = FM10K_SW_MAKE_REG_FIELD64(FFU_SLICE_TCAM_KEY, sglort);
	temp64_2 =
		FM10K_SW_MAKE_REG_FIELD64(FFU_SLICE_TCAM_KEY, ~sglort & 0xffff);
	fm10k_write_switch_reg128(sw,
			FM10K_SW_FFU_SLICE_TCAM(slice, idx),
			temp64_2, temp64);

	/*
	 * Set the corresponding SRAM entry to ROUTE_GLORT to the
	 * corresponding DGLORT.
	 */
	temp64 = 0x40;
	temp64 = temp64 << 32 |
	    FM10K_SW_MAKE_REG_FIELD(FFU_SLICE_SRAM_ROUTE_GLORT_DGLORT, dglort);
	temp64 |=
		FM10K_SW_MAKE_REG_FIELD64(FFU_SLICE_SRAM_COMMAND,
	    FM10K_SW_FFU_SLICE_SRAM_COMMAND_ROUTE_GLORT);
	/*
	 * 11.5.4.2 FFU_SLICE_SRAM[0..31][0..1023]
	 */
	temp64_2 =
		FM10K_SW_FFU_CNT_BANK << (35 - 23) |
		(FM10K_SW_FFU_CNT_START + idx);
	temp64 |= temp64_2 << 23;
	fm10k_write_switch_reg64(sw,
			FM10K_SW_FFU_SLICE_SRAM(slice, idx), temp64);
}

static void
fm10k_ffu_set_dest_glort_mask(struct fm10k_switch *sw,
		unsigned int idx, uint16_t dglort, uint64_t dport_mask)
{
	uint64_t temp64;
	unsigned int multiple_dports;
	unsigned int num_dports;
	unsigned int amplification_factor;
	unsigned int hashed_entries;
	unsigned int i, j;

	multiple_dports = (dport_mask & (dport_mask - 1));

	FM10K_FFU_INIT_PRINT(sw->dpdk_cfg,
			"%s set glort %#x to port ", __func__, dglort);
	/*
	 * Create an exact-match key for the given DGLORT in the DGLORT CAM.
	 */
	fm10k_write_switch_reg(sw, FM10K_SW_GLORT_CAM(idx),
	    FM10K_SW_MAKE_REG_FIELD(GLORT_CAM_KEY, dglort) |
	    FM10K_SW_MAKE_REG_FIELD(GLORT_CAM_KEY_INVERT, ~dglort));

	if (multiple_dports) {
		/*
		 * Create a pair of entries and use the hash value to select
		 * among them.
		 */
		num_dports = 0;
		for (i = 0; i < FM10K_SW_LOGICAL_PORTS_MAX; i++)
			if (dport_mask & (1ULL << i)) {
				num_dports++;
				FM10K_FFU_INIT_PRINT(sw->dpdk_cfg, " %d", i);
			}
		FM10K_FFU_INIT_PRINT(sw->dpdk_cfg, "\n");

		/*
		 * Create multiple entries for each dport to increase the
		 * hash modulus and capture more hash entropy.  The maximum
		 * number of hashed entries is 16.
		 */
		amplification_factor = 16 / num_dports;
		hashed_entries = num_dports * amplification_factor;
		temp64 =
			FM10K_SW_MAKE_REG_FIELD64(GLORT_RAM_STRICT,
		    FM10K_SW_GLORT_RAM_STRICT_HASHED);
		temp64 |=
			FM10K_SW_MAKE_REG_FIELD64(GLORT_RAM_DEST_INDEX,
			sw->glort_dest_table_idx);
		temp64 |=
			FM10K_SW_MAKE_REG_FIELD64(GLORT_RAM_DEST_COUNT,
			hashed_entries);
		fm10k_write_switch_reg64(sw, FM10K_SW_GLORT_RAM(idx), temp64);

		for (i = 0; i < FM10K_SW_LOGICAL_PORTS_MAX; i++)
			if (dport_mask & (1ULL << i))
				for (j = 0; j < amplification_factor; j++) {
					fm10k_write_switch_reg64(sw,
					    FM10K_SW_GLORT_DEST_TABLE
						(sw->glort_dest_table_idx),
					    FM10K_SW_MAKE_REG_FIELD64
						(GLORT_DEST_TABLE_MASK,
						(1ULL << i)));
					sw->glort_dest_table_idx++;
				}
	} else {
		/*
		 * Set the corresponding entry in the DGLORT map RAM to use
		 * strict indexing straight into the DEST_TABLE, then write
		 * the corresponding destination port in the DEST_TABLE.
		 */

		temp64 =
			FM10K_SW_MAKE_REG_FIELD64(GLORT_RAM_STRICT,
		    FM10K_SW_GLORT_RAM_STRICT_STRICT);
		temp64 |=
			FM10K_SW_MAKE_REG_FIELD64(GLORT_RAM_DEST_INDEX,
			sw->glort_dest_table_idx);
		fm10k_write_switch_reg64(sw,
			FM10K_SW_GLORT_RAM(idx), temp64);

		fm10k_write_switch_reg64(sw,
			FM10K_SW_GLORT_DEST_TABLE(sw->glort_dest_table_idx),
		    FM10K_SW_MAKE_REG_FIELD64
			(GLORT_DEST_TABLE_MASK,
			dport_mask));
		sw->glort_dest_table_idx++;
		for (i = 0; i < FM10K_SW_LOGICAL_PORTS_MAX; i++)
			if (dport_mask & (1ULL << i))
				FM10K_FFU_INIT_PRINT(sw->dpdk_cfg, " %d\n", i);
	}
}

static struct fm10k_multi_glort {
	uint8_t lport1;
	uint8_t lport2;
	uint16_t vlan1;
	uint16_t vlan2;
} fm10k_multi_glorts[FM10K_SW_FFU_RULE_MAX];

static uint32_t
fm10k_ffu_multi_glort_get(uint8_t lport1, uint8_t lport2,
		uint16_t vlan1, uint16_t vlan2, bool *p_new)
{
	int i;

	for (i = 0; i < FM10K_SW_FFU_RULE_MAX; i++) {
		if (lport1 == fm10k_multi_glorts[i].lport1 &&
		   lport2 == fm10k_multi_glorts[i].lport2 &&
		   vlan1 == fm10k_multi_glorts[i].vlan1 &&
		   vlan2 == fm10k_multi_glorts[i].vlan2) {
			if (p_new != NULL)
				*p_new = false;
			return FM10K_SW_MULTI_GLORT_START + i;
		}
	}

	for (i = 0; i < FM10K_SW_FFU_RULE_MAX; i++) {
		if (fm10k_multi_glorts[i].lport1 == 0 &&
		   fm10k_multi_glorts[i].lport2 == 0 &&
		   fm10k_multi_glorts[i].vlan1 == 0 &&
		   fm10k_multi_glorts[i].vlan2 == 0) {
			fm10k_multi_glorts[i].lport1 = lport1;
			fm10k_multi_glorts[i].lport2 = lport2;
			fm10k_multi_glorts[i].vlan1 = vlan1;
			fm10k_multi_glorts[i].vlan2 = vlan2;
			if (p_new != NULL)
				*p_new = true;
			return FM10K_SW_MULTI_GLORT_START + i;
		}
	}

	return 0;
}

static uint32_t
fm10k_ffu_set_dest_glort_multi_cast(struct fm10k_switch *sw,
		uint8_t lport1, uint8_t lport2,
		uint16_t vlan1, uint16_t vlan2)
{
	uint32_t dglort;
	bool is_new = false;
	uint32_t idx;
	uint64_t temp64;

	dglort =
		fm10k_ffu_multi_glort_get(lport1, lport2,
				vlan1, vlan2, &is_new);

	if (!is_new)
		return dglort;

	FM10K_FFU_INIT_PRINT(sw->dpdk_cfg,
			"%s set glort %#x to (%d:%d) (%d:%d)\n",
			__func__, dglort, lport1, vlan1, lport2, vlan2);
	/*
	 * Create an exact-match key for the given DGLORT in the DGLORT CAM.
	 */
	idx = sw->glort_cam_ram_idx++;
	fm10k_write_switch_reg(sw, FM10K_SW_GLORT_CAM(idx),
	    FM10K_SW_MAKE_REG_FIELD(GLORT_CAM_KEY, dglort) |
	    FM10K_SW_MAKE_REG_FIELD(GLORT_CAM_KEY_INVERT, ~dglort));

	/*
	 * Create multiple entries for each dport to increase the
	 * hash modulus and capture more hash entropy.  The maximum
	 * number of hashed entries is 16.
	 */
	temp64 =
		FM10K_SW_MAKE_REG_FIELD64(GLORT_RAM_STRICT,
	    FM10K_SW_GLORT_RAM_STRICT_STRICT);
	temp64 |=
		FM10K_SW_MAKE_REG_FIELD64(GLORT_RAM_DEST_INDEX,
		sw->glort_dest_table_idx);
	fm10k_write_switch_reg64(sw,
			FM10K_SW_GLORT_RAM(idx), temp64);

	/* GLORT_DEST_TABLE
	 * Field Name		Bit(s)	Type	Default
	 * DestMask		47:0	RW	0x0
	 * IP_MulticastIndex	59:48	RW	0x0
	 * Reserved		63:60	RSV	0x0
	 */
	temp64 = sw->mcast_dest_table_idx;
	temp64 = temp64 << 48 | 1 << lport1 | 1 << lport2;
	fm10k_write_switch_reg64(sw,
			FM10K_SW_GLORT_DEST_TABLE
			(sw->glort_dest_table_idx), temp64);
	sw->glort_dest_table_idx++;

	/* MCAST_DEST_TABLE
	 * Field Name		Bit(s)	Type	Default
	 * PortMask		47:0	RW	0x0
	 * LenTableIdx		61:48	RW	0x0
	 * Reserved		63:62	RSV	00b
	 */
	temp64 = sw->mcast_len_table_idx;
	temp64 = 1 << lport1 | 1 << lport2 | temp64 << 48;
	fm10k_write_switch_reg64(sw,
			FM10K_SW_SCHED_MCAST_DEST_TABLE
			(sw->mcast_dest_table_idx++), temp64);

	/* MCAST_LEN_TABLE
	 * Field Name		Bit(s)	Type	Default
	 * L3_McastIdx		14:0	RW	0x0
	 * L3_Repcnt		26:15	RW	0x0
	 * Reserved		31:27	RSV	0x0
	 */
	temp64 =
		sw->mcast_vlan_table_idx | 1 << 15;
	fm10k_write_switch_reg64(sw,
			FM10K_SW_SCHED_MCAST_LEN_TABLE
			(sw->mcast_len_table_idx++), temp64);

	/* MCAST_VLAN_TABLE
	 * Field Name		Bit(s)	Type	Default
	 * VID			11:0	RW	0x0
	 * DGLORT		27:12	RW	0x0
	 * ReplaceVID		28	RW	0b
	 * ReplaceDGLORT	29	RW	0b
	 * Reserved		31:30	RSV	00b
	 */
	temp64 = vlan1 |
			fm10k_switch_pf_glort_get
			(lport1) << 12 | 1 << 28 | 1 << 29;
	fm10k_write_switch_reg64(sw,
			FM10K_SW_MOD_MCAST_VLAN_TABLE
			(sw->mcast_vlan_table_idx++), temp64);
	temp64 = vlan2 |
			fm10k_switch_pf_glort_get
			(lport2) << 12 | 1 << 28 | 1 << 29;
	fm10k_write_switch_reg64(sw,
			FM10K_SW_MOD_MCAST_VLAN_TABLE
			(sw->mcast_vlan_table_idx++), temp64);

	return dglort;
}

static uint64_t
fm10k_data64_field64_get(uint64_t data, int start, int end)
{
	uint64_t tmp64 = data;

	if (start == end) {
		tmp64 = (data >> start) & 1;
	} else {
		tmp64 = tmp64 << (64 - end);
		tmp64 = tmp64 >> (64 - end + start);
	}
	return tmp64;
}

static uint32_t
fm10k_data64_field32_get(uint64_t data, int start, int end)
{
	uint64_t tmp64 = data;
	uint32_t tmp32;

	if (start == end) {
		tmp32 = (data >> start) & 1;
	} else {
		tmp64 = tmp64 << (64 - end);
		tmp32 = tmp64 >> (64 - end + start);
	}
	return tmp32;
}

static uint32_t
fm10k_data32_field_get(uint32_t data, int start, int end)
{
	uint32_t tmp32 = data;

	if (start == end) {
		tmp32 = (data >> start) & 1;
	} else {
		tmp32 = tmp32 << (64 - end);
		tmp32 = tmp32 >> (64 - end + start);
	}
	return tmp32;
}

static void
fm10k_glort_register_dump(struct fm10k_switch *sw)
{
	uint32_t i;
	uint64_t data64;
	uint32_t data32;
	uint32_t tmp32;

	if (!fm10k_config_check_debug
		(sw->dpdk_cfg, FM10K_CONFIG_DEBUG_FFU_REG))
		return;

	FM10K_SW_INFO("----- GLORT -----\n");

	for (i = 0; i < sw->glort_cam_ram_idx; i++) {
		data32 = fm10k_read_switch_reg(sw, FM10K_SW_GLORT_CAM(i));
		FM10K_SW_INFO("[%02u]GLORT_CAM %#x Key %#x\n", i, data32,
				fm10k_data32_field_get(data32, 0, 15));
		data64 = fm10k_read_switch_reg64(sw, FM10K_SW_GLORT_RAM(i));
		FM10K_SW_INFO("    GLORT_RAM %#llx Strict %u DestIndex %u\n",
				(unsigned long long)data64,
				fm10k_data64_field32_get(data64, 0, 1),
				fm10k_data64_field32_get(data64, 2, 13));
		tmp32 = fm10k_data64_field32_get(data64, 2, 13);
		data64 =
			fm10k_read_switch_reg64(sw,
				FM10K_SW_GLORT_DEST_TABLE(tmp32));
		FM10K_SW_INFO("    GLORT_DEST_TABLE[%u] %#llx "
				"IP_MulticastIndex %u\n",
				tmp32, (unsigned long long)data64,
				fm10k_data64_field32_get(data64, 48, 59));
		tmp32 = fm10k_data64_field32_get(data64, 48, 59);
		if (tmp32 == 0)
			continue;
		data64 = fm10k_read_switch_reg64(sw,
				FM10K_SW_SCHED_MCAST_DEST_TABLE(tmp32));
		FM10K_SW_INFO("    SCHED_MCAST_DEST_TABLE[%u] "
				"%#llx PortMask %#llx"
				" LenTableIdx %u\n", tmp32,
				(unsigned long long)data64,
				(unsigned long long)
				fm10k_data64_field64_get(data64, 0, 47),
				fm10k_data64_field32_get(data64, 48, 61));
		tmp32 = fm10k_data64_field32_get(data64, 48, 61);
		data32 = fm10k_read_switch_reg(sw,
				FM10K_SW_SCHED_MCAST_LEN_TABLE(tmp32));
		FM10K_SW_INFO("    SCHED_MCAST_LEN_TABLE[%u] %#x "
				"L3_McastIdx %u L3_Repcnt %u\n",
				tmp32, data32,
				fm10k_data32_field_get(data32, 0, 14),
				fm10k_data32_field_get(data32, 15, 26));
		tmp32 = fm10k_data32_field_get(data32, 0, 14);
		data32 = fm10k_read_switch_reg(sw,
				FM10K_SW_MOD_MCAST_VLAN_TABLE(tmp32));
		FM10K_SW_INFO("    MOD_MCAST_VLAN_TABLE[%u] %#x VID %u "
				"DGLORT %#x ReplaceVID %u ReplaceDGLORT %u\n",
				tmp32, data32,
				fm10k_data32_field_get(data32, 0, 11),
				fm10k_data32_field_get(data32, 12, 27),
				fm10k_data32_field_get(data32, 28, 28),
				fm10k_data32_field_get(data32, 29, 29));
		data32 = fm10k_read_switch_reg(sw,
				FM10K_SW_MOD_MCAST_VLAN_TABLE(tmp32 + 1));
		FM10K_SW_INFO("    MOD_MCAST_VLAN_TABLE[%u] %#x VID %u "
				"DGLORT %#x ReplaceVID %u ReplaceDGLORT %u\n",
				tmp32 + 1, data32,
				fm10k_data32_field_get(data32, 0, 11),
				fm10k_data32_field_get(data32, 12, 27),
				fm10k_data32_field_get(data32, 28, 28),
				fm10k_data32_field_get(data32, 29, 29));
	}
}

static void
fm10k_ffu_register_dump(struct fm10k_switch *sw)
{
	int i, j;
	uint32_t data[8];
	uint32_t ffu_valid;

	if (!fm10k_config_check_debug(sw->dpdk_cfg, FM10K_CONFIG_DEBUG_FFU_REG))
		return;

	FM10K_SW_INFO("--------------- FFU REGISTERS DUMP -----------------\n");

	fm10k_read_switch_array(sw, FM10K_SW_FFU_MASTER_VALID, data, 2);
	FM10K_SW_INFO("FFU_MASTER_VALID: %#x %#x\n", data[0], data[1]);
	ffu_valid = data[0];

	for (i = 0; i < 32; i++) {
		if ((ffu_valid & (1 << i)) == 0)
			continue;

		FM10K_SW_INFO("------ SLICE%d ------\n", i);
		fm10k_read_switch_array(sw,
				FM10K_SW_FFU_SLICE_VALID(i), data, 2);
		if (data[0] != 0 || data[1] != 0)
			FM10K_SW_INFO("FFU_SLICE_VALID[%d]: %#x %#x\n",
				i, data[0], data[1]);

		fm10k_read_switch_array(sw,
				FM10K_SW_FFU_SLICE_CASCADE_ACTION(i), data, 2);
		if (data[0] != 0 || data[1] != 0)
			FM10K_SW_INFO("FFU_SLICE_CASCADE_ACTION[%d]: %#x %#x\n",
				i, data[0], data[1]);

		for (j = 0; j < 1; j++) {
			fm10k_read_switch_array(sw,
					FM10K_SW_FFU_SLICE_CFG(i, j), data, 2);
			if (data[0] != 0 || data[1] != 0)
				FM10K_SW_INFO("FFU_SLICE_CFG[%d][%d]: %#x %#x\n",
						i, j, data[0], data[1]);
		}
		for (j = 0; j < 32; j++) {
			fm10k_read_switch_array(sw,
					FM10K_SW_FFU_SLICE_TCAM(i, j), data, 4);
			if ((data[0] != 0 || data[1] != 0 ||
					data[2] != 0 || data[3] != 0))
				FM10K_SW_INFO("FFU_SLICE_TCAM[%d][%d]: "
					"%#x %#x %#x %#x\n",
					i, j, data[0], data[1],
					data[2], data[3]);

			fm10k_read_switch_array(sw,
					FM10K_SW_FFU_SLICE_SRAM(i, j), data, 2);
			if (data[0] != 0 || data[1] != 0)
				FM10K_SW_INFO("FFU_SLICE_SRAM[%d][%d]: %#x %#x\n",
					i, j, data[0], data[1]);
		}
	}
}

static void
fm10k_ffu_mirror_set_action(struct fm10k_switch *sw,
		uint16_t ffu_slice, uint16_t table_idx)
{
	uint64_t data64;

	/* blank the next ffu slice */
	fm10k_write_switch_reg128(sw,
		FM10K_SW_FFU_SLICE_TCAM
		(ffu_slice, table_idx), 0xffffffffff, 0x1);
	/* SET FFU RX_MIRROR */
	data64 = 0x60; /* higher than route, not necessary */
	data64 = data64 << 32 | 0x2 << 21 | 1 << (8 + 4) | 1 << 4;
	fm10k_write_switch_reg64(sw,
		FM10K_SW_FFU_SLICE_SRAM(ffu_slice, table_idx), data64);
}

static void
fm10k_ffu_mirror_set_forward(struct fm10k_switch *sw,
		int profile,
		uint16_t vlan,
		uint16_t dest_lport,
		uint32_t dest_sglort)
{
	uint64_t data64;

	/* RX_MIRROR_CFG */
	fm10k_write_switch_reg(sw, 0xd50000 + 0xd, profile);

	/* FH_MIRROR_PROFILE_TABLE */
	fm10k_write_switch_reg(sw,
			0xd50000 + 0x1 * profile + 0x40, dest_lport);

	/* MOD_MIRROR_PROFILE_TABLE don't work */
	data64 = vlan;
	data64 = (dest_sglort & 0xffff) | data64 << 17;
	fm10k_write_switch_reg64(sw,
			0xe80000 + 0x2 * profile + 0x24000, data64);
}

int
fm10k_ffu_mirror_set(struct fm10k_switch *sw,
		uint16_t src_ext_port,
		uint16_t dest_ext_port,
		uint16_t vlan)
{
	uint16_t table_idx;
	uint16_t ffu_slice = FM10K_FFU_SLICE_SGLORT_VID + 1;
	uint16_t mirror_profile = FM10K_FFU_MIRROR_PROFILE;

	table_idx = FM10K_FFU_EXT_PORT_RULE_INGRESS(src_ext_port - 1);
	fm10k_ffu_mirror_set_action(sw, ffu_slice, table_idx);

	fm10k_ffu_mirror_set_forward(sw, mirror_profile,
			vlan, sw->epl_map[dest_ext_port - 1].logical_port,
			sw->epl_map[dest_ext_port - 1].glort);
	return 0;
}

int
fm10k_ffu_mirror_reset(struct fm10k_switch *sw, int src_ext_port)
{
	uint16_t table_idx;
	uint16_t ffu_slice = FM10K_FFU_SLICE_SGLORT_VID + 1;

	table_idx = FM10K_FFU_EXT_PORT_RULE_INGRESS(src_ext_port - 1);
	fm10k_write_switch_reg64(sw,
			FM10K_SW_FFU_SLICE_SRAM(ffu_slice, table_idx), 0);
	return 0;
}

static void
fm10k_ffu_logical_port_vlan_set(struct fm10k_switch *sw,
		uint16_t lport, uint16_t vlan_id)
{
	uint64_t data;

	/* 11.21.3.9 INGRESS_VID_TABLE[0..4095] */
	data = fm10k_read_switch_reg64(sw,
			0xE80000 + 0x2 * vlan_id + 0x20000);
	data |= 1 << lport;
	fm10k_write_switch_reg64(sw,
			0xE80000 + 0x2 * vlan_id + 0x20000, data);

	FM10K_FFU_RULE_PRINT(sw->dpdk_cfg,
			"%s lport:%d, vlan_id:%d, reg:%#llx\n",
			__func__, lport, vlan_id, (unsigned long long)data);
}

static void
fm10k_ffu_glort_port_vlan_set(struct fm10k_switch *sw,
		uint16_t glort, uint16_t vlan_id)
{
	int i;

	for (i = 0; i < FM10K_SW_PEPS_SUPPORTED; i++) {
		if (sw->pep_map[i].glort == glort) {
			fm10k_ffu_logical_port_vlan_set(sw,
					sw->pep_map[i].logical_port, vlan_id);
			break;
		}
	}
	for (i = 0; i < FM10K_SW_EPLS_SUPPORTED; i++) {
		if (sw->epl_map[i].glort == glort) {
			fm10k_ffu_logical_port_vlan_set(sw,
					sw->epl_map[i].logical_port, vlan_id);
			break;
		}
	}
}

static uint16_t
fm10k_ffu_dpdk_port_glort_get(struct fm10k_switch *sw, int port)
{
	int pf;
	uint16_t glort = 0;

	if (sw->dpdk_cfg->dpdk_port_map[port].type ==
			FM10K_CONFIG_PORT_MAP_PFS ||
		sw->dpdk_cfg->dpdk_port_map[port].type ==
			FM10K_CONFIG_PORT_MAP_PFSS) {
		glort = fm10k_switch_pfs_glort_get
				(sw->dpdk_cfg->dpdk_port_map[port].map_no[0],
				sw->dpdk_cfg->dpdk_port_map[port].map_no[1]);
	} else if (sw->dpdk_cfg->dpdk_port_map[port].type ==
			FM10K_CONFIG_PORT_MAP_PF) {
		pf = sw->dpdk_cfg->dpdk_port_map[port].map_no[0];
		glort = fm10k_switch_pf_glort_get(pf);
	}
	return glort;
}

static void
fm10k_ffu_rule_enable_single_cast(struct fm10k_switch *sw,
		int rule_id,
		uint16_t sglort,
		uint16_t svlan,
		uint16_t dglort,
		uint16_t dvlan)
{
	uint64_t temp64;
	uint64_t sglort_vid_tcam = 0, sglort_vid_tcam_mask = 0;
	uint64_t sram[4] = { 0, 0, 0, 0 };
	uint16_t sram_idx = 0, tcam_slice, sram_slice, i;

	sglort_vid_tcam |= sglort;
	sglort_vid_tcam_mask |= 0xffff;
	if (svlan) {
		temp64 = svlan;
		sglort_vid_tcam |= temp64 << 16;
		sglort_vid_tcam_mask |= 0xfff0000;
		fm10k_ffu_glort_port_vlan_set(sw, sglort, svlan);
	}

	/* set counter */
	sram[sram_idx] |=
			FM10K_SW_MAKE_REG_FIELD64
			(FFU_SLICE_SRAM_COUNTER_BANK,
			FM10K_SW_FFU_CNT_BANK) |
			FM10K_SW_MAKE_REG_FIELD64
			(FFU_SLICE_SRAM_COUNTER_INDEX,
			FM10K_FFU_CNT_INDEX(rule_id));

	sram[sram_idx] |=
			FM10K_SW_MAKE_REG_FIELD64
			(FFU_SLICE_SRAM_PRECEDENCE, 3) |
		    FM10K_SW_MAKE_REG_FIELD
			(FFU_SLICE_SRAM_ROUTE_GLORT_DGLORT, dglort) |
			FM10K_SW_MAKE_REG_FIELD64
			(FFU_SLICE_SRAM_COMMAND,
			FM10K_SW_FFU_SLICE_SRAM_COMMAND_ROUTE_GLORT);
	sram_idx++;

	if (dvlan) {
		/* Force updating VLAN tag if present.
		 * Add if absent. 11.5.3.4
		 */
		temp64 = 3;
		sram[sram_idx] =
			temp64 << 16 | dvlan |
			FM10K_SW_MAKE_REG_FIELD64
			(FFU_SLICE_SRAM_PRECEDENCE, 2) |
			FM10K_SW_MAKE_REG_FIELD64
			(FFU_SLICE_SRAM_COMMAND,
			FM10K_SW_FFU_SLICE_SRAM_COMMAND_FIELD_SET);
		fm10k_ffu_glort_port_vlan_set(sw, dglort, dvlan);
		sram_idx++;
	}

	if (sglort_vid_tcam) {
		tcam_slice = FM10K_FFU_SLICE_SGLORT_VID;
		temp64 =
			FM10K_SW_MAKE_REG_FIELD64
			(FFU_SLICE_TCAM_KEY,
			~sglort_vid_tcam & sglort_vid_tcam_mask);
		fm10k_write_switch_reg128(sw,
			FM10K_SW_FFU_SLICE_TCAM(tcam_slice, rule_id),
			temp64, sglort_vid_tcam);
		FM10K_FFU_RULE_PRINT(sw->dpdk_cfg,
			"TCAM slice:%d, rule:%d, data0:%#llx, data1:%#llx\n",
			tcam_slice, rule_id,
			(unsigned long long)sglort_vid_tcam,
			(unsigned long long)temp64);
	}

	/* blank the next SLICE TCAM */
	fm10k_ffu_always_mismatch(sw, tcam_slice + 1, rule_id);

	for (i = 0; i < sram_idx; i++) {
		sram_slice = FM10K_FFU_SLICE_START + i;
		fm10k_write_switch_reg64(sw,
			FM10K_SW_FFU_SLICE_SRAM(sram_slice, rule_id), sram[i]);
		FM10K_FFU_RULE_PRINT(sw->dpdk_cfg,
			"SRAM slice:%d, rule:%d, data:%#llx\n",
			sram_slice, rule_id, (unsigned long long)sram[i]);
	}
	/* disable the next SLICE SRAM */
	fm10k_write_switch_reg64(sw,
			FM10K_SW_FFU_SLICE_SRAM(sram_slice + 1, rule_id), 0);

	fm10k_stats_rule_count_reg(rule_id);
}

static void
fm10k_ffu_rule_enable_multi_cast(struct fm10k_switch *sw,
		int rule_id,
		uint16_t sglort,
		uint16_t svlan,
		uint8_t lport1,
		uint8_t lport2,
		uint16_t vlan1,
		uint16_t vlan2)
{
	uint64_t temp64;
	uint32_t dglort;
	uint64_t sglort_vid_tcam = 0, sglort_vid_tcam_mask = 0;
	uint64_t sram[4] = { 0, 0, 0, 0 };
	uint16_t sram_idx = 0, tcam_slice, sram_slice, i;

	dglort =
		fm10k_ffu_set_dest_glort_multi_cast(sw,
				lport1, lport2, vlan1, vlan2);

	sglort_vid_tcam |= sglort;
	sglort_vid_tcam_mask |= 0xffff;
	if (svlan) {
		temp64 = svlan;
		sglort_vid_tcam |= temp64 << 16;
		sglort_vid_tcam_mask |= 0xfff0000;
		fm10k_ffu_glort_port_vlan_set(sw, sglort, svlan);
	}

	fm10k_ffu_logical_port_vlan_set(sw, lport1, vlan1);
	fm10k_ffu_logical_port_vlan_set(sw, lport2, vlan2);

	/* set counter */
	sram[sram_idx] |=
			FM10K_SW_MAKE_REG_FIELD64
			(FFU_SLICE_SRAM_COUNTER_BANK,
			FM10K_SW_FFU_CNT_BANK) |
			FM10K_SW_MAKE_REG_FIELD64
			(FFU_SLICE_SRAM_COUNTER_INDEX,
			FM10K_FFU_CNT_INDEX(rule_id));

	sram[sram_idx] |=
			FM10K_SW_MAKE_REG_FIELD64
			(FFU_SLICE_SRAM_PRECEDENCE, 3) |
		    FM10K_SW_MAKE_REG_FIELD
			(FFU_SLICE_SRAM_ROUTE_GLORT_DGLORT, dglort) |
			FM10K_SW_MAKE_REG_FIELD64
			(FFU_SLICE_SRAM_COMMAND,
			FM10K_SW_FFU_SLICE_SRAM_COMMAND_ROUTE_GLORT);
	sram_idx++;

	if (sglort_vid_tcam) {
		tcam_slice = FM10K_FFU_SLICE_SGLORT_VID;
		temp64 =
			FM10K_SW_MAKE_REG_FIELD64
			(FFU_SLICE_TCAM_KEY,
			~sglort_vid_tcam & sglort_vid_tcam_mask);
		fm10k_write_switch_reg128(sw,
			FM10K_SW_FFU_SLICE_TCAM(tcam_slice, rule_id),
			temp64, sglort_vid_tcam);
		FM10K_FFU_RULE_PRINT(sw->dpdk_cfg,
			"TCAM slice:%d, rule:%d, data0:%#llx, data1:%#llx\n",
			tcam_slice, rule_id,
			(unsigned long long)sglort_vid_tcam,
			(unsigned long long)temp64);
	}

	/* blank the next SLICE TCAM */
	fm10k_ffu_always_mismatch(sw, tcam_slice + 1, rule_id);

	for (i = 0; i < sram_idx; i++) {
		sram_slice = FM10K_FFU_SLICE_START + i;
		fm10k_write_switch_reg64(sw,
			FM10K_SW_FFU_SLICE_SRAM(sram_slice, rule_id), sram[i]);
		FM10K_FFU_RULE_PRINT(sw->dpdk_cfg,
			"SRAM slice:%d, rule:%d, data:%#llx\n",
			sram_slice, rule_id, (unsigned long long)sram[i]);
	}
	/* disable the next SLICE SRAM */
	fm10k_write_switch_reg64(sw,
			FM10K_SW_FFU_SLICE_SRAM(sram_slice + 1, rule_id), 0);

	fm10k_stats_rule_count_reg(rule_id);
}

void
fm10k_ffu_flow_enable(struct fm10k_switch *sw, struct fm10k_cfg_flow *flow)
{
	uint16_t sglort = 0;
	uint16_t svlan = 0;
	uint16_t dglort = 0;
	uint16_t dvlan = 0;
	uint16_t lport1 = 0, lport2 = 0;
	uint16_t dvlan2 = 0;

	switch (flow->src_port.port_type) {
	case FM10K_CONFIG_FLOW_EXT_PORT:
		sglort = fm10k_switch_epl_glort_get(flow->src_port.port_no - 1);
		break;
	case FM10K_CONFIG_FLOW_DPDK_PORT:
		sglort =
			fm10k_ffu_dpdk_port_glort_get(sw,
			flow->src_port.port_no);
		break;
	}
	switch (flow->fw_port[0].port_type) {
	case FM10K_CONFIG_FLOW_EXT_PORT:
		dglort =
			fm10k_switch_epl_glort_get
			(flow->fw_port[0].port_no - 1);
		lport1 =
			fm10k_switch_epl_logical_get
			(flow->fw_port[0].port_no - 1);
		break;
	case FM10K_CONFIG_FLOW_DPDK_PORT:
		dglort =
			fm10k_ffu_dpdk_port_glort_get
			(sw, flow->fw_port[0].port_no);
		lport1 =
			fm10k_switch_pf_logical_get
			(flow->fw_port[0].port_no);
		break;
	}
	svlan = flow->src_port.vlan_id;
	dvlan = flow->fw_port[0].vlan_id;

	flow->rule_id = fm10k_ffu_rule_alloc();

	FM10K_FFU_RULE_PRINT(sw->dpdk_cfg,
		"Set rule cond sglort:%#x, vlan:%d ==> "
		"act dglort:%#x, vlan:%d",
		sglort, svlan, dglort, dvlan);

	if (flow->fw_port[1].port_type)	{
		switch (flow->fw_port[1].port_type)	{
		case FM10K_CONFIG_FLOW_EXT_PORT:
			dglort =
				fm10k_switch_epl_glort_get
				(flow->fw_port[1].port_no - 1);
			lport2 =
				fm10k_switch_epl_logical_get
				(flow->fw_port[1].port_no - 1);
			break;
		case FM10K_CONFIG_FLOW_DPDK_PORT:
			dglort =
				fm10k_ffu_dpdk_port_glort_get
				(sw, flow->fw_port[1].port_no);
			lport2 =
				fm10k_switch_pf_logical_get
				(flow->fw_port[1].port_no);
			break;
		}
		dvlan2 = flow->fw_port[1].vlan_id;
		FM10K_FFU_RULE_PRINT(sw->dpdk_cfg,
			" (bypass glort:%#x, vlan:%d)\n",
			dglort, dvlan2);

		fm10k_ffu_rule_enable_multi_cast(sw,
			flow->rule_id, sglort, svlan,
			lport1, lport2, dvlan, dvlan2);
	} else {
		FM10K_FFU_RULE_PRINT(sw->dpdk_cfg, "\n");
		fm10k_ffu_rule_enable_single_cast(sw,
			flow->rule_id, sglort, svlan, dglort, dvlan);
	}
}

void
fm10k_ffu_flow_disable(struct fm10k_switch *sw, struct fm10k_cfg_flow *flow)
{
	int i;

	if (flow->rule_id == 0)
		return;

	FM10K_FFU_RULE_PRINT(sw->dpdk_cfg,
			"Remove flow %d rule %d\n",
			flow->flow_no, flow->rule_id);

	for (i = FM10K_FFU_SLICE_START; i < FM10K_SW_FFU_NUM_SLICES; i++) {
		fm10k_ffu_always_mismatch(sw, i, flow->rule_id);
		fm10k_write_switch_reg64(sw,
				FM10K_SW_FFU_SLICE_SRAM(i, flow->rule_id), 0);
	}
	fm10k_ffu_rule_free(flow->rule_id);
}

static int
fm10k_ffu_configured_flowset_enable(struct fm10k_switch *sw)
{
	struct fm10k_cfg_flowset *flowset;
	struct fm10k_cfg_flow *flow;

	flowset = fm10k_config_flowset_current_get();
	flow = flowset->flow_head.next;
	while (!fm10k_config_flow_list_end(&flowset->flow_head, flow)) {
		fm10k_ffu_flow_enable(sw, flow);
		flow = flow->next;
	}

	return 0;
}

void
fm10k_ffu_flowset_switch(struct fm10k_switch *sw, const char *new_name)
{
	struct fm10k_cfg_flowset *flowset;
	struct fm10k_cfg_flowset *cur_fs;
	struct fm10k_cfg_flow *flow;

	cur_fs = fm10k_config_flowset_current_get();
	if (strcmp(cur_fs->name, new_name) == 0)
		return;

	flowset = fm10k_config_flowset_get(new_name);
	if (flowset == NULL) {
		FM10K_SW_ERR("Can not find flowset %s!!\n", new_name);
		return;
	}

	/* disable current flowset */
	flow = cur_fs->flow_head.next;
	while (!fm10k_config_flow_list_end(&cur_fs->flow_head, flow)) {
		fm10k_ffu_flow_disable(sw, flow);
		flow = flow->next;
	}

	/* enable new flowset */
	fm10k_config_flowset_current_set(flowset);
	flow = flowset->flow_head.next;
	while (!fm10k_config_flow_list_end(&flowset->flow_head, flow)) {
		fm10k_ffu_flow_enable(sw, flow);
		flow = flow->next;
	}
}

int
fm10k_ffu_init(struct fm10k_switch *sw, struct fm10k_dpdk_cfg *cfg)
{
	int ret = 0;
	uint64_t data64;
	uint16_t i, j;
	uint32_t sglort = 0, dglort = 0;
	uint16_t table_idx = 0;
	uint16_t ffu_slice = FM10K_FFU_SLICE_START;

	sw->mcast_dest_table_idx = 1;
	sw->mcast_len_table_idx = 1;
	sw->mcast_vlan_table_idx = 1;

	for (i = 0; i < sizeof(fm10k_ffu_slice_cfgs) / sizeof(uint64_t); i++) {
		for (j = 0; j < FM10K_SW_FFU_NUM_SCENARIOS; j++) {
			data64 = fm10k_ffu_slice_cfgs[i];
			fm10k_write_switch_reg64(sw,
					FM10K_SW_FFU_SLICE_CFG
					(FM10K_FFU_SLICE_START + i, j),
					data64);
		}
		FM10K_FFU_INIT_PRINT(cfg, "SET slice %d cfg = %#llx\n",
				FM10K_FFU_SLICE_START + i,
				(unsigned long long)data64);
	}

	for (table_idx = 0; table_idx < FM10K_SW_FFU_SLICE_TCAM_ENTRIES / 2;
			table_idx++)
		for (i = ffu_slice; i < FM10K_SW_FFU_NUM_SLICES; i++)
			fm10k_ffu_always_mismatch(sw, i, table_idx);

	/*
	 * Create a TCAM entry to match each SGLORT that might be used, and
	 * set the corresponding SRAM action entries to ROUTE_GLORT to the
	 * corresponding DGLORT (see above table).
	 * SGLORT ROUTE is always the first slice
	 */
	for (i = 0; i < FM10K_SW_EXT_PORTS_MAX; i++) {
		if (cfg->ext_port_map[i].type == FM10K_CONFIG_PORT_MAP_NULL)
			continue;

		if (cfg->ext_port_map[i].type == FM10K_CONFIG_PORT_MAP_PF) {
			sglort = fm10k_switch_epl_glort_get(i);
			dglort =
				fm10k_switch_pf_glort_get
				(cfg->ext_port_map[i].map_no[0]);
		} else if (cfg->ext_port_map[i].type ==
				   FM10K_CONFIG_PORT_MAP_PFS) {
			sglort = fm10k_switch_epl_glort_get(i);
			dglort =
				fm10k_switch_pfs_glort_get
				(cfg->ext_port_map[i].map_no[0],
				cfg->ext_port_map[i].map_no[1]);
		}

		table_idx = FM10K_FFU_EXT_PORT_RULE_INGRESS(i);
		fm10k_ffu_route_dglort(sw,
				ffu_slice, table_idx, sglort, dglort);

		/* blank the next ffu slice */
		fm10k_write_switch_reg128(sw,
				FM10K_SW_FFU_SLICE_TCAM
				(ffu_slice + 1, table_idx),
			    0xffffffffff, 0x1);
		table_idx = FM10K_FFU_EXT_PORT_RULE_EGRESS(i);
		fm10k_ffu_route_dglort(sw,
				ffu_slice, table_idx, dglort, sglort);

		/* blank the next ffu slice */
		fm10k_write_switch_reg128(sw,
				FM10K_SW_FFU_SLICE_TCAM
				(ffu_slice + 1, table_idx),
			    0xffffffffff, 0x1);
	}

	for (i = ffu_slice; i < FM10K_SW_FFU_NUM_SLICES; i++) {
		fm10k_write_switch_reg64(sw,
				FM10K_SW_FFU_SLICE_CASCADE_ACTION(i),
				0xf0f000f);
		/* Set slice 0 to valid for all scenarios */
		fm10k_write_switch_reg64(sw,
				FM10K_SW_FFU_SLICE_VALID(i),
				FM10K_SW_FFU_SLICE_VALID_ALL_SCENARIOS);
	}

	/* Mark slice 0 as valid and all chunks as invalid */
	data64 = 0;
	for (i = ffu_slice; i < FM10K_SW_FFU_NUM_SLICES; i++)
		data64 |= FM10K_SW_FFU_MASTER_VALID_SLICE_VALID(i);
	fm10k_write_switch_reg64(sw, FM10K_SW_FFU_MASTER_VALID, data64);

	/*
	 * Set up the DGLORT map according to the desired
	 * SGLORT -> { DGLORT, logical_port } map.
	 */
	for (i = 0; i < sw->info->num_peps; i++) {
		if (sw->pep_map[i].glort == 0)
			continue;
		fm10k_ffu_set_dest_glort_mask(sw,
				sw->glort_cam_ram_idx++,
				sw->pep_map[i].glort,
				FM10K_SW_DPORT_MASK
				(sw->pep_map[i].logical_port));
	}

	for (i = 0; i < FM10K_SW_EPLS_SUPPORTED; i++) {
		if (sw->epl_map[i].glort == 0)
			continue;
		fm10k_ffu_set_dest_glort_mask(sw,
				sw->glort_cam_ram_idx++,
				sw->epl_map[i].glort,
				FM10K_SW_DPORT_MASK
				(sw->epl_map[i].logical_port));
	}

	for (i = 0; i < FM10K_SW_EXT_PORTS_MAX; i++) {
		uint64_t dport_mask = 0;

		if (cfg->ext_port_map[i].type ==
				FM10K_CONFIG_PORT_MAP_PFS) {
			dglort =
				fm10k_switch_pfs_glort_get
				(cfg->ext_port_map[i].map_no[0],
						cfg->ext_port_map[i].map_no[1]);
			dport_mask =
				FM10K_SW_DPORT_MASK
		(sw->pep_map[cfg->ext_port_map[i].map_no[0]].logical_port) |
				FM10K_SW_DPORT_MASK
		(sw->pep_map[cfg->ext_port_map[i].map_no[1]].logical_port);
			fm10k_ffu_set_dest_glort_mask(sw,
				sw->glort_cam_ram_idx++, dglort, dport_mask);
		}
	}

	/* Ensure the rest of the DGLORT TCAM won't match */
	for (table_idx = sw->glort_cam_ram_idx;
		 table_idx < FM10K_SW_GLORT_CAM_ENTRIES;
		 table_idx++)
		fm10k_write_switch_reg(sw,
				FM10K_SW_GLORT_CAM(table_idx),
				FM10K_SW_GLORT_CAM_MATCH_NONE);

	ret = fm10k_ffu_configured_flowset_enable(sw);

	fm10k_ffu_register_dump(sw);
	fm10k_glort_register_dump(sw);
	return ret;
}
