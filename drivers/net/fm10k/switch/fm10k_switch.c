/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright 2019   Silicom Ltd. Connectivity Solutions
 */

#include <rte_ethdev.h>
#include <rte_ethdev_pci.h>
#include <rte_malloc.h>

#include "../base/fm10k_type.h"
#include "../base/fm10k_osdep.h"
#include "../fm10k.h"
#include "../fm10k_logs.h"
#include "fm10k_debug.h"
#include "fm10k_regs.h"
#include "fm10k_switch.h"
#include "fm10k_ext_port.h"
#include "fm10k_serdes.h"
#include "fm10k_i2c.h"
#include "fm10k_ffu.h"
#include "fm10k_stats.h"
#include "fm10k_config.h"

static struct fm10k_switch fm10k_sw;

#define FM10K_AM_TIMEOUT			16384
#define FM10K_COMP_PPM_SCALE			1000000
#define FM10K_LED_POLL_INTERVAL_MS		500
#define FM10K_LED_BLINKS_PER_SECOND		2

/* Max try times to acquire switch status */
#define FM10K_QUERY_SWITCH_STATE_TIMES		10
/* Wait interval to get switch status */
#define FM10K_WAIT_SWITCH_MSG_US		100000

/*
 * use epl as external port map, only support QUAD_ON
 */
struct fm10k_sched_prog {
	uint8_t idle;
	uint8_t phys;	/* physical port */
	uint8_t log;	/* logical port */
	uint8_t quad;	/* now, only support QUAD_ON */
/* convert entry to idle if EPL in quad port mode */
#define FM10K_SW_QUAD_OFF		0
/* always use quad port mode */
#define FM10K_SW_QUAD_ON		1
/* use quad port mode if link speed is 40/100G */
#define FM10K_SW_QUAD_40_100		2
};

static struct fm10k_sched_prog
	fm10k_sched_prog[FM10K_SW_PEPS_SUPPORTED + FM10K_SW_EPLS_SUPPORTED + 1];

/*
 * Note that for physical port numbers, the initial values used for
 * the EPL entries assume EPL[A] is EPL[0] and EPL[B] is EPL[1].
 * fm10k_switch_determine_epls() will update these physical port
 * numbers based on the actual A and B indices.
 */
static void
fm10k_switch_set_sched_prog(void)
{
	int i;
	int start = 0;

	for (i = 0; i < FM10K_SW_EPLS_SUPPORTED; i++) {
		fm10k_sched_prog[i].idle = 0;
		fm10k_sched_prog[i].phys = fm10k_epl_port_map[i].physical_port;
		fm10k_sched_prog[i].log = fm10k_epl_port_map[i].logical_port;
		fm10k_sched_prog[i].quad = FM10K_SW_QUAD_ON;
	}
	start += FM10K_SW_EPLS_SUPPORTED;
	for (i = 0; i < FM10K_SW_PEPS_SUPPORTED; i++) {
		fm10k_sched_prog[start + i].idle = 0;
		fm10k_sched_prog[start + i].phys =
				fm10k_pep_port_map[i].physical_port;
		fm10k_sched_prog[start + i].log =
				fm10k_pep_port_map[i].logical_port;
		fm10k_sched_prog[start + i].quad = FM10K_SW_QUAD_40_100;
	}
	start += FM10K_SW_PEPS_SUPPORTED;
	fm10k_sched_prog[start].idle = 1;
}

static void
fm10k_switch_determine_epls(struct fm10k_switch *sw)
{
	struct fm10k_device_info *cfg = sw->info;
	unsigned int i;
	uint8_t phys;

	sw->epla_no = 0;
	sw->eplb_no = 6;

	switch (FM10K_SW_CARD_ID(cfg->subvendor, cfg->subdevice)) {
	case FM10K_SW_CARD(SILICOM, PE3100G2DQIR_QXSL4):
	case FM10K_SW_CARD(SILICOM, PE3100G2DQIR_QXSL4_REV_2):
		sw->epla_no = 1;
		sw->eplb_no = 6;
		break;
	case FM10K_SW_CARD(SILICOM, PE3100G2DQIRL_QXSL4):
	case FM10K_SW_CARD(SILICOM, PE3100G2DQIRM_QXSL4):
	case FM10K_SW_CARD(SILICOM, PE325G2DSIR):
		sw->epla_no = 1;
		sw->eplb_no = 7;
		break;
	}

	for (i = 0; i < sizeof(fm10k_sched_prog) /
			sizeof(fm10k_sched_prog[0]); i++) {
		if (fm10k_sched_prog[i].idle)
			continue;

		phys = fm10k_sched_prog[i].phys;
		if (phys <= 3) {
			/* Substitute actual epla phys port number */
			fm10k_sched_prog[i].phys = sw->epla_no * 4 + phys;
		} else if (phys >= 4 && phys <= 7) {
			/* Substitute actual eplb phys port number */
			fm10k_sched_prog[i].phys = sw->eplb_no * 4 + (phys - 4);
		}
	}
}

static void
fm10k_hw_eicr_disable_source(struct fm10k_switch *sw, unsigned int source)
{
	unsigned int shift;

	switch (source) {
	case FM10K_SW_EICR_PCA_FAULT:
		shift = FM10K_SW_EICR_PCA_FAULT_SHIFT;
		break;
	case FM10K_SW_EICR_THI_FAULT:
		shift = FM10K_SW_EICR_THI_FAULT_SHIFT;
		break;
	case FM10K_SW_EICR_FUM_FAULT:
		shift = FM10K_SW_EICR_FUM_FAULT_SHIFT;
		break;
	case FM10K_SW_EICR_MAILBOX:
		shift = FM10K_SW_EICR_MAILBOX_SHIFT;
		break;
	case FM10K_SW_EICR_SWITCH_READY:
		shift = FM10K_SW_EICR_SWITCH_READY_SHIFT;
		break;
	case FM10K_SW_EICR_SWITCH_NREADY:
		shift = FM10K_SW_EICR_SWITCH_NREADY_SHIFT;
		break;
	case FM10K_SW_EICR_SWITCH_INT:
		shift = FM10K_SW_EICR_SWITCH_INT_SHIFT;
		break;
	case FM10K_SW_EICR_SRAM_ERROR:
		shift = FM10K_SW_EICR_SRAM_ERROR_SHIFT;
		break;
	case FM10K_SW_EICR_VFLR:
		shift = FM10K_SW_EICR_VFLR_SHIFT;
		break;
	case FM10K_SW_EICR_MAX_HOLD_TIME:
		shift = FM10K_SW_EICR_MAX_HOLD_TIME_SHIFT;
		break;
	default:
		return;
	}

	fm10k_write_switch_reg(sw,
			FM10K_SW_EIMR, FM10K_SW_EIMR_DISABLE << (shift * 2));
	fm10k_write_flush(sw);
}

unsigned int
fm10k_switch_eplidx_to_eplno(struct fm10k_switch *sw, unsigned int eplidx)
{
	return eplidx ? sw->eplb_no : sw->epla_no;
}

static uint16_t
fm10k_switch_ppm_to_tx_clk_compensation_timeout(uint32_t pcs, uint32_t num_ppm)
{
	unsigned int scale;
	unsigned int ppm;
	unsigned int timeout;
	unsigned int am_ppm;

	if (num_ppm == 0 || pcs == FM10K_SW_EPL_PCS_SEL_DISABLE) {
		if (pcs == FM10K_SW_EPL_PCS_SEL_40GBASER ||
		    pcs == FM10K_SW_EPL_PCS_SEL_100GBASER)
			return (FM10K_AM_TIMEOUT / 2);
		else
			return 0;
	}

	if (pcs == FM10K_SW_EPL_PCS_SEL_40GBASER ||
	    pcs == FM10K_SW_EPL_PCS_SEL_100GBASER) {
		am_ppm = 1000000 / FM10K_AM_TIMEOUT;
		scale = FM10K_COMP_PPM_SCALE / 2;
	} else {
		am_ppm = 0;
		scale = FM10K_COMP_PPM_SCALE;
	}

	ppm = num_ppm + am_ppm;
	timeout = scale / ppm;

	if (timeout >= 0xffff)
		return 0xffff;
	else
		return timeout;
}

static int
fm10k_switch_configure_epls(struct fm10k_hw *hw, struct fm10k_switch *sw)
{
	struct fm10k_ext_ports *ext_ports = sw->ext_ports;
	struct fm10k_ext_port *port;
	struct fm10k_device_info *cfg = sw->info;
	u32 mac_cfg[FM10K_SW_MAC_CFG_ARRAY_SIZE];
	u32 data, pcs, qpl;
	u32 pcstmp;
	unsigned int i, j;
	unsigned int dic_enable;
	unsigned int anti_bubble_wm;
	unsigned int rate_fifo_wm;
	unsigned int rate_fifo_slow_inc, rate_fifo_fast_inc;
	int error;
	u16 timeout;

	cfg = fm10k_get_device_info(hw);
	if (cfg == NULL)
		return -1;

	/*
	 * Assumptions:
	 *   - All external interfaces are the same speed
	 *   - 1G/10G ports are packed into the minimum number of EPLs
	 *   - quad-mode EPLs use lane 0 as master
	 *   - The lowest numbered PEPs are used
	 *   - PEPs are always used in x8 mode.
	 */
	switch (cfg->ext_port_speed) {
	case 10:
		pcs = FM10K_SW_EPL_PCS_SEL_10GBASER;
		qpl = FM10K_SW_EPL_QPL_MODE_L1_L1_L1_L1;
		dic_enable = 1;
		anti_bubble_wm = 5;
		rate_fifo_wm = 3;
		rate_fifo_slow_inc = 12;
		rate_fifo_fast_inc = 13;
		break;
	case 25:
		pcs = FM10K_SW_EPL_PCS_SEL_100GBASER;
		qpl = FM10K_SW_EPL_QPL_MODE_L1_L1_L1_L1;
		dic_enable = 0;
		anti_bubble_wm = 6;
		rate_fifo_wm = 3;
		rate_fifo_slow_inc = 32;
		rate_fifo_fast_inc = 33;
		break;
	case 40:
		pcs = FM10K_SW_EPL_PCS_SEL_40GBASER;
		qpl = FM10K_SW_EPL_QPL_MODE_L4_XX_XX_XX;
		dic_enable = 1;
		anti_bubble_wm = 4;
		rate_fifo_wm = 5;
		rate_fifo_slow_inc = 51;
		rate_fifo_fast_inc = 52;
		break;
	case 100:
		pcs = FM10K_SW_EPL_PCS_SEL_100GBASER;
		qpl = FM10K_SW_EPL_QPL_MODE_L4_XX_XX_XX;
		dic_enable = 1;
		anti_bubble_wm = 4;
		rate_fifo_wm = 3;
		rate_fifo_slow_inc = 129;
		rate_fifo_fast_inc = 130;
		break;
	default:
		error = -1;
		goto done;
	}

	/*
	 * EPL_CFG_A
	 *
	 * Mark all used lanes as active and all unused lanes as
	 * inactive. Adjust timeout and skew tolerance values for EPLs that
	 * are used.
	 */
	for (i = 0; i < FM10K_SW_EPLS_MAX; i++) {
		data = fm10k_read_switch_reg(sw, FM10K_SW_EPL_CFG_A(i));
		data &= ~FM10K_SW_EPL_CFG_A_ACTIVE_QUAD;
		if (FM10K_SW_EXT_PORTS_EPL_USED(ext_ports, i)) {
			for (j = 0; j < ext_ports->num_ports; j++) {
				port = &ext_ports->ports[j];
				if (port->eplno == i)
					data |= port->is_quad ?
					    FM10K_SW_EPL_CFG_A_ACTIVE_QUAD :
					    FM10K_SW_EPL_CFG_A_ACTIVE
						(port->first_lane);
			}
			FM10K_SW_REPLACE_REG_FIELD(data,
					EPL_CFG_A_TIMEOUT, 19, data);
			FM10K_SW_REPLACE_REG_FIELD(data,
					EPL_CFG_A_SKEW_TOLERANCE, 38, data);
		}
		fm10k_write_switch_reg(sw, FM10K_SW_EPL_CFG_A(i), data);
	}

	/*
	 * EPL_CFG_B
	 *
	 * Disable all unused lanes and configure all used ones
	 * appropriately.  For EPLs used in quad port mode, the master lane
	 * is the only one that gets configured.
	 */
	data = 0;
	for (i = 0; i < FM10K_SW_EPLS_MAX; i++) {
		if (FM10K_SW_EXT_PORTS_EPL_USED(ext_ports, i)) {
			for (j = 0; j < FM10K_SW_EPL_LANES; j++) {
				if (j < ext_ports->ports_per_epl)
					pcstmp = pcs;
				else
					pcstmp = FM10K_SW_EPL_PCS_SEL_DISABLE;
				data |= FM10K_SW_MAKE_REG_FIELD_IDX
					(EPL_CFG_B_PCS_SEL, j, pcstmp, j, j);
			}
			data |= FM10K_SW_MAKE_REG_FIELD
				(EPL_CFG_B_QPL_MODE, qpl);
		} else {
			for (j = 0; j < FM10K_SW_EPL_LANES; j++)
				data |= FM10K_SW_MAKE_REG_FIELD_IDX
					(EPL_CFG_B_PCS_SEL,	j,
					FM10K_SW_EPL_PCS_SEL_DISABLE, j, j);
			data |= FM10K_SW_MAKE_REG_FIELD
				(EPL_CFG_B_QPL_MODE,
				FM10K_SW_EPL_QPL_MODE_XX_XX_XX_XX);
		}
		fm10k_write_switch_reg(sw, FM10K_SW_EPL_CFG_B(i), data);
	}

	/*
	 * MAC_CFG, LINK_RULES and PCS_ML_BASER_CFG
	 *
	 * Only EPLs/lanes that are being used are initialized. All others
	 * are left at their defaults.
	 */
	for (i = 0; i < FM10K_SW_EPLS_MAX; i++) {
		if (!FM10K_SW_EXT_PORTS_EPL_USED(ext_ports, i))
			continue;
		for (j = 0; j < ext_ports->ports_per_epl; j++) {
			fm10k_read_switch_array(sw, FM10K_SW_MAC_CFG(i, j),
			    mac_cfg, FM10K_SW_MAC_CFG_ARRAY_SIZE);

			/* dic enable */
			FM10K_SW_SET_ARRAY_BIT(mac_cfg,
			    MAC_CFG_TX_IDLE_ENABLE_DIC,
			    dic_enable, mac_cfg);

			/* ifg */
			FM10K_SW_REPLACE_ARRAY_FIELD(mac_cfg,
			    MAC_CFG_TX_IDLE_MIN_IFG_BYTES,
			    12, mac_cfg);

			/* tx pad size: (64 bytes + 8 preamble) / 4 */
			FM10K_SW_REPLACE_ARRAY_FIELD(mac_cfg,
			    MAC_CFG_TX_MIN_COLUMNS,
			    18, mac_cfg);

			/* tx clock compensation */
			timeout =
			    fm10k_switch_ppm_to_tx_clk_compensation_timeout(pcs,
				100);
			FM10K_SW_SET_ARRAY_BIT(mac_cfg,
			    MAC_CFG_TX_CLOCK_COMPENSATION_ENABLE,
			    (timeout > 0), mac_cfg);
			FM10K_SW_REPLACE_ARRAY_FIELD(mac_cfg,
			    MAC_CFG_TX_CLOCK_COMPENSATION_TIMEOUT,
			    timeout, mac_cfg);

			FM10K_SW_SET_ARRAY_BIT(mac_cfg,
			    MAC_CFG_PREAMBLE_MODE,
			    0, mac_cfg);
			FM10K_SW_REPLACE_ARRAY_FIELD(mac_cfg,
			    MAC_CFG_RX_MIN_FRAME_LENGTH,
			    64, mac_cfg);
			FM10K_SW_REPLACE_ARRAY_FIELD(mac_cfg,
			    MAC_CFG_RX_MAX_FRAME_LENGTH,
			    FM10K_SW_PACKET_SIZE_MAX, mac_cfg);
			FM10K_SW_SET_ARRAY_BIT(mac_cfg,
			    MAC_CFG_RX_IGNORE_IFG_ERRORS,
			    0, mac_cfg);
			FM10K_SW_REPLACE_ARRAY_FIELD(mac_cfg,
			    MAC_CFG_TX_FCS_MODE,
			    FM10K_SW_TX_MAX_FCS_MODE_REPLACE_NORMAL, mac_cfg);

			FM10K_SW_SET_ARRAY_BIT(mac_cfg,
			    MAC_CFG_IEEE_1588_ENABLE,
			    0, mac_cfg);
			FM10K_SW_REPLACE_ARRAY_FIELD(mac_cfg,
			    MAC_CFG_TX_DRAIN_MODE,
			    FM10K_SW_TX_MAC_DRAIN_MODE_DRAIN_NORMAL, mac_cfg);

			FM10K_SW_REPLACE_ARRAY_FIELD(mac_cfg,
			    MAC_CFG_TX_PC_ACT_TIMEOUT,
			    100, mac_cfg);
			FM10K_SW_REPLACE_ARRAY_FIELD(mac_cfg,
			    MAC_CFG_TX_PC_ACT_TIME_SCALE,
			    3, mac_cfg);
			FM10K_SW_REPLACE_ARRAY_FIELD(mac_cfg,
			    MAC_CFG_TX_LPI_TIMEOUT,
			    180, mac_cfg);
			FM10K_SW_REPLACE_ARRAY_FIELD(mac_cfg,
			    MAC_CFG_TX_LPI_TIME_SCALE,
			    1, mac_cfg);
			FM10K_SW_REPLACE_ARRAY_FIELD(mac_cfg,
			    MAC_CFG_TX_LPI_HOLD_TIMEOUT,
			    20, mac_cfg);
			FM10K_SW_REPLACE_ARRAY_FIELD(mac_cfg,
			    MAC_CFG_TX_LPI_HOLD_TIME_SCALE,
			    1, mac_cfg);

			FM10K_SW_SET_ARRAY_BIT(mac_cfg,
			    MAC_CFG_TX_LPI_AUTOMATIC,
			    1, mac_cfg);
			FM10K_SW_SET_ARRAY_BIT(mac_cfg,
			    MAC_CFG_TX_LP_IDLE_REQUEST,
			    0, mac_cfg);

			FM10K_SW_REPLACE_ARRAY_FIELD(mac_cfg,
			    MAC_CFG_TX_FAULT_MODE,
			    FM10K_SW_TX_MAC_FAULT_MODE_NORMAL, mac_cfg);

			FM10K_SW_REPLACE_ARRAY_FIELD(mac_cfg,
			    MAC_CFG_TX_ANTI_BUBBLE_WATERMARK,
			    anti_bubble_wm, mac_cfg);
			FM10K_SW_REPLACE_ARRAY_FIELD(mac_cfg,
			    MAC_CFG_TX_RATE_FIFO_WATERMARK,
			    rate_fifo_wm, mac_cfg);
			FM10K_SW_REPLACE_ARRAY_FIELD(mac_cfg,
			    MAC_CFG_TX_RATE_FIFO_FAST_INC,
			    rate_fifo_fast_inc, mac_cfg);
			FM10K_SW_REPLACE_ARRAY_FIELD(mac_cfg,
			    MAC_CFG_TX_RATE_FIFO_SLOW_INC,
			    rate_fifo_slow_inc, mac_cfg);

			fm10k_write_switch_array(sw, FM10K_SW_MAC_CFG(i, j),
			    mac_cfg, FM10K_SW_MAC_CFG_ARRAY_SIZE);

			data =
				fm10k_read_switch_reg(sw,
					FM10K_SW_LINK_RULES(i, j));

			/* link-up debounce params */
			FM10K_SW_REPLACE_REG_FIELD(data,
			    LINK_RULES_FAULT_TIME_SCALE_UP,
			    4, data);
			FM10K_SW_REPLACE_REG_FIELD(data,
			    LINK_RULES_FAULT_TICKS_UP,
			    30, data);

			/* link-down debounce params */
			FM10K_SW_REPLACE_REG_FIELD(data,
			    LINK_RULES_FAULT_TIME_SCALE_DOWN,
			    4, data);
			FM10K_SW_REPLACE_REG_FIELD(data,
			    LINK_RULES_FAULT_TICKS_DOWN,
			    5, data);

			FM10K_SW_REPLACE_REG_FIELD(data,
			    LINK_RULES_HEARTBEAT_TIME_SCALE,
			    cfg->ext_port_speed == 10 ? 4 : 0, data);
			fm10k_write_switch_reg(sw,
				FM10K_SW_LINK_RULES(i, j), data);
		}

		if (cfg->ext_port_speed != 10)
			fm10k_write_switch_reg(sw,
				FM10K_SW_PCS_ML_BASER_CFG(i), 0x00003fff);
	}

	/*
	 * LANE_CFG, LANE_SERDES_CFG, LANE_ENERGY_DETECT_CFG,
	 * LANE_SIGNAL_DETECT_CFG, and EPL_FIFO_ERROR_STATUS
	 *
	 * Only EPLs/lanes that are being used are initialized. All others
	 * are left at their defaults.
	 */
	for (i = 0; i < FM10K_SW_EPLS_MAX; i++) {
		if (!FM10K_SW_EXT_PORTS_EPL_USED(ext_ports, i))
			continue;
		for (j = 0; j < ext_ports->ports_per_epl; j++) {
			fm10k_write_switch_reg(sw,
				FM10K_SW_LANE_CFG(i, j), 0);
			fm10k_write_switch_reg(sw,
				FM10K_SW_LANE_SERDES_CFG(i, j), 0x1106);

			data = fm10k_read_switch_reg(sw,
			    FM10K_SW_LANE_ENERGY_DETECT_CFG(i, j));

			data |=
			FM10K_SW_LANE_ENERGY_DETECT_CFG_ED_MASK_RX_SIGNAL_OK |
			FM10K_SW_LANE_ENERGY_DETECT_CFG_ED_MASK_RX_RDY |
			FM10K_SW_LANE_ENERGY_DETECT_CFG_ED_MASK_RX_ACTIVITY;

			data &=
			~FM10K_SW_LANE_ENERGY_DETECT_CFG_ED_MASK_ENERGY_DETECT;
			fm10k_write_switch_reg(sw,
			    FM10K_SW_LANE_ENERGY_DETECT_CFG(i, j), data);

			data = fm10k_read_switch_reg(sw,
			    FM10K_SW_LANE_SIGNAL_DETECT_CFG(i, j));

			data &=
			~(FM10K_SW_LANE_SIGNAL_DETECT_CFG_SD_MASK_RX_SIGNAL_OK |
			FM10K_SW_LANE_SIGNAL_DETECT_CFG_SD_MASK_RX_RDY);

			data |=
			FM10K_SW_LANE_SIGNAL_DETECT_CFG_SD_MASK_RX_ACTIVITY |
			FM10K_SW_LANE_SIGNAL_DETECT_CFG_SD_MASK_ENERGY_DETECT;

			fm10k_write_switch_reg(sw,
			    FM10K_SW_LANE_SIGNAL_DETECT_CFG(i, j), data);

			data = 0;
			data |= ((1 << j) << 4) | (1 << j);
			fm10k_write_switch_reg(sw,
			    FM10K_SW_EPL_FIFO_ERROR_STATUS(i), data);
		}
	}

	error = fm10k_epl_serdes_reset_and_load_all(sw);
	if (error)
		goto done;

	/*
	 * EPL_FIFO_ERROR_STATUS LINK_IP
	 */
	for (i = 0; i < FM10K_SW_EPLS_MAX; i++) {
		if (!FM10K_SW_EXT_PORTS_EPL_USED(ext_ports, i))
			continue;
		for (j = 0; j < ext_ports->ports_per_epl; j++) {
			fm10k_write_switch_reg(sw,
			    FM10K_SW_LINK_IP(i, j), FM10K_SW_MASK32(31, 0));
		}

		data = 0;
		data |= ((1 << j) << 4) | (1 << j);
		fm10k_write_switch_reg(sw,
		    FM10K_SW_EPL_FIFO_ERROR_STATUS(i), data);
	}
done:
	return (error);
}

int fm10k_switch_dpdk_port_no_get(struct fm10k_hw *hw);

int
fm10k_switch_mirror_set(struct fm10k_hw *hw, u16 dest_port, u16 vlan)
{
	struct fm10k_switch *sw = &fm10k_sw;
	int src_port = fm10k_switch_dpdk_port_no_get(hw);

	/* source port is external port number */
	if (src_port < 0 || src_port == dest_port)
		return -1;

	return fm10k_ffu_mirror_set(sw, src_port, dest_port, vlan);
}

int fm10k_switch_mirror_reset(struct fm10k_hw *hw)
{
	struct fm10k_switch *sw = &fm10k_sw;
	int src_port = fm10k_switch_dpdk_port_no_get(hw);

	if (src_port < 0)
		return -1;

	return fm10k_ffu_mirror_reset(sw, src_port);
}

typedef struct {
	u8 lport;
	u8 has_ftag;
} fm10k_sw_lport;

static int
fm10k_switch_init(struct fm10k_hw *hw, struct fm10k_switch *sw)
{
	u32 data;
	unsigned int num_lports = sw->info->num_peps + FM10K_SW_EPLS_SUPPORTED;
	fm10k_sw_lport all_lports[num_lports];
	struct fm10k_device_info *cfg = sw->info;
	unsigned int i;
	unsigned int is_quad;
	unsigned int table_idx;
	int error;
	u32 watermark;
	u64 data64, data64_2;

	/*
	 * Build list of all logical ports that might appear in the
	 * scheduler program.  Note that for any particular card
	 * configuration, not all of these logical ports may be used.
	 */
	table_idx = 0;
	for (i = 0; i < sw->info->num_peps; i++, table_idx++) {
		all_lports[table_idx].lport = sw->pep_map[i].logical_port;
		all_lports[table_idx].has_ftag = 1;
	}
	for (i = 0; i < FM10K_SW_EPLS_SUPPORTED; i++, table_idx++) {
		all_lports[table_idx].lport = sw->epl_map[i].logical_port;
		all_lports[table_idx].has_ftag = 0;
	}

	if (table_idx != num_lports) {
		FM10K_SW_ERR("fm10k switch lport table construction error");
		return -1;
	}

	/*
	 * Reset the switch to get to the default state
	 */
	data = fm10k_read_switch_reg(sw, FM10K_SW_SOFT_RESET);
	data &= ~FM10K_SW_SOFT_RESET_SWITCH_READY;
	fm10k_write_switch_reg(sw, FM10K_SW_SOFT_RESET, data);
	fm10k_write_flush(sw);

	usec_delay(100);
	data = fm10k_read_switch_reg(sw, FM10K_SW_SOFT_RESET);
	data |= FM10K_SW_SOFT_RESET_SWITCH_RESET |
			FM10K_SW_SOFT_RESET_EPL_RESET;
	fm10k_write_switch_reg(sw, FM10K_SW_SOFT_RESET, data);
	fm10k_write_flush(sw);
	usec_delay(1000);

	/* Clear memories */
	fm10k_write_switch_reg64(sw, FM10K_SW_BIST_CTRL,
	    FM10K_SW_BIST_CTRL_BIST_MODE_FABRIC |
	    FM10K_SW_BIST_CTRL_BIST_MODE_TUNNEL |
	    FM10K_SW_BIST_CTRL_BIST_MODE_EPL);
	fm10k_write_flush(sw);

	fm10k_write_switch_reg64(sw, FM10K_SW_BIST_CTRL,
	    FM10K_SW_BIST_CTRL_BIST_MODE_FABRIC |
	    FM10K_SW_BIST_CTRL_BIST_MODE_TUNNEL |
	    FM10K_SW_BIST_CTRL_BIST_MODE_EPL |
	    FM10K_SW_BIST_CTRL_BIST_RUN_FABRIC |
	    FM10K_SW_BIST_CTRL_BIST_RUN_TUNNEL |
	    FM10K_SW_BIST_CTRL_BIST_RUN_EPL);
	fm10k_write_flush(sw);
	usec_delay(800);

	fm10k_write_switch_reg64(sw, FM10K_SW_BIST_CTRL, 0);
	fm10k_write_flush(sw);

	data = fm10k_read_switch_reg(sw, FM10K_SW_SOFT_RESET);
	data &= ~(FM10K_SW_SOFT_RESET_SWITCH_RESET |
			FM10K_SW_SOFT_RESET_EPL_RESET);
	fm10k_write_switch_reg(sw, FM10K_SW_SOFT_RESET, data);
	fm10k_write_flush(sw);
	/* ensure switch reset is deasserted for at least 100ns */
	usec_delay(1);

	sw->epl_sbus = fm10k_sbus_attach(sw, "EPL", FM10K_SW_SBUS_EPL_CFG);
	if (sw->epl_sbus == NULL) {
		error = -1;
		goto done;
	}

	/* Clear non-BIST accessible pause state memories */
	for (i = 0; i < FM10K_SW_LOGICAL_PORTS_MAX; i++) {
		fm10k_write_switch_reg64(sw,
		    FM10K_SW_CM_EGRESS_PAUSE_COUNT(i, 0), 0);
		fm10k_write_switch_reg64(sw,
		    FM10K_SW_CM_EGRESS_PAUSE_COUNT(i, 1), 0);
	}

	/* Initialize RXQ_MCAST list */
	for (i = 0; i < FM10K_SW_SCHED_RXQ_STORAGE_POINTERS_ENTRIES; i++) {
		fm10k_write_switch_reg64(sw,
		    FM10K_SW_SCHED_RXQ_STORAGE_POINTERS(i),
		    FM10K_SW_MAKE_REG_FIELD
			(SCHED_RXQ_STORAGE_POINTERS_HEAD_PAGE, i) |
		    FM10K_SW_MAKE_REG_FIELD
			(SCHED_RXQ_STORAGE_POINTERS_TAIL_PAGE, i));
	}
	for (i = 0;
	     i < FM10K_SW_SCHED_RXQ_FREELIST_INIT_ENTRIES -
		 FM10K_SW_SCHED_RXQ_STORAGE_POINTERS_ENTRIES; i++) {
		fm10k_write_switch_reg(sw,
		    FM10K_SW_SCHED_RXQ_FREELIST_INIT,
		    FM10K_SW_MAKE_REG_FIELD
			(SCHED_RXQ_FREELIST_INIT_ADDRESS,
			i + FM10K_SW_SCHED_RXQ_STORAGE_POINTERS_ENTRIES));
	}
	/* Initialize TXQ list */
	for (i = 0; i < FM10K_SW_SCHED_TXQ_HEAD_PERQ_ENTRIES; i++) {
		fm10k_write_switch_reg(sw,
		    FM10K_SW_SCHED_TXQ_HEAD_PERQ(i),
		    FM10K_SW_MAKE_REG_FIELD(SCHED_TXQ_HEAD_PERQ_HEAD, i));
		fm10k_write_switch_reg(sw,
		    FM10K_SW_SCHED_TXQ_TAIL0_PERQ(i),
		    FM10K_SW_MAKE_REG_FIELD(SCHED_TXQ_TAIL0_PERQ_TAIL, i));
		fm10k_write_switch_reg(sw,
		    FM10K_SW_SCHED_TXQ_TAIL1_PERQ(i),
		    FM10K_SW_MAKE_REG_FIELD(SCHED_TXQ_TAIL1_PERQ_TAIL, i));
	}
	for (i = 0;
	     i < FM10K_SW_SCHED_TXQ_FREELIST_INIT_ENTRIES -
		 FM10K_SW_SCHED_TXQ_HEAD_PERQ_ENTRIES; i++) {
		fm10k_write_switch_reg(sw,
		    FM10K_SW_SCHED_TXQ_FREELIST_INIT,
		    FM10K_SW_MAKE_REG_FIELD
			(SCHED_TXQ_FREELIST_INIT_ADDRESS,
			i + FM10K_SW_SCHED_TXQ_HEAD_PERQ_ENTRIES));
	}
	/* Initialize free segment list */
	for (i = 0; i < FM10K_SW_LOGICAL_PORTS_MAX; i++) {
		fm10k_write_switch_reg(sw,
		    FM10K_SW_SCHED_SSCHED_RX_PERPORT(i),
		    FM10K_SW_MAKE_REG_FIELD(SCHED_SSCHED_RX_PERPORT_NEXT, i));
	}
	for (i = 0;
	     i < FM10K_SW_SCHED_FREELIST_INIT_ENTRIES -
		 FM10K_SW_LOGICAL_PORTS_MAX; i++) {
		fm10k_write_switch_reg(sw,
		    FM10K_SW_SCHED_FREELIST_INIT,
		    FM10K_SW_MAKE_REG_FIELD
			(SCHED_FREELIST_INIT_ADDRESS,
			i + FM10K_SW_LOGICAL_PORTS_MAX));
	}
	/* Disable switch scan chain */
	fm10k_write_switch_reg(sw,
		FM10K_SW_SCAN_DATA_IN,
	    FM10K_SW_SCAN_DATA_IN_UPDATE_NODES |
		FM10K_SW_SCAN_DATA_IN_PASSTHRU);

	error = fm10k_switch_configure_epls(hw, sw);
	if (error)
		goto done;

	/*
	 * for now configure store-and-forward between PEPs and external
	 * ports regardless of relative speeds
	 */
	for (i = 0; i < num_lports; i++) {
		fm10k_write_switch_reg64(sw,
		    FM10K_SW_SAF_MATRIX(all_lports[i].lport),
		    FM10K_SW_SAF_MATRIX_ENABLE_SNF_ALL_PORTS);
	}

	/* Disable MTU violation trap */
	data = fm10k_read_switch_reg(sw, FM10K_SW_SYS_CFG_1);
	data &= ~FM10K_SW_SYS_CFG_1_TRAP_MTU_VIOLATIONS;
	fm10k_write_switch_reg(sw, FM10K_SW_SYS_CFG_1, data);

	/* Disable ingress VLAN filtering and learning */
	for (i = 0; i < num_lports; i++) {
		fm10k_write_switch_reg(sw,
		    FM10K_SW_PORT_CFG_3(all_lports[i].lport), 0);
	}
	/*
	 * Make all ports members of every VLAN, and configure every VLAN to
	 * use MST instance 0.
	 */
	data64 = 0;
	for (i = 0; i < num_lports; i++)
		data64 |=
			FM10K_SW_INGRESS_VID_TABLE_MEMBERSHIP
			(all_lports[i].lport);
	for (i = 0; i < FM10K_SW_INGRESS_VID_TABLE_ENTRIES; i++) {
		fm10k_write_switch_reg128(sw, FM10K_SW_INGRESS_VID_TABLE(i),
		    FM10K_SW_INGRESS_VID_TABLE_REFLECT, data64);
	}
	data64 = 0;
	for (i = 0; i < num_lports; i++)
		data64 |=
			FM10K_SW_EGRESS_VID_TABLE_MEMBERSHIP
			(all_lports[i].lport);
	for (i = 0; i < FM10K_SW_EGRESS_VID_TABLE_ENTRIES; i++)
		fm10k_write_switch_reg128(sw,
			FM10K_SW_EGRESS_VID_TABLE(i), 0, data64);

	/* Init MOD_VLAN_TAG_VID1_MAP */
	for (i = 0; i < FM10K_SW_INGRESS_VID_TABLE_ENTRIES; i++) {
		data64 = i;
		data64 = data64 << 48;
		fm10k_write_switch_reg64(sw,
			0xE80000 + 0x2 * i + 0x20000, data64);
	}

	/* Configure MST instance 0 to forward for all ports */
	data64 = 0;
	for (i = 0; i < num_lports; i++)
		data64 |=
			FM10K_SW_EGRESS_MST_TABLE_FORWARDING
			(all_lports[i].lport);
	fm10k_write_switch_reg64(sw,
			FM10K_SW_EGRESS_MST_TABLE(0), data64);

	data64 = 0;
	data64_2 = 0;
	for (i = 0; i < num_lports; i++) {
		if (all_lports[i].lport <
		    FM10K_SW_INGRESS_MST_TABLE_PORTS_PER_TABLE) {
			data64 |=
			    FM10K_SW_MAKE_REG_FIELD_IDX64
				(INGRESS_MST_TABLE_STP_STATE,
				all_lports[i].lport,
				FM10K_SW_INGRESS_MST_TABLE_STP_STATE_FORWARD,
				all_lports[i].lport, all_lports[i].lport);
		} else {
			data64_2 |=
			    FM10K_SW_MAKE_REG_FIELD_IDX64
				(INGRESS_MST_TABLE_STP_STATE,
				all_lports[i].lport,
				FM10K_SW_INGRESS_MST_TABLE_STP_STATE_FORWARD,
				all_lports[i].lport, all_lports[i].lport);
		}
	}
	fm10k_write_switch_reg64(sw,
			FM10K_SW_INGRESS_MST_TABLE(0, 0), data64);
	fm10k_write_switch_reg64(sw,
			FM10K_SW_INGRESS_MST_TABLE(1, 0), data64_2);

	for (i = 0; i < num_lports; i++) {
		data64 = fm10k_read_switch_reg64(sw,
		    FM10K_SW_PARSER_PORT_CFG_1(all_lports[i].lport));
		data64 |= FM10K_SW_PARSER_PORT_CFG_1_VLAN1_TAG(0) |
				  FM10K_SW_PARSER_PORT_CFG_1_VLAN2_TAG(0);
		fm10k_write_switch_reg64(sw,
		    FM10K_SW_PARSER_PORT_CFG_1(all_lports[i].lport), data64);

		/*
		 * Configure tags for f-tagged lports
		 */
		if (all_lports[i].has_ftag) {
			data64 = fm10k_read_switch_reg64(sw,
			    FM10K_SW_PARSER_PORT_CFG_1(all_lports[i].lport));
			data64 |= FM10K_SW_PARSER_PORT_CFG_1_FTAG;
			fm10k_write_switch_reg64(sw,
			    FM10K_SW_PARSER_PORT_CFG_1(all_lports[i].lport),
			    data64);

			data64 = fm10k_read_switch_reg64(sw,
			    FM10K_SW_MOD_PER_PORT_CFG_2(all_lports[i].lport));
			data64 |= FM10K_SW_MOD_PER_PORT_CFG_2_FTAG;
			fm10k_write_switch_reg64(sw,
			    FM10K_SW_MOD_PER_PORT_CFG_2(all_lports[i].lport),
			    data64);
		}
		data64 = fm10k_read_switch_reg64(sw,
		    FM10K_SW_PARSER_PORT_CFG_2(all_lports[i].lport));
		data64 |= FM10K_SW_PARSER_PORT_CFG_2_PARSE_L3;
		data64 |= FM10K_SW_PARSER_PORT_CFG_2_PARSE_L4;
		fm10k_write_switch_reg64(sw,
		    FM10K_SW_PARSER_PORT_CFG_2(all_lports[i].lport), data64);
	}

	/*
	 * Assign default SGLORTs for the EPL ports in the PARSER config.
	 * This isn't necessary for the PEPs as for those, as all frames
	 * ingressing from PEPs are tagged with an SGLORT that is derived
	 * from a per-tx-queue setting.
	 *
	 * The PORT_CFG_ISL register offset is determined by logical port
	 * number and the register contents determined by the corresponding
	 * SGLORT.
	 */
	for (i = 0; i < FM10K_SW_EPLS_SUPPORTED; i++) {
		fm10k_write_switch_reg(sw,
			    FM10K_SW_PORT_CFG_ISL
				(fm10k_sw.epl_map[i].logical_port),
			    FM10K_SW_MAKE_REG_FIELD
				(PORT_CFG_ISL_SGLORT,
				fm10k_sw.epl_map[i].glort));
	}

	/*
	 * FFU counter, 11.10.3.5 POLICER_CFG[0..3]
	 */
	fm10k_write_switch_reg64(sw,
			0xE40000 + 0x2 * FM10K_SW_FFU_CNT_BANK + 0x11000,
			FM10K_SW_POLICER_LAST);

	/*
	 * 11.10.3.1 POLICER_CFG_4K[0..1][0..4095]
	 * 11.10.3.2 POLICER_CFG_512[0..1][0..511]
	 */
	for (i = 0; i < FM10K_SW_FFU_CNT_MAX; i++) {
		if (FM10K_SW_FFU_CNT_BANK < 2) {
			fm10k_read_switch_reg64(sw,
				0xE40000 + 0x2000 * FM10K_SW_FFU_CNT_BANK +
				0x2 * (i + FM10K_SW_FFU_CNT_START) + 0x0);
		} else {
			fm10k_read_switch_reg64(sw,
				0xE40000 + 0x400 * (FM10K_SW_FFU_CNT_BANK - 2) +
				0x2 * (i + FM10K_SW_FFU_CNT_START) + 0x4000);
		}
	}

	/*
	 * FFU
	 *
	 * The FFU is programmed to match on SGLORT and to set the DGLORT to
	 * a unique value corresponding to the given SGLORT.  One TCAM entry
	 * is required per host port per PEP and one per external interface.
	 * Only slice 0 is used.
	 */
	if (fm10k_ffu_init(sw, sw->dpdk_cfg) < 0)
		return -1;

	/*
	 * Program the segment scheduler (see tables at top)
	 *
	 * The TX and RX schedules are the same.  Page 0 is used.
	 */
	if (cfg->ext_port_speed == 40 || cfg->ext_port_speed == 100)
		is_quad = 1;
	else
		is_quad = 0;
	for (i = 0; i < sizeof(fm10k_sched_prog) /
			sizeof(fm10k_sched_prog[0]); i++) {
		/*
		 * In addition to explicit idle cycles, non-quad port
		 * entries are converted to idle cycles if the interfaces
		 * are configured in quad-port mode.
		 */
		if (fm10k_sched_prog[i].idle ||
		    (!fm10k_sched_prog[i].quad && is_quad))
			data = FM10K_SW_SCHED_SCHEDULE_IDLE;
		else
			data = FM10K_SW_SCHED_SCHEDULE_ENTRY
				(fm10k_sched_prog[i].phys,
			    fm10k_sched_prog[i].log,
			    (fm10k_sched_prog[i].quad == FM10K_SW_QUAD_40_100) ?
			    is_quad : fm10k_sched_prog[i].quad);
		fm10k_write_switch_reg(sw,
			FM10K_SW_SCHED_RX_SCHEDULE(0, i), data);
		fm10k_write_switch_reg(sw,
			FM10K_SW_SCHED_TX_SCHEDULE(0, i), data);
	}

	fm10k_write_switch_reg(sw, FM10K_SW_SCHED_SCHEDULE_CTRL,
	    FM10K_SW_SCHED_SCHEDULE_CTRL_RX_ENABLE |
	    FM10K_SW_MAKE_REG_FIELD
		(SCHED_SCHEDULE_CTRL_RX_MAX_INDEX,
		(sizeof(fm10k_sched_prog) /
		sizeof(fm10k_sched_prog[0])) - 1) |
	    FM10K_SW_SCHED_SCHEDULE_CTRL_TX_ENABLE |
	    FM10K_SW_MAKE_REG_FIELD
		(SCHED_SCHEDULE_CTRL_TX_MAX_INDEX,
		(sizeof(fm10k_sched_prog) /
		sizeof(fm10k_sched_prog[0])) - 1));

	/* Per 5.7.10.4 */
	watermark = FM10K_SW_MEM_POOL_SEGS_MAX -
	    ((sw->info->num_peps +
	    FM10K_SW_EPLS_SUPPORTED * FM10K_SW_EPL_LANES) *
	    FM10K_SW_HOWMANY(FM10K_SW_PACKET_SIZE_MAX,
	    FM10K_SW_MEM_POOL_SEG_SIZE, FM10K_SW_MEM_POOL_SEG_SIZE)) -
	    FM10K_SW_MEM_POOL_SEGS_RSVD;
	fm10k_write_switch_reg(sw, FM10K_SW_CM_GLOBAL_WM,
	    FM10K_SW_MAKE_REG_FIELD(CM_GLOBAL_WM_WATERMARK, watermark));
	fm10k_write_switch_reg(sw, FM10K_SW_CM_GLOBAL_CFG,
	    FM10K_SW_CM_GLOBAL_CFG_WM_SWEEP_EN |
	    FM10K_SW_CM_GLOBAL_CFG_PAUSE_GEN_SWEEP_EN |
	    FM10K_SW_CM_GLOBAL_CFG_PAUSE_REC_SWEEP_EN |
	    FM10K_SW_MAKE_REG_FIELD
		(CM_GLOBAL_CFG_NUM_SWEEPER_PORTS,
		FM10K_SW_LOGICAL_PORTS_MAX));

	/* Configure stats counters */
	data = FM10K_SW_RX_STATS_CFG_ENABLE_ALL_BANKS;
	data64 =
	    FM10K_SW_MOD_STATS_CFG_ENABLE_GROUP_7 |
	    FM10K_SW_MOD_STATS_CFG_ENABLE_GROUP_8;
	for (i = 0; i < num_lports; i++) {
		fm10k_write_switch_reg(sw,
		    FM10K_SW_RX_STATS_CFG(all_lports[i].lport),
		    data | ((all_lports[i].has_ftag) ?
			FM10K_SW_MAKE_REG_FIELD
			(RX_STATS_CFG_PER_FRAME_ADJUSTMENT,
			FM10K_SW_FTAG_SIZE) :
			0));
		fm10k_write_switch_reg64(sw,
		    FM10K_SW_MOD_STATS_CFG(all_lports[i].lport),
		    data64);
	}

	/* Transition switch to ready */
	data = fm10k_read_switch_reg(sw, FM10K_SW_SOFT_RESET);
	data |= FM10K_SW_SOFT_RESET_SWITCH_READY;
	fm10k_write_switch_reg(sw, FM10K_SW_SOFT_RESET, data);

 done:
	return (error);
}

static void
fm10k_switch_leds_init(struct fm10k_switch *sw)
{
	struct fm10k_device_info *cfg = sw->info;
	unsigned int i;
	uint8_t addr;
	uint32_t data;
	int max;

	switch (FM10K_SW_CARD_ID(cfg->subvendor, cfg->subdevice)) {
	case FM10K_SW_CARD(SILICOM, PE340G2DBIR_QS41):
	case FM10K_SW_CARD(SILICOM_RB, PE340G2DBIR_QS41):
	case FM10K_SW_CARD(SILICOM, PE340G2DBIR_QS43):
	case FM10K_SW_CARD(SILICOM_RB, PE340G2DBIR_QS43):
	case FM10K_SW_CARD(SILICOM, PE340G2DBIR_QL4):
	case FM10K_SW_CARD(SILICOM_RB, PE340G2DBIR_QL4):
		FM10K_SW_I2C_REQ_LOCK(sw->i2c);
		/*
		 * Set up the first PCA9545 mux so we can get at the PCA9635
		 * that the LED control lines are connected to.
		 */
		fm10k_i2c_write8(sw->i2c, 0x70, 0x04);

		/*
		 * PCA9635 initialization
		 * only differences from defaults are noted
		 */

		/* MODE1 - put into sleep mode to ensure it is brought out
		 * of sleep without violating the oscillator startup wait
		 */
		fm10k_i2c_write16(sw->i2c, 0x6a, 0x00, 0x10);
		fm10k_udelay(10);

		/* MODE1 - normal mode, disable all call */
		fm10k_i2c_write16(sw->i2c, 0x6a, 0x00, 0x00);

		/* Wait for oscillator to stabilize after coming out of sleep */
		fm10k_udelay(500);

		/* MODE2 - group control is blinking, open drain outputs,
		 * OE high -> LEDn = 0
		 */
		fm10k_i2c_write16(sw->i2c, 0x6a, 0x01, 0x20);

		/* PWM0 - 100% duty cycle */
		fm10k_i2c_write16(sw->i2c, 0x6a, 0x02, 0xff);

		/* PWM1 - 100% duty cycle */
		fm10k_i2c_write16(sw->i2c, 0x6a, 0x03, 0xff);

		/* GRPPWM - 50% blink duty cycle */
		fm10k_i2c_write16(sw->i2c, 0x6a, 0x12, 0x80);

		/* GRPFREQ - FM10K_LED_BLINKS_PER_SECOND */
		if (24 / FM10K_LED_BLINKS_PER_SECOND > 1)
			max = 24 / FM10K_LED_BLINKS_PER_SECOND;
		else
			max = 1;
		fm10k_i2c_write16(sw->i2c, 0x6a, 0x13, max - 1);

		FM10K_SW_I2C_REQ_UNLOCK(sw->i2c);
		break;
	case FM10K_SW_CARD(SILICOM, PE3100G2DQIR_QXSL4):
	case FM10K_SW_CARD(SILICOM, PE3100G2DQIR_QXSL4_REV_2):
		FM10K_SW_I2C_REQ_LOCK(sw->i2c);
		/*
		 * Set up the first PCA9545 mux so we can get at the PCA9635s
		 * that the LED control lines are connected to.
		 */
		fm10k_i2c_write8(sw->i2c, 0x70, 0x01);

		/*
		 * PCA9635 initialization
		 * only differences from defaults are noted
		 */

		addr = 0x6a;
		for (i = 0; i < 2; i++) {
			/* MODE1 - put into sleep mode to ensure it is
			 * brought out of sleep without violating the
			 * oscillator startup wait
			 */
			fm10k_i2c_write16(sw->i2c, addr, 0x00, 0x10);
			fm10k_udelay(10);

			/* MODE1 - normal mode, disable all call */
			fm10k_i2c_write16(sw->i2c, addr, 0x00, 0x00);

			/* MODE2 - group control is blinking, open drain
			 * outputs, OE high -> LEDn = 0
			 */
			fm10k_i2c_write16(sw->i2c, addr, 0x01, 0x20);

			/* Wait for oscillator to stabilize
			 * after coming out of sleep
			 */
			fm10k_udelay(500);

			/* PWM0 - 100% duty cycle */
			fm10k_i2c_write16(sw->i2c, addr, 0x02, 0xff);

			/* PWM3 - 100% duty cycle */
			fm10k_i2c_write16(sw->i2c, addr, 0x05, 0xff);

			/* GRPPWM - 50% blink duty cycle */
			fm10k_i2c_write16(sw->i2c, addr, 0x12, 0x80);

			/* GRPFREQ - FM10K_LED_BLINKS_PER_SECOND */
			if (24 / FM10K_LED_BLINKS_PER_SECOND > 1)
				max = 24 / FM10K_LED_BLINKS_PER_SECOND;
			else
				max = 1;
			fm10k_i2c_write16(sw->i2c, addr, 0x13, max - 1);

			addr = 0x69;
		}
		FM10K_SW_I2C_REQ_UNLOCK(sw->i2c);
		break;

	case FM10K_SW_CARD(SILICOM, PE3100G2DQIRL_QXSL4):
		FM10K_SW_I2C_REQ_LOCK(sw->i2c);
		addr = 0x62;
		fm10k_i2c_write16(sw->i2c, addr, 0x03, 0x88);
		fm10k_i2c_write16(sw->i2c, addr, 0x01, 0x0);

		data = fm10k_read_switch_reg(sw, 0xc2b);
		data |= 1 << 24;
		fm10k_write_switch_reg(sw, 0xc2b, data);
		FM10K_SW_I2C_REQ_UNLOCK(sw->i2c);
		break;

	case FM10K_SW_CARD(SILICOM, PE3100G2DQIRM_QXSL4):
		FM10K_SW_I2C_REQ_LOCK(sw->i2c);
		/*
		 * PCA9538 initialization
		 * only differences from defaults are noted
		 */
		/*
		 *	00000000: No link
		 *	00000001: 10G Port 0
		 *  00000010: 40G Port 0
		 *  00000100: 25G Port 0
		 *  00001000: 100G Port 0
		 */
		addr = 0x62;
		fm10k_i2c_write16(sw->i2c, addr, 0x03, 0x0);
		fm10k_i2c_write16(sw->i2c, addr, 0x01, 0x0);

		addr = 0x65;
		fm10k_i2c_write16(sw->i2c, addr, 0x03, 0xf0);
		fm10k_i2c_write16(sw->i2c, addr, 0x01, 0x0);

		addr = 0x66;
		fm10k_i2c_write16(sw->i2c, addr, 0x03, 0x0);
		fm10k_i2c_write16(sw->i2c, addr, 0x01, 0x0);

		/* set LEC_CFG */
		data = fm10k_read_switch_reg(sw, 0xc2b);
		data |= 1 << 24;
		fm10k_write_switch_reg(sw, 0xc2b, data);

		/* port from rdifd, LED */
		fm10k_gpio_output_set(sw, 4, 0);
		FM10K_SW_I2C_REQ_UNLOCK(sw->i2c);
		break;

	default:
		FM10K_SW_ERR("don't know how to operate LEDs for this card "
		    "(subvendor=0x%04x subdevice=0x%04x)",
		    cfg->subvendor, cfg->subdevice);
		break;
	}
}

static unsigned int
fm10k_switch_pca9635_led_bits(uint8_t led_flags)
{
	unsigned int bits;

	if (led_flags & FM10K_SW_EXT_PORT_LED_FLAG_UP) {
		if (led_flags & FM10K_SW_EXT_PORT_LED_FLAG_ACTIVE)
			bits = 0x3; /* group blink */
		else
			bits = 0x1; /* full on */
	} else {
		bits = 0; /* off */
	}
	return (bits);
}

static void
fm10k_switch_process_leds(void *ctx)
{
	struct fm10k_switch *sw = ctx;
	struct fm10k_device_info *cfg = sw->info;
	struct fm10k_ext_ports *ports = sw->ext_ports;
	struct fm10k_ext_port *port;
	unsigned int i;
	unsigned int num_ports = ports->num_ports;
	uint32_t data;
	uint8_t update_port[num_ports];
	uint8_t led_flags = 0, read;
	uint8_t addr;

	FM10K_SW_SWITCH_LOCK(sw);

	if (sw->master_hw->sw_addr == NULL) {
		FM10K_SW_SWITCH_UNLOCK(sw);
		return;
	}

	for (i = 0; i < num_ports; i++) {
		port = &ports->ports[i];
		data = fm10k_read_switch_reg(sw,
		    FM10K_SW_EPL_LED_STATUS(port->eplno));
		led_flags =
		    ((data & FM10K_SW_EPL_LED_STATUS_PORT_LINK_UP
		    (port->first_lane)) ?
			FM10K_SW_EXT_PORT_LED_FLAG_UP : 0) |
		    ((data &
			(FM10K_SW_EPL_LED_STATUS_PORT_TRANSMITTING
			(port->first_lane) |
			FM10K_SW_EPL_LED_STATUS_PORT_RECEIVING
			(port->first_lane))) ?
			FM10K_SW_EXT_PORT_LED_FLAG_ACTIVE : 0);
		update_port[i] = (led_flags != port->last_led_flags);
		port->last_led_flags = led_flags;
	}
	FM10K_SW_SWITCH_UNLOCK(sw);

	switch (FM10K_SW_CARD_ID(cfg->subvendor, cfg->subdevice)) {
	case FM10K_SW_CARD(SILICOM, PE340G2DBIR_QS41):
	case FM10K_SW_CARD(SILICOM_RB, PE340G2DBIR_QS41):
	case FM10K_SW_CARD(SILICOM, PE340G2DBIR_QS43):
	case FM10K_SW_CARD(SILICOM_RB, PE340G2DBIR_QS43):
	case FM10K_SW_CARD(SILICOM, PE340G2DBIR_QL4):
	case FM10K_SW_CARD(SILICOM_RB, PE340G2DBIR_QL4):
		FM10K_SW_I2C_REQ_LOCK(sw->i2c);
		/*
		 * Set up the first PCA9545 mux so we can get at the PCA9635
		 * that the LED control lines are connected to.
		 */
		fm10k_i2c_write8(sw->i2c, 0x70, 0x04);

		if (!(update_port[0] || update_port[1])) {
			FM10K_SW_I2C_REQ_UNLOCK(sw->i2c);
			break;
		}

		led_flags =
		    fm10k_switch_pca9635_led_bits
			(ports->ports[0].last_led_flags) |
		    (fm10k_switch_pca9635_led_bits
		    (ports->ports[1].last_led_flags) << 2);

		fm10k_i2c_write16(sw->i2c, 0x6a, 0x14, led_flags);
		FM10K_SW_I2C_REQ_UNLOCK(sw->i2c);
		break;

	case FM10K_SW_CARD(SILICOM, PE3100G2DQIR_QXSL4):
	case FM10K_SW_CARD(SILICOM, PE3100G2DQIR_QXSL4_REV_2):
		FM10K_SW_I2C_REQ_LOCK(sw->i2c);
		/*
		 * Set up the first PCA9545 mux so we can get at the PCA9635s
		 * that the LED control lines are connected to.
		 */
		fm10k_i2c_write8(sw->i2c, 0x70, 0x01);

		/* will need to update for QSFPs operating
		 * as four independent lanes/ports
		 */
		for (i = 0; i < 2; i++) {
			if (update_port[i] == 0)
				continue;

			addr = (i == 0) ? 0x6a : 0x69;

			port = &ports->ports[i];
			led_flags =
				fm10k_switch_pca9635_led_bits
				(port->last_led_flags);

			switch (port->lane_speed * port->num_lanes) {
			case 100:
				fm10k_i2c_write16(sw->i2c,
					addr, 0x14, led_flags);
				break;
			case 40:
				fm10k_i2c_write16(sw->i2c,
					addr, 0x14, led_flags << 6);
				break;
			}
		}
		FM10K_SW_I2C_REQ_UNLOCK(sw->i2c);
		break;

	case FM10K_SW_CARD(SILICOM, PE3100G2DQIRL_QXSL4):
		FM10K_SW_I2C_REQ_LOCK(sw->i2c);
		led_flags = 0;
		addr = 0x62;
		for (i = 0; i < 2; i++) {
			if (!update_port[i])
				continue;
			port = &ports->ports[i];
			if (port->last_led_flags &
				FM10K_SW_EXT_PORT_LED_FLAG_UP) {
				switch (port->lane_speed * port->num_lanes) {
				case 100:
				case 25:
					led_flags |= 0x6 << (4 * i); /* 100G */
					break;
				case 40:
				case 10:
					led_flags |= 0x4 << (4 * i); /* 40G */
					break;
				default:
					led_flags = 0;
				}
			} else {
				led_flags = 0; /* off */
			}
		}

		if (update_port[0] || update_port[1]) {
			fm10k_i2c_read8_ext(sw->i2c, addr, 0x1, &read);
			if (update_port[0])
				led_flags |= read & 0xf0;
			else
				led_flags |= read & 0xf;
			fm10k_i2c_write16(sw->i2c, addr, 0x1, led_flags);
			fm10k_i2c_read8_ext(sw->i2c, addr, 0x1, &read);
		}
		FM10K_SW_I2C_REQ_UNLOCK(sw->i2c);
		break;

	case FM10K_SW_CARD(SILICOM, PE3100G2DQIRM_QXSL4):
		FM10K_SW_I2C_REQ_LOCK(sw->i2c);
		/* will need to update for QSFPs
		 * operating as four independent lanes/ports
		 */
		for (i = 0; i < 2; i++) {
			if (!update_port[i])
				continue;
			/*
			 *  00000000: No link
			 *  00000001: 10G Port 0
			 *  00000010: 40G Port 0
			 *  00000100: 25G Port 0
			 *  00001000: 100G Port 0
			 */
			addr = (i == 0) ? 0x62 : 0x66;
			port = &ports->ports[i];
			if (port->last_led_flags &
				FM10K_SW_EXT_PORT_LED_FLAG_UP) {
				switch (port->lane_speed * port->num_lanes) {
				case 100:
					led_flags = 0x08; /* 100G */
					break;
				case 40:
					led_flags = 0x02; /* 40G */
					break;
				}
			} else {
				led_flags = 0; /* off */
			}
			fm10k_i2c_write16(sw->i2c,
				addr, 0x1, led_flags);
		}
		FM10K_SW_I2C_REQ_UNLOCK(sw->i2c);
		break;
	}
}

static void *
fm10k_switch_leds_update(void *ctx)
{
	struct fm10k_switch *sw = ctx;

	while (sw->detaching == 0) {
		fm10k_switch_process_leds(ctx);
		usec_delay(FM10K_LED_POLL_INTERVAL_MS * 1000);
	}
	return NULL;
}

static void
fm10k_switch_leds_off(struct fm10k_switch *sw, struct fm10k_ext_ports *ports)
{
	struct fm10k_device_info *cfg = sw->info;
	struct fm10k_ext_port *port;
	unsigned int i;
	uint8_t led_flags;
	uint8_t addr;

	switch (FM10K_SW_CARD_ID(cfg->subvendor, cfg->subdevice)) {
	case FM10K_SW_CARD(SILICOM, PE340G2DBIR_QS41):
	case FM10K_SW_CARD(SILICOM_RB, PE340G2DBIR_QS41):
	case FM10K_SW_CARD(SILICOM, PE340G2DBIR_QS43):
	case FM10K_SW_CARD(SILICOM_RB, PE340G2DBIR_QS43):
	case FM10K_SW_CARD(SILICOM, PE340G2DBIR_QL4):
	case FM10K_SW_CARD(SILICOM_RB, PE340G2DBIR_QL4):
		FM10K_SW_I2C_REQ_LOCK(sw->i2c);
		/*
		 * Set up the first PCA9545 mux so we can get at the PCA9635
		 * that the LED control lines are connected to.
		 */
		fm10k_i2c_write8(sw->i2c, 0x70, 0x04);

		led_flags =
		    fm10k_switch_pca9635_led_bits(0) |
		    (fm10k_switch_pca9635_led_bits(0) << 2);

		fm10k_i2c_write16(sw->i2c, 0x6a, 0x14, led_flags);
		FM10K_SW_I2C_REQ_UNLOCK(sw->i2c);
		break;

	case FM10K_SW_CARD(SILICOM, PE3100G2DQIR_QXSL4):
	case FM10K_SW_CARD(SILICOM, PE3100G2DQIR_QXSL4_REV_2):
	case FM10K_SW_CARD(SILICOM, PE3100G2DQIRL_QXSL4):
	case FM10K_SW_CARD(SILICOM, PE3100G2DQIRM_QXSL4):
		FM10K_SW_I2C_REQ_LOCK(sw->i2c);
		/*
		 * Set up the first PCA9545 mux so we can get at the PCA9635s
		 * that the LED control lines are connected to.
		 */
		fm10k_i2c_write8(sw->i2c, 0x70, 0x01);

		/* will need to update for QSFPs
		 * operating as four independent lanes/ports
		 */
		addr = 0x6a;
		led_flags = fm10k_switch_pca9635_led_bits(0);
		for (i = 0; i < 2; i++) {
			port = &ports->ports[i];
			switch (port->lane_speed * port->num_lanes) {
			case 100:
				fm10k_i2c_write16(sw->i2c,
					addr, 0x14, led_flags);
				break;
			case 40:
				fm10k_i2c_write16(sw->i2c,
					addr, 0x14, led_flags << 6);
				break;
			}

			addr = 0x69;
		}
		FM10K_SW_I2C_REQ_UNLOCK(sw->i2c);
		break;
	}
}

static void *
fm10k_switch_process_intr(void *ctx)
{
	struct fm10k_switch *sw = ctx;
	struct fm10k_ext_ports *ports;
	uint64_t gid;
	uint64_t reenable_mask;
	uint16_t epl_mask;

	while (sw->detaching == 0) {
		sem_wait(&sw->intr_tq);
		/*
		 * Mask off all global interrupt detect interrupts
		 * toward PCIe to ensure that the PEP sees the
		 * interrupt condition clear so that all subsequent
		 * events will be recognized.  The same result could
		 * be achieved by masking off all block-internal
		 * interrupt sources in all blocks prior to handling
		 * any global interrupt detect interrupts, but the
		 * approach taken here makes the interrupt processing
		 * easier to decompose and increases the likelihood that
		 * more work will be coalesced into fewer interrupts.
		 */
		FM10K_SW_TRACE("masking off PCIe gid interrupts");
		FM10K_SW_SWITCH_LOCK(sw);
		/*
		 * sw->ext_ports will be set to NULL during detach
		 * to indicate that we are to no longer process EPL
		 * interrupts.
		 */
		ports = sw->ext_ports;
		/*
		 * sw->sm_mailbox_enabled will be set to 0 during
		 * detach to indicate that we are to no longer process
		 * PEP interrupts.
		 */
		fm10k_write_switch_reg64(sw,
			FM10K_SW_INTERRUPT_MASK_PCIE(sw->pepno),
			FM10K_SW_INTERRUPT_MASK_PCIE_ALL);
		gid = fm10k_read_switch_reg64(sw,
				FM10K_SW_GLOBAL_INTERRUPT_DETECT);
		FM10K_SW_SWITCH_UNLOCK(sw);

		reenable_mask = 0;
		if (ports) {
			epl_mask = fm10k_ext_ports_epl_intrs(ports, gid);
			reenable_mask |=
				FM10K_SW_MAKE_REG_FIELD64
				(INTERRUPT_MASK_PCIE_EPL, epl_mask);
		}

		if (gid & FM10K_SW_GLOBAL_INTERRUPT_DETECT_I2C)
			fm10k_i2c_intr(sw->i2c);

		reenable_mask |= FM10K_SW_INTERRUPT_MASK_PCIE_I2C;

		FM10K_SW_TRACE("re-enabling PCIe gid interrupts");
		FM10K_SW_SWITCH_LOCK(sw);
		fm10k_write_switch_reg64(sw,
			FM10K_SW_INTERRUPT_MASK_PCIE(sw->pepno),
			~reenable_mask);
		FM10K_SW_SWITCH_UNLOCK(sw);
	}
	return NULL;
}

void
fm10k_switch_intr(struct fm10k_hw *hw)
{
	if (hw)
		sem_post(&fm10k_sw.intr_tq);
}

static void
fm10k_switch_enable_interrupts(struct fm10k_switch *sw)
{
	uint64_t data64;
	unsigned int i;

	data64 = fm10k_read_switch_reg64(sw,
			FM10K_SW_INTERRUPT_MASK_PCIE(sw->pepno));

	/* enable interrupts from all EPLs in use */
	FM10K_SW_REPLACE_REG_FIELD64(data64, INTERRUPT_MASK_PCIE_EPL,
	    ~sw->ext_ports->epl_mask, data64);

	/* enable interrupts from all non-master PEPs in use */
	FM10K_SW_REPLACE_REG_FIELD64(data64, INTERRUPT_MASK_PCIE_PCIE,
	    ~sw->pep_mask, data64);

	/* enable I2C interrupt */
	data64 &= ~FM10K_SW_INTERRUPT_MASK_PCIE_I2C;
	fm10k_write_switch_reg64(sw,
		FM10K_SW_INTERRUPT_MASK_PCIE(sw->pepno), data64);

	/* enable outbound mailbox interrupts from all non-master PEPs */
	for (i = 0; i < FM10K_SW_PEPS_MAX; i++) {
		if (sw->pep_mask & (1 << i))
			fm10k_write_switch_reg(sw, FM10K_SW_PCIE_GLOBAL(i, IM),
			    ~FM10K_SW_IM_MAILBOX);
	}
	FM10K_WRITE_REG(sw->master_hw,
		FM10K_SW_EIMR, FM10K_SW_EIMR_FIELD(SWITCH_INT, ENABLE));
}

static void
fm10k_switch_detach(struct fm10k_hw *hw)
{
	struct fm10k_ext_ports *ports;
	struct fm10k_switch *sw = &fm10k_sw;

	if (hw == NULL || sw == NULL)
		return;

	FM10K_SW_SWITCH_LOCK(sw);
	sw->detaching = 1;
	FM10K_SW_SWITCH_UNLOCK(sw);

	if (sw->ext_ports) {
		ports = sw->ext_ports;
		/*
		 * Ensure that any further switch interrupts will not
		 * process sw->ext_ports before detaching them.
		 */
		FM10K_SW_SWITCH_LOCK(sw);
		sw->ext_ports = NULL;
		FM10K_SW_SWITCH_UNLOCK(sw);
		fm10k_switch_leds_off(sw, ports);
		fm10k_ext_ports_detach(ports);
	}

	if (sw->epl_sbus)
		fm10k_sbus_detach(sw->epl_sbus);

	/*
	 * Detach i2c after ext_ports so ext_ports can use i2c to disable
	 * phys.
	 */
	if (sw->i2c)
		fm10k_i2c_detach(sw->i2c);

	fm10k_hw_eicr_disable_source(sw, FM10K_SW_EICR_SWITCH_INT);

	pthread_join(sw->intr_task, NULL);
	pthread_join(sw->led_task, NULL);
	pthread_join(sw->stats_task, NULL);
}

static unsigned int
fm10k_switch_get_fabric_clock(struct fm10k_switch *sw)
{
	uint32_t pll_fabric;
	unsigned int freq;

	pll_fabric = fm10k_read_switch_reg(sw, FM10K_SW_PLL_FABRIC_LOCK);

	switch (FM10K_SW_REG_FIELD(pll_fabric, PLL_FABRIC_FREQSEL)) {
	case FM10K_SW_PLL_FABRIC_FREQSEL_CTRL: {
		uint32_t ctrl;
		unsigned int refdiv, fbdiv4, fbdiv255;

		ctrl = fm10k_read_switch_reg(sw, FM10K_SW_PLL_FABRIC_CTRL);
		refdiv = FM10K_SW_REG_FIELD(ctrl, PLL_FABRIC_REFDIV);
		fbdiv4 = FM10K_SW_REG_FIELD(ctrl, PLL_FABRIC_FBDIV4);
		fbdiv255 = FM10K_SW_REG_FIELD(ctrl, PLL_FABRIC_FBDIV255);

		freq = (15625 * 4 * fbdiv255 * (1 + fbdiv4)) / (refdiv * 100);
		break;
	}
	case FM10K_SW_PLL_FABRIC_FREQSEL_F600:
		freq = 600;
		break;
	case FM10K_SW_PLL_FABRIC_FREQSEL_F500:
		freq = 500;
		break;
	case FM10K_SW_PLL_FABRIC_FREQSEL_F400:
		freq = 400;
		break;
	case FM10K_SW_PLL_FABRIC_FREQSEL_F300:
		freq = 300;
		break;
	default:
		freq = 0;
		break;
	}

	return freq;
}

static unsigned int
fm10k_switch_get_sku(struct fm10k_switch *sw)
{
	uint32_t fuse_data;

	fuse_data = fm10k_read_switch_reg(sw, FM10K_SW_FUSE_DATA_0);
	return FM10K_SW_REG_FIELD(fuse_data, FUSE_SKU);
}

static const char *
fm10k_switch_get_sku_name(unsigned int sku)
{
	const char *name;

	switch (sku) {
	case FM10K_SW_FUSE_SKU_FM10840:
		name = "FM10840";
		break;
	case FM10K_SW_FUSE_SKU_FM10420:
		name = "FM10420";
		break;
	case FM10K_SW_FUSE_SKU_FM10064:
		name = "FM10064";
		break;
	default:
		name = "FM10xxx-unknown";
		break;
	}

	return name;
}

static void
fm10k_switch_describe(struct fm10k_switch *sw)
{
	unsigned int i, pair;
	unsigned int reset_state, active_state;
	uint32_t device_cfg, resets;

	if (!fm10k_config_check_debug(sw->dpdk_cfg, FM10K_CONFIG_DEBUG_CONFIG))
		return;

	FM10K_SW_INFO("switch: device is %s, fabric clock %u MHz",
	    fm10k_switch_get_sku_name(fm10k_switch_get_sku(sw)),
	    fm10k_switch_get_fabric_clock(sw));

	device_cfg = fm10k_read_switch_reg(sw, FM10K_SW_DEVICE_CFG);
	FM10K_SW_INFO("switch: 100G Ethernet is %s",
	    (device_cfg & FM10K_SW_DEVICE_CFG_PCIE_100G_DIS) ?
	    "disabled" : "enabled");
	switch (FM10K_SW_REG_FIELD(device_cfg, DEVICE_CFG_FEATURE)) {
	case FM10K_SW_DEVICE_CFG_PCIE_FULL:
		FM10K_SW_INFO("switch: PEPs 0-7 support 2x4 and 1x8 "
		    "modes, PEP 8 is x1");
		break;
	case FM10K_SW_DEVICE_CFG_PCIE_HALF:
		FM10K_SW_INFO("switch: PEPs 0-3 support 2x4 and 1x8 "
		    "modes, PEPs 4 and 6 are 1x4, PEP 8 is x1");
		break;
	case FM10K_SW_DEVICE_CFG_PCIE_BASIC:
		FM10K_SW_INFO("switch: PEP 0 supports, PEP 8 is x1, "
		    "only one can be used");
		break;
	}

	resets = fm10k_read_switch_reg(sw, FM10K_SW_SOFT_RESET);
	for (i = 0; i < FM10K_SW_PEPS_MAX; i++) {
		if (!(device_cfg & FM10K_SW_DEVICE_CFG_PCIE_EN(i)))
			continue;

		pair = i / 2;
		reset_state = !!(resets & FM10K_SW_SOFT_RESET_PCIE_RESET(i));
		active_state = !!(resets & FM10K_SW_SOFT_RESET_PCIE_ACTIVE(i));
		if (pair < FM10K_SW_DEVICE_CFG_PCIE_MODE_PAIRS &&
		    !(device_cfg & FM10K_SW_DEVICE_CFG_PCIE_MODE_2X4(pair))) {
			if (i % 2 == 0)
				FM10K_SW_INFO("switch: PEP[%u,%u] is enabled in 1x8 "
				    "mode (Reset=%u, Active=%u)", i, i + 1,
				    reset_state, active_state);
			else
				FM10K_SW_INFO("switch: PEP[%u] unexpectedly enabled when "
				    "PEP[%u,%u] is in 1x8 mode (Reset=%u, "
				    "Active=%u)", i, i - 1, i, reset_state,
				    active_state);
		} else {
			FM10K_SW_INFO("switch: PEP[%u] is enabled in 1x4 "
			    "mode (Reset=%u, Active=%u)", i, reset_state,
			    active_state);
		}
	}
}

static struct fm10k_switch *
fm10k_switch_attach(struct fm10k_hw *hw, struct fm10k_switch *sw)
{
	int i;
	int error = 0;

	/*
	 * apparently no way to determine one's own PEP number.
	 */
	switch (sw->info->num_peps) {
	case 1:
		sw->pepno = 0;
		break;

	case 2:
		/*
		 * assumption is using even numbered PEPs
		 * starting with 0, and the highest numbered PEP
		 * is the master
		 */
		sw->pepno = 2;
		for (i = 0; i < sw->info->num_peps - 1; i++)
			sw->pep_mask |= (1 << (i * 2));
		break;

	case 4:
		sw->pepno = 6;
		sw->pep_mask = 0x7f;
		break;

	default:
		sw->pepno = 0;
	}

	/* When not zero, initialize serdes in 'near loopback' mode */
	sw->serdes_loopback = 0;

	pthread_mutex_init(&sw->lock, NULL);

	fm10k_switch_determine_epls(sw);

	sw->ext_ports = fm10k_ext_ports_attach(sw);
	if (sw->ext_ports == NULL)
		goto fail;

	/* label all external ports with their logical port numbers */
	for (i = 0; i < sw->ext_ports->num_ports; i++)
		sw->ext_ports->ports[i].lport = sw->epl_map[i].logical_port;

	/* interrupt */
	sem_init(&sw->intr_tq, 0, 0);
	pthread_create(&sw->intr_task, NULL, fm10k_switch_process_intr, sw);

	fm10k_switch_enable_interrupts(sw);
	fm10k_switch_describe(sw);

	sw->i2c = fm10k_i2c_attach(sw);
	if (sw->i2c == NULL)
		goto fail;

	error = fm10k_switch_init(hw, sw);
	if (error)
		goto fail;

	for (i = 0; i < sw->ext_ports->num_ports; i++)
		fm10k_ext_port_up(&sw->ext_ports->ports[i]);

	usec_delay(10000);
	fm10k_switch_leds_init(sw);

	pthread_create(&sw->led_task, NULL, &fm10k_switch_leds_update, sw);
	pthread_create(&sw->stats_task, NULL, &fm10k_switch_process_stats, sw);
	sw->inited = 1;
	return sw;
fail:
	if (sw != NULL)
		fm10k_switch_detach(hw);
	return NULL;
}

struct fm10k_switch*
fm10k_switch_get(void)
{
	return &fm10k_sw;
}

static int
fm10k_switch_dpdk_port_get(struct fm10k_switch *sw, int pf_no)
{
	int i;

	for (i = 0; i < FM10K_SW_LOGICAL_PORTS_MAX; i++) {
		if (sw->dpdk_cfg->ports[i].pf_no == pf_no)
			return i;
	}
	return -1;
}

static int
fm10k_switch_mapped_ext_port_get(struct fm10k_switch *sw, int pf_no)
{
	int i;
	struct fm10k_cfg_port_pf_map *map;

	for (i = 0; i < FM10K_SW_EXT_PORTS_MAX; i++) {
		map = &sw->dpdk_cfg->ext_port_map[i];
		if (map->type == FM10K_CONFIG_PORT_MAP_PF) {
			if (map->map_no[0] == pf_no)
				return i;
		} else if (map->type == FM10K_CONFIG_PORT_MAP_PFS) {
			if (map->map_no[0] == pf_no || map->map_no[1] == pf_no)
				return i;
		}
	}
	return -1;
}

static int
fm10k_switch_start(struct fm10k_switch *sw,
		struct fm10k_hw *master_hw, eth_fm10k_dev_init_half_func *func)
{
	int i, j;
	struct fm10k_dpdk_port *port;
	struct fm10k_cfg_port_pf_map *map;
	struct fm10k_cfg_port_pf_map *dpdk_map;
	int dpdk_port_no, ext_port_no = 0;
	int pf_no;

	sw->info = fm10k_get_device_info(master_hw);
	sw->master_hw = master_hw;
	sw->pep_map = fm10k_pep_port_map;
	sw->epl_map = fm10k_epl_port_map;

	sw->info->ext_port_speed = sw->dpdk_cfg->ext_port_speed;
	fm10k_switch_set_sched_prog();
	fm10k_switch_attach(master_hw, &fm10k_sw);

	for (i = 0; i < FM10K_SW_LOGICAL_PORTS_MAX; i++) {
		port = &sw->dpdk_cfg->ports[i];
		map = &sw->dpdk_cfg->dpdk_port_map[i];
		if (port->type != FM10K_CONFIG_DPDK_PF ||
			map->type != FM10K_CONFIG_PORT_MAP_PFS)
			continue;

		for (j = 0; j < 2; j++) {
			pf_no =	map->map_no[j];
			dpdk_port_no =
				fm10k_switch_dpdk_port_get(sw, pf_no);
			dpdk_map =
				&sw->dpdk_cfg->dpdk_port_map[dpdk_port_no];
			if (map->type == FM10K_CONFIG_PORT_MAP_PF) {
				dpdk_map->type = FM10K_CONFIG_PORT_MAP_PFSS;
				dpdk_map->map_no[0] =
				sw->dpdk_cfg->dpdk_port_map[i].map_no[0];
				dpdk_map->map_no[1] =
				sw->dpdk_cfg->dpdk_port_map[i].map_no[1];
			}
		}
	}

	/* Make sure Switch Manager is ready before going forward. */
	for (i = 0; i < FM10K_QUERY_SWITCH_STATE_TIMES; i++) {
		if (master_hw->switch_ready)
			break;
		/* Delay some time to acquire async LPORT_MAP info. */
		rte_delay_us(FM10K_WAIT_SWITCH_MSG_US);
	}

	if (!master_hw->switch_ready) {
		PMD_INIT_LOG(ERR, "switch is not ready");
		return -1;
	}

	/* do initialize all ports, after switch is ready */
	for (i = 0; i < FM10K_SW_LOGICAL_PORTS_MAX; i++) {
		uint32_t sglort;
		struct fm10k_dpdk_port *port = &sw->dpdk_cfg->ports[i];
		struct fm10k_hw *hw = port->hw;

		if (hw == NULL)
			break;

		map = &sw->dpdk_cfg->ext_port_map[ext_port_no];
		if (port->type == FM10K_CONFIG_DPDK_PF)	{
			pf_no = fm10k_switch_dpdk_pf_no_get(hw);
			if (map->type == FM10K_CONFIG_PORT_MAP_PF) {
				sglort = fm10k_switch_pf_glort_get(pf_no);
			} else if (map->type == FM10K_CONFIG_PORT_MAP_PFS) {
				ext_port_no =
					fm10k_switch_mapped_ext_port_get
					(sw, pf_no);
				sglort =
					fm10k_switch_pfs_glort_get
					(map->map_no[0], map->map_no[1]);
			} else {
				FM10K_SW_ERR("Unknown mapped port type %d!",
						port->type);
				return -1;
			}
			hw->mac.dglort_map = sglort | 0xffff0000;
			hw->switch_ready = master_hw->switch_ready;
			func(hw);
		}
	}
	return 0;
}

static int
fm10k_switch_dpdk_port_reg(struct fm10k_switch *sw,
		struct fm10k_hw *hw, void *rte_dev, uint8_t is_pf, bool master)
{
	int i;
	struct fm10k_dpdk_port *port;
	struct fm10k_cfg_port_pf_map *map;

	if (sw->dpdk_cfg == NULL && is_pf) {
		if (fm10k_config_init(sw, hw) < 0)
			return -1;
	}

	for (i = 0; i < FM10K_SW_LOGICAL_PORTS_MAX; i++) {
		port = &sw->dpdk_cfg->ports[i];
		if (master && port->hw &&
				port->type == FM10K_CONFIG_DPDK_PF) {
			port->pf_no =
				sw->dpdk_cfg->pf_max - 1 -
				(hw->mac.addr[5] - port->hw->mac.addr[5]);
			sw->dpdk_cfg->pf_hw[port->pf_no] = port->hw;
		}

		if (port->hw == NULL) {
			port->hw = hw;
			port->rte_dev = rte_dev;
			fm10k_flow_list_init(&port->flow_list);
			map = &sw->dpdk_cfg->dpdk_port_map[i];
			if (is_pf) {
				sw->dpdk_cfg->pf_bind++;
				port->type = FM10K_CONFIG_DPDK_PF;
				if (map->type == FM10K_CONFIG_PORT_MAP_NULL) {
					map->type = FM10K_CONFIG_PORT_MAP_PF;
					map->map_no[0] = i;
				}
			} else {
				port->type = FM10K_CONFIG_DPDK_VF;
			}
			if (master)
				sw->dpdk_cfg->master_hw = hw;
			if (sw->dpdk_cfg->master_hw) {
				port->pf_no =
					sw->dpdk_cfg->pf_max - 1 -
					(sw->dpdk_cfg->master_hw->mac.addr[5] -
					hw->mac.addr[5]);
				sw->dpdk_cfg->pf_hw[port->pf_no] = port->hw;
			}

			return 0;
		}
	}
	return -1;
}

int
fm10k_switch_dpdk_port_no_get(struct fm10k_hw *hw)
{
	int i;

	for (i = 0; i < FM10K_SW_LOGICAL_PORTS_MAX; i++) {
		if (fm10k_sw.dpdk_cfg->ports[i].hw == NULL)
			break;

		if (fm10k_sw.dpdk_cfg->ports[i].hw == hw)
			return i;
	}
	return -1;
}

void *
fm10k_switch_dpdk_port_rte_dev_get(struct fm10k_hw *hw)
{
	int port_no = fm10k_switch_dpdk_port_no_get(hw);

	if (port_no < 0)
		return NULL;

	return fm10k_switch_get()->dpdk_cfg->ports[port_no].rte_dev;
}

struct fm10k_flow_list *
fm10k_switch_dpdk_port_flow_list_get(struct fm10k_hw *hw)
{
	int port_no = fm10k_switch_dpdk_port_no_get(hw);

	if (port_no < 0)
		return NULL;

	return fm10k_switch_get()->dpdk_cfg->ports[port_no].flow_list;
}

static int
fm10k_switch_dpdk_cfg_check(struct fm10k_switch *sw)
{
	int i;
	bool need_default = true;
	struct fm10k_dpdk_port *port;
	struct fm10k_cfg_port_pf_map *map;

	if (sw->dpdk_cfg->master_hw == NULL) {
		FM10K_SW_ERR("Master PF is not bound!!!");
		return -1;
	}

	for (i = 0; i < FM10K_SW_LOGICAL_PORTS_MAX; i++) {
		port = &sw->dpdk_cfg->ports[i];
		map = &sw->dpdk_cfg->dpdk_port_map[i];
		if (port->type == FM10K_CONFIG_DPDK_PF &&
				map->type != FM10K_CONFIG_PORT_MAP_NULL) {
			need_default = false;
			break;
		}
	}

	if (need_default) {
		for (i = 0; i < FM10K_SW_LOGICAL_PORTS_MAX; i++) {
			map = &sw->dpdk_cfg->dpdk_port_map[i];
			if (port->type == FM10K_CONFIG_DPDK_PF) {
				map->type = FM10K_CONFIG_PORT_MAP_PF;
				map->map_no[0] = i;
			}
		}
	}

	return 0;
}

static void
fm10k_switch_dpdk_cfg_describe(struct fm10k_switch *sw)
{
	int i;
	struct fm10k_cfg_port_pf_map *map;

	if (!fm10k_config_check_debug(sw->dpdk_cfg, FM10K_CONFIG_DEBUG_CONFIG))
		return;

	FM10K_SW_INFO("--- FM10K DYNAMIC CONFIG ---\n");
	FM10K_SW_INFO("  PF Bind  : %d\n", sw->dpdk_cfg->pf_bind);

	FM10K_SW_INFO("--- PF ---\n");
	for (i = 0; i < FM10K_SW_PEP_PORTS_MAX; i++)	{
		uint8_t *mac;
		struct rte_eth_dev *dev;
		struct rte_pci_device *pdev;

		if (sw->dpdk_cfg->pf_hw[i] == NULL)
			continue;
		dev = (struct rte_eth_dev *)
			fm10k_switch_dpdk_port_rte_dev_get
			(sw->dpdk_cfg->pf_hw[i]);
		pdev = RTE_ETH_DEV_TO_PCI(dev);
		mac = sw->dpdk_cfg->pf_hw[i]->mac.addr;
		FM10K_SW_INFO("  PF%d : Logical %d Glort %#x "
				"PCI Addr %02x:%02x.%x "
				"MAC Addr %02x:%02x:%02x:%02x:%02x:%02x\n",
				i, fm10k_pep_port_map[i].logical_port,
				fm10k_switch_pf_glort_get(i), pdev->addr.bus,
				pdev->addr.devid,
				pdev->addr.function,
				mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
	}

	FM10K_SW_INFO("--- DPDK PORT ---\n");
	for (i = 0; i < FM10K_SW_LOGICAL_PORTS_MAX; i++) {
		if (sw->dpdk_cfg->ports[i].type == FM10K_CONFIG_DPDK_NULL)
			break;
		map = &sw->dpdk_cfg->dpdk_port_map[i];
		if (map->type == FM10K_CONFIG_PORT_MAP_NULL)
			break;
		if (map->type == FM10K_CONFIG_PORT_MAP_PF) {
			FM10K_SW_INFO("  DPDK PORT%d : Map to PF%d\n",
					i, map->map_no[0]);
		} else {
			FM10K_SW_INFO("  DPDK PORT%d : Map to PF%d&PF%d Glort %#x\n",
					i,
					map->map_no[0],	map->map_no[1],
					fm10k_switch_pfs_glort_get
					(map->map_no[0], map->map_no[1]));
		}
	}

	FM10K_SW_INFO("--- EXT PORT ---\n");
	for (i = 0; i < sw->dpdk_cfg->ext_port_num; i++) {
		map = &sw->dpdk_cfg->ext_port_map[i];
		FM10K_SW_INFO("  EXT  PORT%d : Logical %d Glort %#x", i,
				fm10k_epl_port_map[i].logical_port,
				fm10k_switch_epl_glort_get(i));

		if (map->type == FM10K_CONFIG_PORT_MAP_NULL)
			FM10K_SW_INFO("\n");
		else if (map->type == FM10K_CONFIG_PORT_MAP_PF)
			FM10K_SW_INFO(" Map to PF%d\n", map->map_no[0]);
		else
			FM10K_SW_INFO(" Map to PF%d&PF%d Glort %#x\n",
					map->map_no[0], map->map_no[1],
					fm10k_switch_pfs_glort_get
					(map->map_no[0], map->map_no[1]));
	}
}

int
fm10k_switch_dpdk_port_start(struct fm10k_hw *hw, void *rte_dev,
		uint8_t is_pf, bool master, eth_fm10k_dev_init_half_func *func)
{
	int ret;
	struct fm10k_switch *sw = fm10k_switch_get();

	if (fm10k_switch_dpdk_port_reg(sw, hw, rte_dev, is_pf, master) != 0) {
		FM10K_SW_ERR("Register ports failed!!!");
		return -1;
	}

	/*
	 * After all pfs are started
	 * start switch here
	 */
	if (fm10k_sw.dpdk_cfg->pf_max != 0 &&
		fm10k_sw.dpdk_cfg->pf_bind == fm10k_sw.dpdk_cfg->pf_num) {
		if (fm10k_switch_dpdk_cfg_check(&fm10k_sw) != 0)
			return -1;

		ret = fm10k_switch_start(&fm10k_sw,
				fm10k_sw.dpdk_cfg->master_hw, func);
		fm10k_switch_dpdk_cfg_describe(&fm10k_sw);
		return ret;
	}
	return 0;
}

void fm10k_switch_dpdk_port_stop(struct fm10k_hw *hw)
{
	if (hw)
		return;
}

/*
 * for multi-host, only dpdk port 0 and 1 are real rx/tx packets
 * so we need init/setup/start dpdk port queue for them.
 * for dpdk port 2/3, we need init/setup except start.
 * we map the pf queue to 0/1 dpdk queue first, then map
 * the other pf queue to 2/3 dpdk queue.
 */
int
fm10k_switch_dpdk_hw_queue_map(struct fm10k_hw *hw,
		uint16_t queue, uint16_t max_queue,
		struct fm10k_hw **map_hw, uint16_t *map_queue)
{
	int idx, pf_no;
	int dpdk_port_no = fm10k_switch_dpdk_port_no_get(hw);
	struct fm10k_dpdk_port *port;
	struct fm10k_switch *sw = fm10k_switch_get();

	if (dpdk_port_no < 0) {
		FM10K_SW_ERR("Can not find the dpdk port!!!");
		return -1;
	}

	port = &sw->dpdk_cfg->ports[dpdk_port_no];

	if (port->type == FM10K_CONFIG_DPDK_VF) {
		FM10K_SW_ERR("Not support yet!!!");
		return -1;
	}

	if (port->type != FM10K_CONFIG_DPDK_PF) {
		FM10K_SW_ERR("Can not be here!!!");
		return -1;
	}

	if (sw->dpdk_cfg->dpdk_port_map[dpdk_port_no].type ==
			FM10K_CONFIG_PORT_MAP_PF) {
		pf_no = sw->dpdk_cfg->dpdk_port_map[dpdk_port_no].map_no[0];
		*map_hw = sw->dpdk_cfg->pf_hw[pf_no];
		*map_queue = queue;
		return 1;
	} else if (sw->dpdk_cfg->dpdk_port_map[dpdk_port_no].type ==
			FM10K_CONFIG_PORT_MAP_PFS) {
		idx = queue % 2;
		pf_no = sw->dpdk_cfg->dpdk_port_map[dpdk_port_no].map_no[idx];
		*map_hw = sw->dpdk_cfg->pf_hw[pf_no];
		*map_queue = queue / 2;
		return 1;
	} else if (sw->dpdk_cfg->dpdk_port_map[dpdk_port_no].type ==
			FM10K_CONFIG_PORT_MAP_PFSS) {
		idx = queue % 2;
		pf_no = sw->dpdk_cfg->dpdk_port_map[dpdk_port_no].map_no[idx];
		*map_hw = sw->dpdk_cfg->pf_hw[pf_no];
		*map_queue = (max_queue + 1) / 2 + queue / 2;
		return 0;
	} else if (sw->dpdk_cfg->dpdk_port_map[dpdk_port_no].type ==
			FM10K_CONFIG_PORT_MAP_NULL) {
		FM10K_SW_ERR("Unmapped dpdk port %d!!!", dpdk_port_no);
		return -1;
	}
	FM10K_SW_ERR("Unknown mapped type!!!");
	return -1;
}

int
fm10k_switch_dpdk_mapped_hw_get(struct fm10k_hw *hw, struct fm10k_hw *hw_list[])
{
	int pf_no, pf_no_ext;
	int dpdk_port_no = fm10k_switch_dpdk_port_no_get(hw);
	struct fm10k_dpdk_port *port;
	struct fm10k_cfg_port_pf_map *map;
	struct fm10k_switch *sw = fm10k_switch_get();

	if (dpdk_port_no < 0) {
		FM10K_SW_ERR("Can not find the dpdk port!!!");
		return -1;
	}

	port = &sw->dpdk_cfg->ports[dpdk_port_no];
	map = &sw->dpdk_cfg->dpdk_port_map[dpdk_port_no];

	if (port->type == FM10K_CONFIG_DPDK_VF) {
		hw_list[0] = hw;
		hw_list[1] = NULL;
		return 1;
	}

	if (port->type != FM10K_CONFIG_DPDK_PF) {
		FM10K_SW_ERR("Can not be here!!!");
		return -1;
	} else if (map->type ==	FM10K_CONFIG_PORT_MAP_PF) {
		pf_no = map->map_no[0];
		hw_list[0] = sw->dpdk_cfg->pf_hw[pf_no];
		hw_list[1] = NULL;
		return 1;
	} else if (map->type ==	FM10K_CONFIG_PORT_MAP_PFS) {
		pf_no = map->map_no[0];
		hw_list[0] = sw->dpdk_cfg->pf_hw[pf_no];
		pf_no_ext = map->map_no[1];
		hw_list[1] = sw->dpdk_cfg->pf_hw[pf_no_ext];
		return 2;
	} else if (map->type == FM10K_CONFIG_PORT_MAP_PFSS) {
		pf_no = map->map_no[0];
		hw_list[0] = sw->dpdk_cfg->pf_hw[pf_no];
		pf_no_ext = map->map_no[1];
		hw_list[1] = sw->dpdk_cfg->pf_hw[pf_no_ext];
		return 0;
	}

	hw_list[0] = NULL;
	hw_list[1] = NULL;
	return 0;
}

void
fm10k_switch_dpdk_tx_queue_num_set(struct fm10k_hw *hw, uint8_t num)
{
	int port_no = fm10k_switch_dpdk_port_no_get(hw);
	struct fm10k_switch *sw = fm10k_switch_get();

	if (port_no < 0) {
		FM10K_SW_ERR("Can not find the dpdk port!!!");
		return;
	}

	sw->dpdk_cfg->ports[port_no].tx_queue_num = num;
}

void
fm10k_switch_dpdk_rx_queue_num_set(struct fm10k_hw *hw, uint8_t num)
{
	int port_no = fm10k_switch_dpdk_port_no_get(hw);
	struct fm10k_switch *sw = fm10k_switch_get();

	if (port_no < 0) {
		FM10K_SW_ERR("Can not find the dpdk port!!!");
		return;
	}

	sw->dpdk_cfg->ports[port_no].rx_queue_num = num;
}

int
fm10k_switch_dpdk_pf_no_get(struct fm10k_hw *hw)
{
	int port_no = fm10k_switch_dpdk_port_no_get(hw);
	struct fm10k_switch *sw = fm10k_switch_get();

	if (port_no < 0) {
		FM10K_SW_ERR("Can not find the dpdk port!!!");
		return -1;
	}

	return sw->dpdk_cfg->ports[port_no].pf_no;
}


void
fm10k_switch_flowset_switchto(const char *name)
{
	struct fm10k_switch *sw = fm10k_switch_get();

	fm10k_ffu_flowset_switch(sw, name);
}

void
fm10k_switch_show_port(void)
{
	struct fm10k_switch *sw = fm10k_switch_get();

	fm10k_stats_epl_port_print(sw);
	fm10k_stats_dpdk_port_print(sw);
}

void
fm10k_switch_show_ffu(void)
{
	struct fm10k_switch *sw = fm10k_switch_get();

	fm10k_stats_ffu_count_print(sw);
}

void
fm10k_switch_show_bank(void)
{
	struct fm10k_switch *sw = fm10k_switch_get();

	fm10k_stats_port_bank_print(sw);
}
