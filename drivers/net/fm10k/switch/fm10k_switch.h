/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright 2019   Silicom Ltd. Connectivity Solutions
 */

#ifndef _FM10K_SW_SWITCH_H_
#define _FM10K_SW_SWITCH_H_

#include <rte_spinlock.h>
#include <pthread.h>
#include <sys/time.h>
#include <semaphore.h>

#include "../fm10k.h"
#include "fm10k_debug.h"
#include "fm10k_regs.h"

/*
 * EPL
 */
#define FM10K_SW_EXT_PORTS_MAX		4

#define FM10K_SW_EPLS_MAX		9
#define FM10K_SW_EPLS_SUPPORTED		2
#define FM10K_SW_EPL_LANES		4
#define FM10K_SW_EPLS_PEP_MAX		2

/*
 * PEP
 */
#define FM10K_SW_PEP_GROUP_MAX		10
#define FM10K_SW_LOGICAL_PORTS_MAX	48

#define FM10K_SW_PEPS_MAX		9
#define FM10K_SW_PEPS_SUPPORTED		4
#define FM10K_SW_PEP_PORTS_MAX		4
#define FM10K_SW_PEP_MAP_NUM_MAX	2

/*
 * GLORT
 */
#define FM10K_SW_GROUP_MAX		10
#define FM10K_SW_VFS_MAX		64
#define FM10K_SW_PFS_GLORT_START	0x1000
#define FM10K_SW_VF_GLORT_START		0x2000
#define FM10K_SW_MULTI_GLORT_START	0x3000
#define FM10K_SW_DPORT_MASK(lport)	(1ULL << (lport))

/*
 * CONFIG
 */
#define FM10K_SW_CONFIG_MAX		1000
#define FM10K_SW_FFU_RULE_MAX		256
#define FM10K_SW_FFU_RULE_ITEM_MAX	10

/*
 * FFU COUNT
 */
#define FM10K_SW_FFU_CNT_BANK		3
#define FM10K_SW_POLICER_LAST		19
#define FM10K_SW_FFU_CNT_START		(FM10K_SW_POLICER_LAST + 1)
#define FM10K_SW_FFU_CNT_MAX		100

#define FM10K_SW_PEP_QUEUES_MAX		256
#define FM10K_SW_VF_QUEUES_MAX		(FM10K_SW_PEP_QUEUES_MAX - 1)

#define FM10K_SW_FTAG_SIZE		8
#define FM10K_SW_PACKET_SIZE_MIN	17
#define FM10K_SW_PACKET_SIZE_MAX	(15 * 1024)
#define FM10K_SW_MTU_MAX		\
	(FM10K_SW_PACKET_SIZE_MAX - ETHER_HDR_LEN - ETHER_VLAN_ENCAP_LEN)
#define FM10K_SW_SEG_SIZE_MAX		(16 * 1024)
#define FM10K_SW_TSO_SIZE_MAX		(256 * 1024 - 1)

#define FM10K_SW_MEM_POOL_SEG_SIZE	192
#define FM10K_SW_MEM_POOL_SEGS_MAX	24576
#define FM10K_SW_MEM_POOL_SEGS_RSVD	256

/*
 * GLORT MAP
 */
#define FM10K_SW_EPLA_GLORT		0x10
#define FM10K_SW_EPLB_GLORT		0x20
#define FM10K_SW_PEP01_GLORT		0x30
#define FM10K_SW_PEP23_GLORT		0x40
#define FM10K_SW_PEP45_GLORT		0x50
#define FM10K_SW_PEP67_GLORT		0x60

/*
 * logical port number
 */
#define FM10K_SW_EPLA_LOGICAL_PORT	1
#define FM10K_SW_EPLB_LOGICAL_PORT	2

#define FM10K_SW_PEP01_LOGICAL_PORT	3
#define FM10K_SW_PEP23_LOGICAL_PORT	4
#define FM10K_SW_PEP45_LOGICAL_PORT	5
#define FM10K_SW_PEP67_LOGICAL_PORT	6

/*
 * physical port number
 */
#define FM10K_SW_EPLA_PHYSICAL_PORT	0
#define FM10K_SW_EPLB_PHYSICAL_PORT	4

#define FM10K_SW_PEP01_PHYSICAL_PORT	36
#define FM10K_SW_PEP23_PHYSICAL_PORT	40
#define FM10K_SW_PEP45_PHYSICAL_PORT	44
#define FM10K_SW_PEP67_PHYSICAL_PORT	48


#define FM10K_SW_CARD_ID(v_, d_)	(((v_) << 16) | (d_))
#define FM10K_SW_CARD(vname_, dname_)	\
		FM10K_SW_CARD_ID(FM10K_SW_VENDOR_ID_##vname_, \
				FM10K_SW_DEV_ID_##dname_)

/*
 * All IDs that may appear in the vendor ID or subsystem vendor ID.
 */
#define FM10K_SW_VENDOR_ID_INTEL	0x8086
#define FM10K_SW_VENDOR_ID_SILICOM	0x1374
#define FM10K_SW_VENDOR_ID_SILICOM_RB	0x1B2E

/*
 * All IDs that may appear in the device ID or subsystem device ID.
 */
#define FM10K_SW_DEV_ID_FM10K				0x15a4

/* Silicom cards */
#define FM10K_SW_DEV_ID_PE310G4DBIR_T			0x01B0
#define FM10K_SW_DEV_ID_PE310G4DBIR_SRD			0x01B1
#define FM10K_SW_DEV_ID_PE310G4DBIR_LRD			0x01B2
#define FM10K_SW_DEV_ID_PE310G4DBIR_ER			0x01B3
#define FM10K_SW_DEV_ID_PE310G4DBIR_DA			0x01B4
#define FM10K_SW_DEV_ID_PE340G2DBIR_QS41		0x01B8
#define FM10K_SW_DEV_ID_PE340G2DBIR_QS43		0x01B9
#define FM10K_SW_DEV_ID_PE340G2DBIR_QL4			0x01BA
#define FM10K_SW_DEV_ID_PE3100G2DQIR_QXSL4		0x01C0
#define FM10K_SW_DEV_ID_PE3100G2DQIR_QXSL4_REV_2	0x01C4
#define FM10K_SW_DEV_ID_PE3100G2DQIRL_QXSL4		0x01C1
#define FM10K_SW_DEV_ID_PE3100G2DQIRM_QXSL4		0x01C2
#define FM10K_SW_DEV_ID_PE325G2DSIR			0x01C8

/*
 * SWITCH
 */

struct fm10k_device_info {
	uint16_t subvendor;
	uint16_t subdevice;
	const char *desc;
	uint8_t num_ext_ports;
	uint8_t ext_port_speed;
	uint8_t num_epls;
	uint8_t num_peps;
};

static struct fm10k_device_info fm10k_device_table[] = {
	{   FM10K_SW_VENDOR_ID_SILICOM,	FM10K_SW_DEV_ID_PE3100G2DQIR_QXSL4,
		"Silicom PE3100G2DQiR-QX4/QS4/QL4",	2, 100, 2, 2 },
	{   FM10K_SW_VENDOR_ID_SILICOM,	FM10K_SW_DEV_ID_PE3100G2DQIRL_QXSL4,
		"Silicom PE3100G2DQiRL-QX4/QS4/QL4",	2, 100, 2, 2 },
	{   FM10K_SW_VENDOR_ID_SILICOM,	FM10K_SW_DEV_ID_PE3100G2DQIRM_QXSL4,
		"Silicom PE3100G2DQiRM-QX4/QS4/QL4",	2, 100, 2, 4 },
};

struct fm10k_i2c;
struct fm10k_sbus;
struct fm10k_hw;
struct fm10k_ext_ports;
struct fm10k_dpdk_cfg;

struct fm10k_sw_port_map {
	uint32_t glort;
	uint16_t logical_port; /* logical port number */
	uint16_t physical_port; /*  */
};

struct fm10k_switch {
	uint32_t *hw_addr;
	uint32_t *sw_addr;
	struct fm10k_hw *master_hw;
	struct fm10k_device_info *info;
	pthread_mutex_t lock;
	struct fm10k_i2c *i2c;
	struct fm10k_sbus *epl_sbus;
	struct fm10k_ext_ports *ext_ports;
	sem_t intr_tq;
	pthread_t intr_task;
	pthread_t led_task;
	pthread_t stats_task;
	uint32_t detaching;
	uint32_t glort_cam_ram_idx;
	uint32_t glort_dest_table_idx;
	uint32_t mcast_dest_table_idx;
	uint32_t mcast_len_table_idx;
	uint32_t mcast_vlan_table_idx;
	uint32_t epl_serdes_code_version_build_id;
	uint16_t pep_mask; /* mask of non-master peps */
	uint8_t pepno;
	uint8_t serdes_loopback;
	uint8_t epla_no;
	uint8_t eplb_no;
	uint8_t mac_addr[6];
	struct fm10k_dpdk_cfg *dpdk_cfg;
	struct fm10k_sw_port_map *pep_map;
	struct fm10k_sw_port_map *epl_map;
	int inited;
};

#define FM10K_SW_SWITCH_LOCK(sw_)	pthread_mutex_lock(&((sw_)->lock))
#define FM10K_SW_SWITCH_UNLOCK(sw_)	pthread_mutex_unlock(&((sw_)->lock))

#define FM10K_SW_HOWMANY(x, y, y1)	(((x) + (y) - 1) / (y1))

static inline uint64_t
fm10k_uptime_us(void)
{
	struct timeval tv;

	gettimeofday(&tv, NULL);
	return ((uint64_t)tv.tv_sec * 1000000 + tv.tv_usec);
}

static inline uint32_t
fm10k_read_switch_reg(struct fm10k_switch *sw, uint32_t reg)
{
	return ((volatile uint32_t *)sw->master_hw->sw_addr)[reg];
}

static inline uint64_t
fm10k_read_switch_reg64(struct fm10k_switch *sw, uint32_t reg)
{
	uint64_t temp, result;

	result = ((volatile uint32_t *)sw->master_hw->sw_addr)[reg];
	temp = ((volatile uint32_t *)sw->master_hw->sw_addr)[reg + 1];
	result |= temp << 32;

	return result;
}

static inline void
fm10k_read_switch_array(struct fm10k_switch *sw,
		uint32_t reg, uint32_t *results, uint32_t count)
{
	unsigned int i;

	for (i = 0; i < count; i++)
		results[i] =
			((volatile uint32_t *)sw->master_hw->sw_addr)[reg + i];
}

static inline void
fm10k_write_switch_reg(struct fm10k_switch *sw, uint32_t reg, uint32_t val)
{
	((volatile uint32_t *)sw->master_hw->sw_addr)[reg] = val;
}

static inline void
fm10k_write_switch_reg64(struct fm10k_switch *sw, uint32_t reg, uint64_t val)
{
	((volatile uint32_t *)sw->master_hw->sw_addr)[reg] = val & 0xffffffff;
	((volatile uint32_t *)sw->master_hw->sw_addr)[reg + 1] = val >> 32;
}

static inline void
fm10k_write_switch_reg128(struct fm10k_switch *sw,
		uint32_t reg, uint64_t val_hi, uint64_t val_lo)
{
	((volatile uint32_t *)sw->master_hw->sw_addr)[reg] =
			val_lo & 0xffffffff;
	((volatile uint32_t *)sw->master_hw->sw_addr)[reg + 1] =
			val_lo >> 32;
	((volatile uint32_t *)sw->master_hw->sw_addr)[reg + 2] =
			val_hi & 0xffffffff;
	((volatile uint32_t *)sw->master_hw->sw_addr)[reg + 3] =
			val_hi >> 32;
}

static inline void
fm10k_write_switch_array(struct fm10k_switch *sw,
		uint32_t reg, uint32_t *data, uint32_t count)
{
	unsigned int i;

	for (i = 0; i < count; i++)
		((volatile uint32_t *)sw->master_hw->sw_addr)[reg + i] =
				data[i];
}

static inline uint32_t
fm10k_write_flush(struct fm10k_switch *sw)
{
	return ((volatile uint32_t *)sw->master_hw->hw_addr)[FM10K_SW_CTRL];
}

static inline void
fm10k_gpio_output_set(struct fm10k_switch *sw, int gpio_pin, int value)
{
	uint32_t data;

	data = fm10k_read_switch_reg(sw, FM10K_SW_GPIO_CFG);
	data |= 1 << gpio_pin;
	data &= ~(1 << (gpio_pin + 16));

	/* set gpio output */
	fm10k_write_switch_reg(sw, FM10K_SW_GPIO_CFG, data);

	/*
	 * Wait 1 msec (PCA spec specifies reset pulse width = 4 ns and
	 *  and reset time = 100 ns)
	 */
	usec_delay(1000);
	/* set reset */
	data = fm10k_read_switch_reg(sw, FM10K_SW_GPIO_DATA);
	if (value == 0)
		data &= ~(1 << gpio_pin);
	else
		data |= 1 << gpio_pin;

	fm10k_write_switch_reg(sw, FM10K_SW_GPIO_DATA, data);
}

static struct fm10k_sw_port_map fm10k_pep_port_map[FM10K_SW_PEPS_SUPPORTED] = {
	{
		FM10K_SW_PEP01_GLORT,
		FM10K_SW_PEP01_LOGICAL_PORT,
		FM10K_SW_PEP01_PHYSICAL_PORT
	},
	{
		FM10K_SW_PEP23_GLORT,
		FM10K_SW_PEP23_LOGICAL_PORT,
		FM10K_SW_PEP23_PHYSICAL_PORT
	},
	{
		FM10K_SW_PEP45_GLORT,
		FM10K_SW_PEP45_LOGICAL_PORT,
		FM10K_SW_PEP45_PHYSICAL_PORT
	},
	{
		FM10K_SW_PEP67_GLORT,
		FM10K_SW_PEP67_LOGICAL_PORT,
		FM10K_SW_PEP67_PHYSICAL_PORT
	},
};

static struct fm10k_sw_port_map fm10k_epl_port_map[FM10K_SW_EPLS_SUPPORTED] = {
	{
		FM10K_SW_EPLA_GLORT,
		FM10K_SW_EPLA_LOGICAL_PORT,
		FM10K_SW_EPLA_PHYSICAL_PORT
	},
	{
		FM10K_SW_EPLB_GLORT,
		FM10K_SW_EPLB_LOGICAL_PORT,
		FM10K_SW_EPLB_PHYSICAL_PORT
	},
};

static inline uint32_t
fm10k_switch_pf_logical_get(uint8_t pf_no)
{
	return fm10k_pep_port_map[pf_no].logical_port;
}

static inline uint32_t
fm10k_switch_epl_logical_get(uint8_t epl_no)
{
	return fm10k_epl_port_map[epl_no].logical_port;
}

static inline uint32_t
fm10k_switch_vf_glort_get(uint8_t vf_no)
{
	return FM10K_SW_VF_GLORT_START + vf_no;
}

static inline uint32_t
fm10k_switch_pf_glort_get(uint8_t pf_no)
{
	return fm10k_pep_port_map[pf_no].glort;
}

static inline uint32_t
fm10k_switch_epl_glort_get(uint8_t epl_no)
{
	return fm10k_epl_port_map[epl_no].glort;
}

static inline uint32_t
fm10k_switch_pfs_glort_get(uint8_t pf1, uint8_t pf2)
{
	uint8_t idx;
	if (pf1 > pf2)
		idx = (pf2 & 0xf) | (pf1 << 4 & 0xf0);
	else
		idx = (pf1 & 0xf) | (pf2 << 4 & 0xf0);
	return FM10K_SW_PFS_GLORT_START + idx;
}

static inline struct fm10k_device_info*
fm10k_get_device_info(struct fm10k_hw *hw)
{
	unsigned int i;
	struct fm10k_device_info *info;
	uint16_t pci_vendor = hw->vendor_id;
	uint16_t pci_device = hw->device_id;
	uint16_t pci_subvendor = hw->subsystem_vendor_id;
	uint16_t pci_subdevice = hw->subsystem_device_id;

	if (pci_vendor != FM10K_SW_VENDOR_ID_INTEL ||
	    pci_device != FM10K_SW_DEV_ID_FM10K)
		return (NULL);

	for (i = 0;
			i < sizeof(fm10k_device_table) /
				sizeof(fm10k_device_table[0]);
			i++) {
		info = &fm10k_device_table[i];
		if (pci_subvendor == info->subvendor &&
		    pci_subdevice == info->subdevice) {
			return info;
		}
	}

	return NULL;
}

#define fm10k_udelay	usec_delay
typedef int eth_fm10k_dev_init_half_func(struct fm10k_hw *hw);

unsigned int fm10k_switch_eplidx_to_eplno
	(struct fm10k_switch *sw, unsigned int eplidx);
void fm10k_switch_intr(struct fm10k_hw *hw);

struct fm10k_switch *fm10k_switch_get(void);
struct fm10k_device_info *fm10k_get_device_info(struct fm10k_hw *hw);
int fm10k_switch_dpdk_port_start(struct fm10k_hw *hw, void *rte_dev,
		uint8_t is_pf, bool master, eth_fm10k_dev_init_half_func *func);
void fm10k_switch_dpdk_port_stop(struct fm10k_hw *hw);
int fm10k_switch_dpdk_hw_queue_map(struct fm10k_hw *hw,
		uint16_t queue, uint16_t max_queue,
		struct fm10k_hw **map_hw, uint16_t *map_queue);
int fm10k_switch_dpdk_mapped_hw_get(struct fm10k_hw *hw,
		struct fm10k_hw *hw_list[]);
int fm10k_switch_dpdk_pf_no_get(struct fm10k_hw *hw);
int fm10k_switch_dpdk_port_no_get(struct fm10k_hw *hw);
void *fm10k_switch_dpdk_port_rte_dev_get(struct fm10k_hw *hw);
struct fm10k_flow_list *
fm10k_switch_dpdk_port_flow_list_get(struct fm10k_hw *hw);
void fm10k_switch_dpdk_tx_queue_num_set(struct fm10k_hw *hw, uint8_t num);
void fm10k_switch_dpdk_rx_queue_num_set(struct fm10k_hw *hw, uint8_t num);

int fm10k_switch_mirror_set(struct fm10k_hw *hw, u16 dest_port, u16 vlan);
int fm10k_switch_mirror_reset(struct fm10k_hw *hw);

void fm10k_switch_flowset_switchto(const char *name);
void fm10k_switch_show_port(void);
void fm10k_switch_show_ffu(void);
void fm10k_switch_show_bank(void);

void fm10k_flow_list_init(void *flow_list);
const struct rte_flow_ops *fm10k_flow_ops_get(void);

#endif
