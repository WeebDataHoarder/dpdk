/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright 2019   Silicom Ltd. Connectivity Solutions
 */

#include <rte_time.h>
#include <rte_kvargs.h>
#include <rte_hash.h>
#include <rte_flow.h>
#include <rte_flow_driver.h>
#include <rte_tm_driver.h>

#include "fm10k_debug.h"
#include "fm10k_ext_port.h"
#include "fm10k_regs.h"
#include "fm10k_sbus.h"
#include "fm10k_serdes.h"
#include "fm10k_spico_code.h"
#include "fm10k_switch.h"

#define FM10K_SW_SERDES_BIST_TIMEOUT_MS			5000
#define FM10K_SW_SBM_INTERRUPT_TIMEOUT_MS		5000
#define FM10K_SW_SERDES_INTERRUPT_TIMEOUT_MS		3000
#define FM10K_SW_SERDES_INT02_TIMEOUT_MS		500

#define FM10K_SW_SERDES_CONFIG_DELAY_US			20
#define FM10K_SW_SERDES_RESET_DELAY_US			22
#define FM10K_SW_SERDES_DFE_STOP_CYCLE_DELAY_US		200

#define FM10K_SW_SERDES_DIVIDER_ETHMODE_25G		0xa5
#define FM10K_SW_SERDES_DIVIDER_ETHMODE_10G		0x42

#define FM10K_SW_SERDES_WIDTH_40			3  /* 25G */
#define FM10K_SW_SERDES_WIDTH_20			1  /* 10G */

#define FM10K_SW_SERDES_EQ_SEL_PRECUR			0
#define FM10K_SW_SERDES_EQ_SEL_ATTEN			1
#define FM10K_SW_SERDES_EQ_SEL_POSTCUR			2

#define FM10K_SW_SERDES_DFE_DEFAULT_HF			0x00
#define FM10K_SW_SERDES_DFE_DEFAULT_LF			0x0c
#define FM10K_SW_SERDES_DFE_DEFAULT_DC			0x38
#define FM10K_SW_SERDES_DFE_DEFAULT_BW			0x0f

#define FM10K_SW_SERDES_ICAL_STOP_MAX_CYCLES		10
#define FM10K_SW_SERDES_PCAL_STOP_MAX_CYCLES		500

#define FM10K_SERDES_SPICO_REG_0X00			0x00
#define FM10K_SERDES_SPICO_REG_0X01			0x01
#define FM10K_SERDES_SPICO_REG_0X02			0x02
#define FM10K_SERDES_SPICO_REG_0X03			0x03
#define FM10K_SERDES_SPICO_REG_0X04			0x04
#define FM10K_SERDES_SPICO_REG_0X05			0x05
#define FM10K_SERDES_SPICO_REG_0X07			0x07
#define FM10K_SERDES_SPICO_REG_0X08			0x08
#define FM10K_SERDES_SPICO_REG_0X0A			0x0A
#define FM10K_SERDES_SPICO_REG_0X0B			0x0b
#define FM10K_SERDES_SPICO_REG_0X14			0x14
#define FM10K_SERDES_SPICO_REG_0X16			0x16

/*
 * These values correspond to IES FM_PORT_LINK_OPTIMIZATION_BALANCE
 * settings.
 */
#define FM10K_SW_LINK_OPT_PARAM_A10G			0x4800
#define FM10K_SW_LINK_OPT_PARAM_B10G			0xA000
#define FM10K_SW_LINK_OPT_PARAM_A25G			0x10001
#define FM10K_SW_LINK_OPT_PARAM_B25G			0x10001

#define FM10K_SW_SERDES_DFE_DATA_LEVEL0_THRESHOLD	10

static int fm10k_spico_ram_bist(struct fm10k_sbus *sb);
static int fm10k_load_epl_spico_code(struct fm10k_switch *sw);
static int fm10k_sbm_spico_upload_image(struct fm10k_sbus *sb,
		const uint16_t *image, unsigned int num_words);
static int fm10k_sbm_check_crc_version_build_id(struct fm10k_sbus *sb,
		uint32_t expected_version_build_id);
static int fm10k_sbm_spico_do_crc(struct fm10k_sbus *sb);
static int fm10k_sbm_get_version_build_id(struct fm10k_sbus *sb,
		uint32_t *version_build_id);
static int fm10k_serdes_swap_upload_image(struct fm10k_sbus *sb,
		const uint16_t *image, unsigned int num_words);
static int fm10k_serdes_swap_alt_upload_image(struct fm10k_sbus *sb,
		const uint16_t *image, unsigned int num_words);
static int fm10k_swap_image_check_crc(struct fm10k_sbus *sb,
		unsigned int crc_code);
static int fm10k_serdes_spico_upload_image(struct fm10k_sbus *sb,
		uint8_t dev, const uint16_t *image, unsigned int num_words);
static int fm10k_epl_serdes_check_crc_version_build_id(struct fm10k_switch *sw,
	    uint32_t expected_version_build_id);
static int fm10k_epl_serdes_spico_do_crc(struct fm10k_switch *sw,
		unsigned int serdes);
static int fm10k_epl_serdes_spico_reset(struct fm10k_switch *sw,
		unsigned int serdes);
static int fm10k_epl_serdes_get_version_build_id(struct fm10k_switch *sw,
	    unsigned int serdes, uint32_t *version_build_id);
static int fm10k_sbm_spico_int(struct fm10k_sbus *sb,
		uint8_t int_num, uint32_t param, uint32_t *result);
static int fm10k_epl_serdes_spico_int(struct fm10k_switch *sw,
		unsigned int serdes, uint16_t int_num,
		uint32_t param, uint32_t *result);
static int fm10k_epl_serdes_set_bit_rate(struct fm10k_switch *sw,
		unsigned int serdes, unsigned int divider);
static int fm10k_epl_serdes_set_width_mode(struct fm10k_switch *sw,
		unsigned int serdes, unsigned int width_mode);
static void fm10k_epl_serdes_set_pcsl_width_mode(struct fm10k_switch *sw,
	    unsigned int serdes, unsigned int width_mode);
static int fm10k_epl_serdes_set_pcsl_bit_slip(struct fm10k_switch *sw,
		unsigned int serdes);
static int fm10k_epl_serdes_init_signal_ok(struct fm10k_switch *sw,
		unsigned int serdes, unsigned int threshold);
static int fm10k_epl_serdes_set_tx_eq(struct fm10k_switch *sw,
		unsigned int serdes, unsigned int which, unsigned int tx_eq);
static int fm10k_epl_serdes_spico_int02_retry(struct fm10k_switch *sw,
		unsigned int serdes, uint32_t data, unsigned int timeout_ms);
static int fm10k_epl_serdes_dma_reg_write(struct fm10k_switch *sw,
		unsigned int serdes, unsigned int addr, uint32_t data);
static int fm10k_epl_serdes_dma_esb_write(struct fm10k_switch *sw,
		unsigned int serdes, unsigned int addr, uint32_t data);
static int fm10k_epl_serdes_dma_esb_read_modify_write(struct fm10k_switch *sw,
	    unsigned int serdes, unsigned int addr, uint32_t data,
		uint32_t mask, uint32_t *result);
static int fm10k_epl_serdes_dma_esb_read(struct fm10k_switch *sw,
		unsigned int serdes, unsigned int addr, uint32_t *result);
static int fm10k_epl_serdes_spico_wr_only_int(struct fm10k_switch *sw,
		unsigned int serdes, unsigned int int_num, uint32_t param);
static void fm10k_epl_serdes_eplno_lane(struct fm10k_switch *sw,
		unsigned int serdes, unsigned int *eplno, unsigned int *lane);
static int fm10k_epl_serdes_configure_near_loopback
		(struct fm10k_ext_port_lane *lane);
static int fm10k_epl_serdes_config_dfe_param(struct fm10k_switch *sw,
	    unsigned int serdes, uint32_t param);
static int fm10k_epl_serdes_get_dfe_status(struct fm10k_switch *sw,
		unsigned int serdes, uint32_t *status);
static int fm10k_epl_serdes_get_ical_result(struct fm10k_switch *sw,
		unsigned int serdes, uint32_t *converged);

int
fm10k_epl_serdes_reset_and_load_all(struct fm10k_switch *sw)
{
	struct fm10k_sbus *sb = sw->epl_sbus;
	int error;

	FM10K_SW_TRACE("sbus %s: initializing EPL serdes", sb->name);

	error = fm10k_spico_ram_bist(sb);
	if (error)
		goto done;

	error = fm10k_load_epl_spico_code(sw);
	if (error)
		goto done;

done:
	return (error);
}

int
fm10k_spico_ram_bist(struct fm10k_sbus *sb)
{
	uint64_t start_time_us, elapsed_time_ms;
	uint32_t data;
	int error;

	FM10K_SW_TRACE("sbus %s: starting SPICO RAM BIST", sb->name);

	error = fm10k_sbus_write(sb, FM10K_SW_SBUS_ADDR_SPICO,
			FM10K_SERDES_SPICO_REG_0X00, 0x03);
	if (error)
		goto done;

	error = fm10k_sbus_write(sb, FM10K_SW_SBUS_ADDR_SPICO,
			FM10K_SERDES_SPICO_REG_0X00, 0x05);
	if (error)
		goto done;

	start_time_us = fm10k_uptime_us();
	elapsed_time_ms = 0;
	while (elapsed_time_ms < FM10K_SW_SERDES_BIST_TIMEOUT_MS) {
		elapsed_time_ms = (fm10k_uptime_us() - start_time_us) / 1000;
		error = fm10k_sbus_read(sb,
				FM10K_SW_SBUS_ADDR_SPICO,
				FM10K_SERDES_SPICO_REG_0X00, &data);
		if (error)
			goto done;

		if (data & 0x18) {
			error = fm10k_sbus_write(sb,
					FM10K_SW_SBUS_ADDR_SPICO,
					FM10K_SERDES_SPICO_REG_0X00, 0x00);
			if (error)
				goto done;
			if ((data & 0x18) != 0x08) {
				FM10K_SW_INFO("sbus %s: SPICO RAM "
				    "BIST failed with bad status (0x%08x)",
				    sb->name, data);
				error = -1;
				goto done;
			}
			goto done;
		}
	}
	fm10k_sbus_write(sb, FM10K_SW_SBUS_ADDR_SPICO,
			FM10K_SERDES_SPICO_REG_0X00, 0x00);
	error = -1;
	FM10K_SW_INFO("sbus %s: SPICO RAM BIST timed out after %u "
	    "ms", sb->name, (uint32_t)elapsed_time_ms);

done:
	return (error);
}

static int
fm10k_load_epl_spico_code(struct fm10k_switch *sw)
{
	struct fm10k_sbus *sb = sw->epl_sbus;
	const uint16_t *sbm_code_image;
	uint32_t sbm_code_size;
	uint32_t sbm_code_version_build_id;
	const uint16_t *serdes_code_image;
	uint32_t serdes_code_size;
	uint32_t serdes_code_version_build_id;
	const uint16_t *swap_code_image;
	uint32_t swap_code_size;
	unsigned int swap_crc_code;
	int error = 0;

	FM10K_SW_TRACE("sbus %s: loading SPICO code", sb->name);

	sbm_code_image = fm10000_sbus_master_code_prd;
	sbm_code_size = fm10000_sbus_master_code_size_prd;
	sbm_code_version_build_id =
	    fm10000_sbus_master_code_version_build_id_prd;

	FM10K_SW_TRACE("sbus %s: sbm code version=%4.4x build=%4.4x",
	    sb->name, sbm_code_version_build_id >> 16,
	    sbm_code_version_build_id & 0xffff);

	serdes_code_image = fm10000_serdes_spico_code_prd2;
	serdes_code_size = fm10000_serdes_spico_code_size_prd2;
	serdes_code_version_build_id =
	    fm10000_serdes_spico_code_version_build_id_prd2;
	sw->epl_serdes_code_version_build_id = serdes_code_version_build_id;

	FM10K_SW_TRACE("sbus %s: serdes code version=%4.4x build=%4.4x",
	    sb->name, serdes_code_version_build_id >> 16,
	    serdes_code_version_build_id & 0xffff);

	swap_code_image = fm10000_serdes_swap_code_prd2;
	swap_code_size = fm10000_serdes_swap_code_size_prd2;

	error = fm10k_sbm_spico_upload_image(sb, sbm_code_image,
	    sbm_code_size);
	if (error)
		goto done;

	error = fm10k_sbm_check_crc_version_build_id(sb,
	    sbm_code_version_build_id);
	if (error)
		goto done;

	if (swap_code_size > 0) {
		if ((sbm_code_version_build_id & 0x00008000) == 0) {
			error = fm10k_serdes_swap_upload_image(sb,
			    swap_code_image, swap_code_size);
			swap_crc_code = 0x1a;
		} else {
			error = fm10k_serdes_swap_alt_upload_image(sb,
			    swap_code_image, swap_code_size);
			swap_crc_code = 0x04;
		}
		if (error)
			goto done;

		error = fm10k_swap_image_check_crc(sb, swap_crc_code);
		if (error)
			goto done;
	}

	error = fm10k_serdes_spico_upload_image(sb,
			FM10K_SW_SBUS_ADDR_BROADCAST,
			serdes_code_image, serdes_code_size);
	if (error)
		goto done;

	error = fm10k_epl_serdes_check_crc_version_build_id(sw,
	    serdes_code_version_build_id);
	if (error)
		goto done;

done:
	return (error);
}

static int
fm10k_sbm_spico_upload_image(struct fm10k_sbus *sb,
		const uint16_t *image, unsigned int num_words)
{
	uint64_t start_time_us, elapsed_time_us;
	unsigned int i;
	uint32_t data;
	int error;

	FM10K_SW_TRACE("sbus %s: uploading SBM SPICO image "
	    "(%u words)", sb->name, num_words);

	data = (1 << 6) | (1 << 7);
	error = fm10k_sbus_write(sb, FM10K_SW_SBUS_ADDR_SPICO,
			FM10K_SERDES_SPICO_REG_0X01, data);
	if (error)
		goto done;

	data &= ~(1 << 7);
	error = fm10k_sbus_write(sb, FM10K_SW_SBUS_ADDR_SPICO,
			FM10K_SERDES_SPICO_REG_0X01, data);
	if (error)
		goto done;

	data |= (1 << 9);
	error = fm10k_sbus_write(sb, FM10K_SW_SBUS_ADDR_SPICO,
			FM10K_SERDES_SPICO_REG_0X01, data);
	if (error)
		goto done;

	start_time_us = fm10k_uptime_us();
	for (i = 0; i < num_words; i++) {
		data = 0x80000000 | ((uint32_t)image[i] << 16) | i;
		error = fm10k_sbus_write(sb,
				FM10K_SW_SBUS_ADDR_SPICO,
				FM10K_SERDES_SPICO_REG_0X03, data);
		if (error)
			goto done;
	}
	elapsed_time_us = fm10k_uptime_us() - start_time_us;
	FM10K_SW_TRACE("sbus %s: SBM image upload time %u ms",
	    sb->name, (uint32_t)(elapsed_time_us / 1000));

	data = (1 << 6);
	error = fm10k_sbus_write(sb, FM10K_SW_SBUS_ADDR_SPICO,
			FM10K_SERDES_SPICO_REG_0X01, data);
	if (error)
		goto done;

	data = (1 << 18) | (1 << 19);
	error = fm10k_sbus_write(sb, FM10K_SW_SBUS_ADDR_SPICO,
			FM10K_SERDES_SPICO_REG_0X16, data);
	if (error)
		goto done;

	data = (1 << 6) | (1 << 8);
	error = fm10k_sbus_write(sb, FM10K_SW_SBUS_ADDR_SPICO,
			FM10K_SERDES_SPICO_REG_0X01, data);
	if (error)
		goto done;

done:
	return (error);
}

static int
fm10k_sbm_check_crc_version_build_id(struct fm10k_sbus *sb,
		uint32_t expected_version_build_id)
{
	int error;
	uint32_t version_build_id;

	FM10K_SW_TRACE("sbus %s: checking SBM SPICO code CRC "
	    "and version", sb->name);

	error = fm10k_sbm_spico_do_crc(sb);
	if (error)
		goto done;

	error = fm10k_sbm_get_version_build_id(sb, &version_build_id);
	if (error)
		goto done;

	if (version_build_id != expected_version_build_id) {
		FM10K_SW_INFO("sbus %s: SBM SPICO code version "
		    "compare failed (expected 0x%08x, got 0x%08x)",
		    sb->name, expected_version_build_id, version_build_id);
		error = -1;
		goto done;
	}
	FM10K_SW_TRACE("sbus %s: SBM SPICO image "
	    "version=0x%4.4x build_id=0x%4.4x", sb->name,
	    version_build_id >> 16,
	    version_build_id & 0xffff);
done:
	return (error);
}

static int
fm10k_sbm_spico_do_crc(struct fm10k_sbus *sb)
{
	int error;
	uint32_t crc;

	FM10K_SW_TRACE("sbus %s: performing SBM SPICO CRC "
	    "check", sb->name);

	error = fm10k_sbm_spico_int(sb, 0x02, 0, &crc);
	if (error)
		goto done;

	crc = crc >> 16;
	if (crc == 0xffff) {
		FM10K_SW_INFO("sbus %s: SBM SPICO CRC check failed "
		    "(CRC interrupt returned 0xffff)", sb->name);
		error = -1;
		goto done;
	} else if (crc == 0x0001)
		FM10K_SW_TRACE("sbus %s: SBM SPICO CRC check "
		    "passed ", sb->name);
	else
		FM10K_SW_INFO("sbus %s: unexpected SBM SPICO CRC "
		    "check result 0x%04x", sb->name, crc);

done:
	return (error);
}

static int
fm10k_sbm_get_version_build_id(struct fm10k_sbus *sb,
		uint32_t *version_build_id)
{
	int error;
	uint32_t result;

	FM10K_SW_TRACE("sbus %s: getting SBM code version",
	    sb->name);

	error = fm10k_sbm_spico_int(sb, 0x00, 0, &result);
	if (error)
		goto done;

	*version_build_id = result & 0xffff0000;
	error = fm10k_sbm_spico_int(sb, 0x01, 0, &result);
	if (error)
		goto done;

	*version_build_id |= (result >> 16) & 0xffff;

done:
	return (error);
}

static int
fm10k_serdes_swap_upload_image(struct fm10k_sbus *sb,
		const uint16_t *image, unsigned int num_words)
{
	uint64_t start_time_us, elapsed_time_us;
	unsigned int i;
	uint32_t addr;
	uint32_t reg01, reg05;
	int error;

	FM10K_SW_TRACE("sbus %s: uploading SBM swap image "
	    "(%u words)", sb->name, num_words);

	error = fm10k_sbm_spico_int(sb, 0x1c, 0, &addr);
	if (error)
		goto done;

	addr >>= 16;
	if (addr == 0xffff) {
		FM10K_SW_INFO("sbus %s: invalid build - cannot "
		    "upload SBM swap image", sb->name);
		error = -1;
		goto done;
	}
	FM10K_SW_TRACE("sbus %s: SBM swap image initial load "
	    "address 0x%04x (%u)", sb->name, addr, addr);

	reg05 = 1;
	error = fm10k_sbus_write(sb, FM10K_SW_SBUS_ADDR_SPICO,
			FM10K_SERDES_SPICO_REG_0X05, reg05);
	if (error)
		goto done;

	error = fm10k_sbus_read(sb, FM10K_SW_SBUS_ADDR_SPICO,
			FM10K_SERDES_SPICO_REG_0X01, &reg01);
	if (error)
		goto done;

	reg01 |= (1 << 9);
	error = fm10k_sbus_write(sb, FM10K_SW_SBUS_ADDR_SPICO,
			FM10K_SERDES_SPICO_REG_0X01, reg01);
	if (error)
		goto done;

	start_time_us = fm10k_uptime_us();
	error = fm10k_sbus_write(sb, FM10K_SW_SBUS_ADDR_SPICO,
			FM10K_SERDES_SPICO_REG_0X03, addr);
	if (error)
		goto done;
	error = fm10k_sbus_write(sb,
			FM10K_SW_SBUS_ADDR_SPICO,
			FM10K_SERDES_SPICO_REG_0X03, 0x80000000 | addr);
	if (error)
		goto done;

	for (i = 0; i < num_words - 2; i += 3) {
		error = fm10k_sbus_write(sb, FM10K_SW_SBUS_ADDR_SPICO,
			FM10K_SERDES_SPICO_REG_0X14,
			0xc0000000 | image[i] |
			((uint32_t)image[i + 1] << 10) |
			((uint32_t)image[i + 2] << 20));
		if (error)
			goto done;
	}

	if (num_words - i == 2) {
		error = fm10k_sbus_write(sb, FM10K_SW_SBUS_ADDR_SPICO,
			FM10K_SERDES_SPICO_REG_0X14,
			0x80000000 | image[i] |
			((uint32_t)image[i + 1] << 10));
		if (error)
			goto done;
	} else if (num_words - i == 1) {
		error = fm10k_sbus_write(sb, FM10K_SW_SBUS_ADDR_SPICO,
			FM10K_SERDES_SPICO_REG_0X14,
			0x40000000 | image[i]);
		if (error)
			goto done;
	}
	elapsed_time_us = fm10k_uptime_us() - start_time_us;
	FM10K_SW_TRACE("sbus %s: SBM swap image upload time "
	    "%u ms", sb->name, (uint32_t)(elapsed_time_us / 1000));

	reg01 &= ~(1 << 9);
	error = fm10k_sbus_write(sb, FM10K_SW_SBUS_ADDR_SPICO,
		FM10K_SERDES_SPICO_REG_0X01, reg01);
	if (error)
		goto done;

	reg05 &= ~(1 << 0);
	error = fm10k_sbus_write(sb, FM10K_SW_SBUS_ADDR_SPICO,
		FM10K_SERDES_SPICO_REG_0X05, reg05);
	if (error)
		goto done;

done:
	return (error);
}

static int
fm10k_serdes_swap_alt_upload_image(struct fm10k_sbus *sb,
		const uint16_t *image, unsigned int num_words)
{
	uint64_t start_time_us, elapsed_time_us;
	unsigned int i;
	uint32_t addr;
	uint32_t reg01, reg05;
	int error;

	FM10K_SW_TRACE("sbus %s: alt-uploading SBM swap image "
	    "(%u words)", sb->name, num_words);

	reg05 = 1;
	error = fm10k_sbus_write(sb, FM10K_SW_SBUS_ADDR_SPICO,
		FM10K_SERDES_SPICO_REG_0X05, reg05);
	if (error)
		goto done;

	error = fm10k_sbus_read(sb, FM10K_SW_SBUS_ADDR_SPICO,
		FM10K_SERDES_SPICO_REG_0X01, &reg01);
	if (error)
		goto done;

	reg01 |= (1 << 10) | (1 << 11);
	error = fm10k_sbus_write(sb, FM10K_SW_SBUS_ADDR_SPICO,
		FM10K_SERDES_SPICO_REG_0X01, reg01);
	if (error)
		goto done;

	addr = 0x0400;
	FM10K_SW_TRACE("sbus %s: SBM swap image initial load "
	    "address 0x%04x (%u)", sb->name, addr, addr);

	start_time_us = fm10k_uptime_us();
	for (i = 0; i < num_words; i++) {
		error = fm10k_sbus_write(sb, FM10K_SW_SBUS_ADDR_SPICO,
			FM10K_SERDES_SPICO_REG_0X04,
			0x8000 | ((uint32_t)image[i + 1] << 16) |
			(addr + i));
		if (error)
			goto done;
	}
	elapsed_time_us = fm10k_uptime_us() - start_time_us;
	FM10K_SW_TRACE("sbus %s: SBM swap image alt-upload "
	    "time %u ms", sb->name, (uint32_t)(elapsed_time_us / 1000));

	reg01 &= ~((1 << 10) | (1 << 11));
	error = fm10k_sbus_write(sb, FM10K_SW_SBUS_ADDR_SPICO,
			FM10K_SERDES_SPICO_REG_0X01, reg01);
	if (error)
		goto done;

	reg05 &= ~(1 << 0);
	error = fm10k_sbus_write(sb, FM10K_SW_SBUS_ADDR_SPICO,
			FM10K_SERDES_SPICO_REG_0X05, reg05);
	if (error)
		goto done;

done:
	return error;
}

static int
fm10k_swap_image_check_crc(struct fm10k_sbus *sb, unsigned int crc_code)
{
	int error;
	uint32_t crc;

	FM10K_SW_TRACE("sbus %s: performing SBM swap image CRC "
	    "check", sb->name);

	error =
	    fm10k_sbm_spico_int(sb, crc_code, 0, &crc);
	if (error)
		goto done;

	crc >>= 16;
	if (crc == 0xffff) {
		FM10K_SW_INFO("sbus %s: SBM swap image CRC check "
		    "failed (CRC interrupt 0x%02x returned 0xffff)",
		    sb->name, crc_code);
		error = -1;
		goto done;
	} else if (crc == 0x0001)
		FM10K_SW_TRACE("sbus %s: SBM swap image CRC "
		    "check passed ", sb->name);
	else
		FM10K_SW_INFO("sbus %s: unexpected SBM swap image "
		    "CRC check result 0x%04x", sb->name, crc);

done:
	return (error);
}

static int
fm10k_serdes_spico_upload_image(struct fm10k_sbus *sb,
		uint8_t dev, const uint16_t *image, unsigned int num_words)
{
	uint64_t start_time_us, elapsed_time_us;
	unsigned int i;
	uint32_t data;
	uint32_t reg07;
	int error;

	FM10K_SW_TRACE("sbus %s: uploading serdes SPICO image "
	    "to bus address 0x%02x (%u words)", sb->name, dev, num_words);

	reg07 = (1 << 0) | (1 << 4);
	error = fm10k_sbus_write(sb, dev, FM10K_SERDES_SPICO_REG_0X03, reg07);
	if (error)
		goto done;

	reg07 &= ~(1 << 0);
	error = fm10k_sbus_write(sb, dev, FM10K_SERDES_SPICO_REG_0X07, reg07);
	if (error)
		goto done;

	data = (1 << 30);
	error = fm10k_sbus_write(sb, dev, FM10K_SERDES_SPICO_REG_0X00, data);
	if (error)
		goto done;

	data = (1 << 4) | (1 << 5);
	error = fm10k_sbus_write(sb, dev, FM10K_SERDES_SPICO_REG_0X08, data);
	if (error)
		goto done;

	start_time_us = fm10k_uptime_us();
	for (i = 0; i < num_words; i += 3) {
		data = 0xc0000000 | image[i];
		if (i + 1 < num_words) {
			data |= (uint32_t)image[i + 1] << 10;

			if (i + 2 < num_words)
				data |= (uint32_t)image[i + 2] << 20;
		}
		error = fm10k_sbus_write(sb, dev,
				FM10K_SERDES_SPICO_REG_0X0A, data);
		if (error)
			goto done;
	}
	elapsed_time_us = fm10k_uptime_us() - start_time_us;
	FM10K_SW_TRACE("sbus %s: serdes SPICO image upload "
	    "time %u ms", sb->name, (uint32_t)(elapsed_time_us / 1000));

	error = fm10k_sbus_write(sb, dev, FM10K_SERDES_SPICO_REG_0X00, 0);
	if (error)
		goto done;

	error = fm10k_sbus_write(sb, dev,
			FM10K_SERDES_SPICO_REG_0X01, 0x20000000);
	if (error)
		goto done;

	data = (1 << 18) | (1 << 19);
	error = fm10k_sbus_write(sb, dev, FM10K_SERDES_SPICO_REG_0X0B, data);
	if (error)
		goto done;

	reg07 |= (1 << 0);
	error = fm10k_sbus_write(sb, dev, FM10K_SERDES_SPICO_REG_0X07, reg07);
	if (error)
		goto done;

	reg07 &= ~(1 << 0);
	error = fm10k_sbus_write(sb, dev, FM10K_SERDES_SPICO_REG_0X07, reg07);
	if (error)
		goto done;

	reg07 |= (1 << 1);
	error = fm10k_sbus_write(sb, dev, FM10K_SERDES_SPICO_REG_0X07, reg07);
	if (error)
		goto done;

	error = fm10k_sbus_write(sb, dev, FM10K_SERDES_SPICO_REG_0X08, 0);
	if (error)
		goto done;

done:
	return (error);
}

static int
fm10k_epl_serdes_check_crc_version_build_id(struct fm10k_switch *sw,
		uint32_t expected_version_build_id)
{
	struct fm10k_sbus *sb = sw->epl_sbus;
	unsigned int first = 0;
	unsigned int last = FM10K_SW_EPLS_MAX * FM10K_SW_EPL_LANES - 1;
	unsigned int i;
	int error;
	uint32_t data;
	uint32_t version_build_id;

	FM10K_SW_TRACE("sbus %s: checking SPICO code CRC and "
	    "version for serdes %u through %u", sb->name, first, last);

	for (i = first; i <= last; i++) {
		error = fm10k_sbus_read(sb,
				FM10K_SW_SBUS_ADDR_EPL_SERDES(i), 0xff, &data);
		if (error || data != 0x01) {
			FM10K_SW_TRACE("sbus %s: skipping "
			    "serdes %u (error=%d, data=0x%08x)", sb->name, i,
			    error, data);
			continue;
		}

		error = fm10k_epl_serdes_spico_do_crc(sw, i);
		if (error)
			goto done;

		error = fm10k_epl_serdes_get_version_build_id(sw, i,
		    &version_build_id);
		if (error)
			goto done;

		if (version_build_id != expected_version_build_id) {
			FM10K_SW_INFO("sbus %s: SERDES SPICO code "
			    "version compare failed (expected 0x%08x, got "
			    "0x%08x)", sb->name, expected_version_build_id,
			    version_build_id);
			error = -1;
			goto done;
		}
	}
	FM10K_SW_TRACE("sbus %s: serdes %u through %u are OK",
	    sb->name, first, last);

done:
	return (error);
}

static int
fm10k_epl_serdes_spico_do_crc(struct fm10k_switch *sw, unsigned int serdes)
{
	struct fm10k_sbus *sb = sw->epl_sbus;
	int error;
	uint32_t crc;

	FM10K_SW_TRACE("sbus %s: serdes %u: performing SPICO "
	    "CRC check", sb->name, serdes);

	error = fm10k_epl_serdes_spico_int(sw, serdes, 0x3c, 0, &crc);
	if (error)
		goto done;

	if (crc != 0) {
		FM10K_SW_INFO("sbus %s: serdes %u:  SPICO CRC check "
		    "failed (CRC interrupt returned 0x%08x)",
		    sb->name, serdes, crc);
		error = -1;
		goto done;
	}

done:
	return (error);
}

static int
fm10k_epl_serdes_spico_reset(struct fm10k_switch *sw, unsigned int serdes)
{
	struct fm10k_sbus *sb = sw->epl_sbus;
	int error;
	uint32_t data;
	uint8_t addr = FM10K_SW_SBUS_ADDR_EPL_SERDES(serdes);

	FM10K_SW_TRACE("sbus %s: serdes %u: resetting SPICO",
	    sb->name, serdes);

	fm10k_udelay(FM10K_SW_SERDES_RESET_DELAY_US);
	error = fm10k_sbus_write(sb, addr, FM10K_SERDES_SPICO_REG_0X00, 0x00);
	if (error)
		goto done;
	fm10k_udelay(FM10K_SW_SERDES_RESET_DELAY_US);

	data = (1 << 0) | (1 << 4);
	error = fm10k_sbus_write(sb, addr, FM10K_SERDES_SPICO_REG_0X07, data);
	fm10k_udelay(FM10K_SW_SERDES_RESET_DELAY_US);
	if (error)
		goto done;

	data = (1 << 18) | (1 << 19);
	error = fm10k_sbus_write(sb, addr, FM10K_SERDES_SPICO_REG_0X0B, data);
	fm10k_udelay(FM10K_SW_SERDES_RESET_DELAY_US);
	if (error)
		goto done;

	data = (1 << 4);
	error = fm10k_sbus_write(sb, addr, FM10K_SERDES_SPICO_REG_0X07, data);
	fm10k_udelay(FM10K_SW_SERDES_RESET_DELAY_US);
	if (error)
		goto done;

	data = (1 << 1);
	error = fm10k_sbus_write(sb, addr, FM10K_SERDES_SPICO_REG_0X07, data);
	fm10k_udelay(FM10K_SW_SERDES_RESET_DELAY_US);
	if (error)
		goto done;

	error = fm10k_sbus_write(sb, addr, FM10K_SERDES_SPICO_REG_0X08, 0);
	fm10k_udelay(FM10K_SW_SERDES_RESET_DELAY_US);
	if (error)
		goto done;

done:
	return (error);
}

static int
fm10k_epl_serdes_get_version_build_id(struct fm10k_switch *sw,
		unsigned int serdes, uint32_t *version_build_id)
{
	int error;
	uint32_t data;

	FM10K_SW_TRACE("sbus %s: serdes %u: getting code "
	    "version", sw->epl_sbus->name, serdes);

	error = fm10k_epl_serdes_spico_int(sw, serdes, 0x00, 0, &data);
	if (error)
		goto done;

	*version_build_id = data << 16;
	error = fm10k_epl_serdes_spico_int(sw, serdes, 0x3f, 0, &data);
	if (error)
		goto done;

	*version_build_id |= data & 0xffff;
done:
	return (error);
}

static int
fm10k_sbm_spico_int(struct fm10k_sbus *sb,
		uint8_t int_num, uint32_t param, uint32_t *result)
{
	uint64_t start_time_us, elapsed_time_ms;
	uint32_t data;
	int error;

	FM10K_SW_TRACE("sbus %s: SBM interrupt 0x%02x "
	    "(param=0x%08x)", sb->name, int_num, param);

	/* int write */
	error = fm10k_sbus_write(sb, FM10K_SW_SBUS_ADDR_SPICO,
			FM10K_SERDES_SPICO_REG_0X02,
			(param << 16) | int_num);
	if (error)
		goto done;

	error = fm10k_sbus_read(sb, FM10K_SW_SBUS_ADDR_SPICO,
			FM10K_SERDES_SPICO_REG_0X07, &data);
	if (error)
		goto done;

	data |= (1 << 0);
	error = fm10k_sbus_write(sb, FM10K_SW_SBUS_ADDR_SPICO,
			FM10K_SERDES_SPICO_REG_0X07, data);
	if (error)
		goto done;

	data &= ~(1 << 0);
	error = fm10k_sbus_write(sb, FM10K_SW_SBUS_ADDR_SPICO,
			FM10K_SERDES_SPICO_REG_0X07, data);
	if (error)
		goto done;

	/* int read */
	start_time_us = fm10k_uptime_us();
	elapsed_time_ms = 0;
	while (elapsed_time_ms < FM10K_SW_SBM_INTERRUPT_TIMEOUT_MS) {
		elapsed_time_ms = (fm10k_uptime_us() - start_time_us) / 1000;
		error = fm10k_sbus_read(sb,
				FM10K_SW_SBUS_ADDR_SPICO,
				FM10K_SERDES_SPICO_REG_0X08, &data);
		if (error)
			goto done;

		if ((data & 0x8000) || ((data & 0x3ff) == 0)) {
			if (elapsed_time_ms > 5)
				fm10k_udelay(5);
		} else {
			FM10K_SW_TRACE("sbus %s: SBM "
			    "interrupt 0x%02x (param=0x%08x, result=0x%08x)",
			    sb->name, int_num, param, data);
			*result = data;
			goto done;
		}
	}
	error = -1;
	FM10K_SW_INFO("sbus %s: SBM interrupt timed out after %u "
	    "ms", sb->name, (uint32_t)elapsed_time_ms);

done:
	return (error);
}

static int
fm10k_epl_serdes_spico_int(struct fm10k_switch *sw,
		unsigned int serdes, uint16_t int_num,
		uint32_t param, uint32_t *result)
{
	struct fm10k_sbus *sb = sw->epl_sbus;
	uint64_t start_time_us, elapsed_time_ms;
	unsigned int sbus_addr;
	int error;
	uint32_t data, intr;

	sbus_addr = FM10K_SW_SBUS_ADDR_EPL_SERDES(serdes);

	FM10K_SW_TRACE("sbus %s: serdes %u: (sbus addr 0x%02x) "
	    "interrupt 0x%04x (param=0x%08x)", sb->name, serdes, sbus_addr,
	    int_num, param);

	/* int write */
	start_time_us = fm10k_uptime_us();
	elapsed_time_ms = 0;
	while (elapsed_time_ms < FM10K_SW_SERDES_INTERRUPT_TIMEOUT_MS) {
		elapsed_time_ms = (fm10k_uptime_us() - start_time_us) / 1000;
		error = fm10k_sbus_read(sb, sbus_addr, 0x04, &data);
		if (error)
			goto done;

		if (data & ((1 << 16) | (1 << 17))) {
			if (elapsed_time_ms > 5)
				fm10k_udelay(5);
		} else {
			break;
		}
	}
	if (elapsed_time_ms >= FM10K_SW_SERDES_INTERRUPT_TIMEOUT_MS) {
		error = fm10k_sbus_read(sb, sbus_addr, 0x03, &intr);
		FM10K_SW_INFO("sbus %s: serdes %u: interrupt timed out "
		    "after %u ms (intr=0x%04x param=0x%04x error=%u "
		    "reg[4]=0x%08x)", sb->name, serdes,
		    (uint32_t)elapsed_time_ms, intr >> 16, intr & 0xffff, error,
		    data);
		error = -1;
		goto done;
	} else {
		error = fm10k_sbus_write(sb, sbus_addr,
				FM10K_SERDES_SPICO_REG_0X03,
				((uint32_t)int_num << 16) | param);
		if (error)
			goto done;
	}

	/* int read */
	start_time_us = fm10k_uptime_us();
	elapsed_time_ms = 0;
	while (elapsed_time_ms < FM10K_SW_SERDES_INTERRUPT_TIMEOUT_MS) {
		elapsed_time_ms = (fm10k_uptime_us() - start_time_us) / 1000;
		error = fm10k_sbus_read(sb, sbus_addr, 0x04, &data);
		if (error)
			goto done;

		if (data & ((1 << 16) | (1 << 17))) {
			if (elapsed_time_ms > 5)
				fm10k_udelay(5);
		} else {
			break;
		}
	}
	if (elapsed_time_ms >= FM10K_SW_SERDES_INTERRUPT_TIMEOUT_MS) {
		error = fm10k_sbus_read(sb, sbus_addr, 0x03, &intr);
		FM10K_SW_INFO("sbus %s: serdes %u: interrupt timed out "
		    "(2) after %u ms (intr=0x%04x param=0x%04x error=%u "
		    "reg[4]=0x%08x)", sb->name, serdes,
		    (uint32_t)elapsed_time_ms, intr >> 16, intr & 0xffff, error,
		    data);
		error = -1;
		goto done;
	} else {
		FM10K_SW_TRACE("sbus %s: serdes %u: interrupt "
		    "0x%04x (param=0x%08x, result=0x%08x)", sb->name, serdes,
		    int_num, param, data);
		if (result)
			*result = data;
	}

done:
	return (error);
}

int
fm10k_epl_serdes_start_bringup(struct fm10k_ext_port_lane *lane)
{
	struct fm10k_switch *sw = lane->port->ports->sw;
	unsigned int serdes = lane->abs_laneno;
	unsigned int speed = lane->port->lane_speed;
	unsigned int divider, width_mode;
	int error = 0;
	uint32_t data;

	/*
	 * Bit rate and width
	 */
	if (speed == 25) {
		/* 25G */
		divider = FM10K_SW_SERDES_DIVIDER_ETHMODE_25G;
		width_mode = FM10K_SW_SERDES_WIDTH_40;
	} else {
		/* 10G */
		divider = FM10K_SW_SERDES_DIVIDER_ETHMODE_10G;
		width_mode = FM10K_SW_SERDES_WIDTH_20;
	}

	/*
	 * state = CONFIGURED
	 */

	FM10K_SW_TRACE("sbus %s: serdes %u: configuring",
			sw->epl_sbus->name, serdes);

	error = fm10k_epl_serdes_spico_reset(sw, serdes);
	fm10k_udelay(FM10K_SW_SERDES_RESET_DELAY_US);
	if (error)
		goto done;

	error = fm10k_epl_serdes_spico_do_crc(sw, serdes);
	fm10k_udelay(FM10K_SW_SERDES_RESET_DELAY_US);
	if (error)
		goto done;

	fm10k_udelay(FM10K_SW_SERDES_CONFIG_DELAY_US);
	error = fm10k_epl_serdes_set_bit_rate(sw, serdes, divider);
	if (error)
		goto done;

	fm10k_epl_serdes_set_pcsl_width_mode(sw, serdes,
	    width_mode);

	/*
	 * Data select
	 */
	FM10K_SW_TRACE("sbus %s: serdes %u: setting TX data select",
	    sw->epl_sbus->name, serdes);
	error = fm10k_epl_serdes_spico_int02_retry(sw, serdes, 0x1ff,
	    FM10K_SW_SERDES_INT02_TIMEOUT_MS);
	if (error)
		goto done;

	error = fm10k_epl_serdes_dma_reg_write(sw, serdes, 0x21, 0x0c00);
	if (error)
		goto done;

	/*
	 * Configure TX equalization
	 *
	 * Use defaults - copper and optical are the same.
	 *
	 * DEFAULTS| precursor | cursor | postcursor |
	 * --------+-----------+--------+------------+
	 *  copper |     0     |    0   |     15     |
	 * --------+-----------+--------+------------+
	 *  optical|     0     |    0   |     15     |
	 * --------+-----------+--------+------------+
	 */
	FM10K_SW_TRACE("sbus %s: serdes %u: setting TX cursors",
	    sw->epl_sbus->name, serdes);
	error = fm10k_epl_serdes_set_tx_eq(sw, serdes,
	    FM10K_SW_SERDES_EQ_SEL_ATTEN, 0);
	if (error)
		goto done;
	error = fm10k_epl_serdes_set_tx_eq(sw, serdes,
	    FM10K_SW_SERDES_EQ_SEL_PRECUR, 0);
	if (error)
		goto done;
	error = fm10k_epl_serdes_set_tx_eq(sw, serdes,
	    FM10K_SW_SERDES_EQ_SEL_POSTCUR, 15);
	if (error)
		goto done;

	/*
	 * Configure 'options', which means:
	 *   - PLL calibration mode
	 *   - Phase slip
	 *   - RX termination
	 */
	FM10K_SW_TRACE("sbus %s: serdes %u: setting PLL calibration mode",
	    sw->epl_sbus->name, serdes);
	/* enable PLL calibration */
	data = (1 << 0) | (1 << 1);
	error =
	    fm10k_epl_serdes_spico_wr_only_int(sw, serdes, 0x11, data);
	if (error)
		goto done;
	/* There appears to be no action to take for phase slip */
	FM10K_SW_TRACE("sbus %s: serdes %u: configuring RX termination",
	    sw->epl_sbus->name, serdes);
	/* RX termination appears to default to AVDD */
	data = (1 << 0);
	error =
	    fm10k_epl_serdes_spico_wr_only_int(sw, serdes, 0x2b, data);
	if (error)
		goto done;

	/*
	 * Set TX and RX lane polarity.
	 * Assuming no inversion required.
	 */
	FM10K_SW_TRACE("sbus %s: serdes %u: configuring TX/RX lane polarity",
	    sw->epl_sbus->name, serdes);
	data = 0x0300;
	error = fm10k_epl_serdes_spico_int(sw, serdes, 0x13, data, NULL);
	if (error)
		goto done;

	/*
	 * Force local fault
	 */
	fm10k_epl_serdes_set_signal_detect(lane,
			FM10K_SW_LANE_OVERRIDE_FORCE_BAD);

	/*
	 * Enable TX/RX
	 */
	FM10K_SW_TRACE("sbus %s: serdes %u: enabling TX/RX",
	    sw->epl_sbus->name, serdes);
	fm10k_epl_serdes_set_signal_detect(lane,
			FM10K_SW_LANE_OVERRIDE_FORCE_BAD);
	error =
	    fm10k_epl_serdes_spico_wr_only_int(sw, serdes, 0x01, 0x03);
	if (error)
		goto done;

	/*
	 * Clear and enable interrupts
	 */
	FM10K_SW_TRACE("sbus %s: serdes %u: configuring interrupts",
	    sw->epl_sbus->name, serdes);
	error = fm10k_sbus_write(sw->epl_sbus,
			FM10K_SW_SBUS_ADDR_EPL_SERDES(serdes),
			FM10K_SERDES_SPICO_REG_0X08, 0);
	if (error)
		goto done;

done:
	return (error);
}

int
fm10k_epl_serdes_continue_bringup(struct fm10k_ext_port_lane *lane)
{
	struct fm10k_switch *sw = lane->port->ports->sw;
	unsigned int serdes = lane->abs_laneno;
	unsigned int speed = lane->port->lane_speed;
	unsigned int width_mode;
	int error = 0;
	int near_loopback = sw->serdes_loopback;

	if (speed == 25) {
		/* 25G */
		width_mode = FM10K_SW_SERDES_WIDTH_40;
	} else {
		/* 10G */
		width_mode = FM10K_SW_SERDES_WIDTH_20;
	}

	fm10k_udelay(FM10K_SW_SERDES_CONFIG_DELAY_US);
	error = fm10k_epl_serdes_set_pcsl_bit_slip(sw, serdes);
	if (error)
		goto done;

	fm10k_udelay(FM10K_SW_SERDES_CONFIG_DELAY_US);
	error = fm10k_epl_serdes_set_width_mode(sw, serdes, width_mode);
	if (error)
		goto done;

	FM10K_SW_TRACE("sbus %s: serdes %u: enabling TX output",
	    sw->epl_sbus->name, serdes);
	fm10k_udelay(FM10K_SW_SERDES_CONFIG_DELAY_US);
	error =
	    fm10k_epl_serdes_spico_wr_only_int(sw, serdes, 0x01, 0x07);
	if (error)
		goto done;

	fm10k_udelay(FM10K_SW_SERDES_CONFIG_DELAY_US);
	error = fm10k_epl_serdes_init_signal_ok(sw, serdes, 0);
	if (error)
		goto done;

	if (near_loopback) {
		/* enable near loopback */
		error = fm10k_epl_serdes_configure_near_loopback(lane);
		if (error)
			goto done;
	}

done:
	return (error);
}

int
fm10k_epl_serdes_disable(struct fm10k_ext_port_lane *lane)
{
	struct fm10k_switch *sw = lane->port->ports->sw;
	unsigned int serdes = lane->abs_laneno;
	int error = 0;

	fm10k_epl_serdes_set_signal_detect(lane,
			FM10K_SW_LANE_OVERRIDE_FORCE_BAD);

	/* is this delay necessary? in some places the IES SDK does this */
	fm10k_udelay(FM10K_SW_SERDES_RESET_DELAY_US);
	error = fm10k_epl_serdes_spico_reset(sw, serdes);
	fm10k_udelay(FM10K_SW_SERDES_RESET_DELAY_US);
	if (error)
		goto done;

done:
	return (error);
}

static int
fm10k_epl_serdes_set_bit_rate(struct fm10k_switch *sw,
		unsigned int serdes, unsigned int divider)
{
	int error;
	uint32_t data;

	FM10K_SW_TRACE("sbus %s: serdes %u: setting bit rate",
	    sw->epl_sbus->name, serdes);

	data = divider & 0x7ff;
	data |= (1 << 12);
	data |= (1 << 15);

	error = fm10k_epl_serdes_spico_wr_only_int(sw, serdes, 0x05, data);
	if (error)
		goto done;

done:
	return (error);
}

static int
fm10k_epl_serdes_set_width_mode(struct fm10k_switch *sw,
		unsigned int serdes, unsigned int width_mode)
{
	int error;
	uint32_t data;

	FM10K_SW_TRACE("sbus %s: serdes %u: setting width mode to %u",
	    sw->epl_sbus->name, serdes, width_mode);

	data = FM10K_SW_SERDES_WIDTH_20 | (FM10K_SW_SERDES_WIDTH_20 << 4);
	fm10k_udelay(FM10K_SW_SERDES_RESET_DELAY_US);
	error = fm10k_epl_serdes_spico_wr_only_int(sw, serdes, 0x14, data);
	if (error)
		goto done;

	data = width_mode | (width_mode << 4);
	fm10k_udelay(FM10K_SW_SERDES_RESET_DELAY_US);
	error = fm10k_epl_serdes_spico_wr_only_int(sw, serdes, 0x14, data);
	if (error)
		goto done;

done:
	fm10k_udelay(FM10K_SW_SERDES_RESET_DELAY_US);
	return error;
}

static int
fm10k_epl_serdes_set_tx_eq(struct fm10k_switch *sw,
		unsigned int serdes, unsigned int which, unsigned int tx_eq)
{
	int error;
	uint32_t data;

	FM10K_SW_TRACE("sbus %s: serdes %u: setting TX cursor %u to %u",
	    sw->epl_sbus->name, serdes, which, tx_eq);

	data = (tx_eq & 0xff) | (which << 14);
	error = fm10k_epl_serdes_spico_wr_only_int(sw, serdes, 0x15, data);
	if (error)
		goto done;

done:
	return error;
}

void
fm10k_epl_serdes_set_signal_detect(struct fm10k_ext_port_lane *lane,
		unsigned int override)
{
	struct fm10k_switch *sw = lane->port->ports->sw;
	unsigned int eplno = lane->port->eplno;
	uint32_t data;

	FM10K_SW_TRACE("sbus %s: serdes %u: setting signal detect to %s",
	    sw->epl_sbus->name, lane->abs_laneno,
	    (override == 0) ? "NORMAL" :
	    (override == 1) ? "FORCE_GOOD" :
	    (override == 2) ? "FORCE_BAD" : "<unknown>");

	FM10K_SW_SWITCH_LOCK(sw);
	data = fm10k_read_switch_reg(sw,
	    FM10K_SW_LANE_SIGNAL_DETECT_CFG(eplno, lane->rel_laneno));
	FM10K_SW_REPLACE_REG_FIELD(data,
	    LANE_SIGNAL_DETECT_CFG_SD_OVERRIDE,
	    override, data);
	fm10k_write_switch_reg(sw,
	    FM10K_SW_LANE_SIGNAL_DETECT_CFG(eplno, lane->rel_laneno), data);
	FM10K_SW_SWITCH_UNLOCK(sw);
}

static void
fm10k_epl_serdes_set_pcsl_width_mode(struct fm10k_switch *sw,
		unsigned int serdes, unsigned int width_mode)
{
	unsigned int eplno, lane;
	uint32_t pcsl_cfg;

	FM10K_SW_TRACE("sbus %s: serdes %u: setting pcsl width mode",
	    sw->epl_sbus->name, serdes);

	fm10k_epl_serdes_eplno_lane(sw, serdes, &eplno, &lane);

	FM10K_SW_SWITCH_LOCK(sw);
	pcsl_cfg = fm10k_read_switch_reg(sw, FM10K_SW_PCSL_CFG(eplno, lane));
	if (width_mode == FM10K_SW_SERDES_WIDTH_40) {
		pcsl_cfg &= ~FM10K_SW_PCSL_CFG_RX_GB_NARROW;
		pcsl_cfg &= ~FM10K_SW_PCSL_CFG_TX_GB_NARROW;
	} else {
		pcsl_cfg |= FM10K_SW_PCSL_CFG_RX_GB_NARROW;
		pcsl_cfg |= FM10K_SW_PCSL_CFG_TX_GB_NARROW;
	}
	pcsl_cfg &= ~FM10K_SW_PCSL_CFG_RX_BIT_SLIP_ENABLE;
	fm10k_write_switch_reg(sw, FM10K_SW_PCSL_CFG(eplno, lane), pcsl_cfg);
	FM10K_SW_SWITCH_UNLOCK(sw);
}

static int
fm10k_epl_serdes_set_pcsl_bit_slip(struct fm10k_switch *sw,
		unsigned int serdes)
{
	unsigned int eplno, lane;
	int error;
	uint32_t int_return, pcsl_cfg;

	FM10K_SW_TRACE("sbus %s: serdes %u: setting pcsl bit slip",
	    sw->epl_sbus->name, serdes);

	fm10k_epl_serdes_eplno_lane(sw, serdes, &eplno, &lane);
	error =
	    fm10k_epl_serdes_spico_int(sw, serdes, 0x0c, (1 << 7), &int_return);
	if (error)
		goto done;

	FM10K_SW_SWITCH_LOCK(sw);
	pcsl_cfg = fm10k_read_switch_reg(sw, FM10K_SW_PCSL_CFG(eplno, lane));
	pcsl_cfg |= FM10K_SW_PCSL_CFG_RX_BIT_SLIP_ENABLE;
	if (int_return & 0x1)
		pcsl_cfg |= FM10K_SW_PCSL_CFG_RX_BIT_SLIP_INITIAL;
	fm10k_write_switch_reg(sw, FM10K_SW_PCSL_CFG(eplno, lane), pcsl_cfg);
	FM10K_SW_SWITCH_UNLOCK(sw);

	error = fm10k_epl_serdes_spico_wr_only_int(sw, serdes, 0x0c, (1 << 8));
	if (error)
		goto done;

done:
	return (error);
}

static int
fm10k_epl_serdes_init_signal_ok(struct fm10k_switch *sw,
		unsigned int serdes, unsigned int threshold)
{
	int error;

	FM10K_SW_TRACE("sbus %s: serdes %u: setting signal OK threshold to %u",
	    sw->epl_sbus->name, serdes, threshold);

	error =
	    fm10k_epl_serdes_spico_int(sw, serdes, 0x20, 0x20, NULL);
	if (error)
		goto done;

	error = fm10k_epl_serdes_dma_esb_read_modify_write(sw, serdes, 0x080,
	    (threshold & 0xf) << 2, 0x3c, NULL);
	if (error)
		goto done;

done:
	return (error);
}

static int
fm10k_epl_serdes_spico_int02_retry(struct fm10k_switch *sw,
		unsigned int serdes, uint32_t data, unsigned int timeout_ms)
{
	uint64_t start_time_us, elapsed_time_ms;
	uint32_t result;
	int error;

	start_time_us = fm10k_uptime_us();
	elapsed_time_ms = 0;
	while (elapsed_time_ms < timeout_ms) {
		elapsed_time_ms = (fm10k_uptime_us() - start_time_us) / 1000;
		error =
		    fm10k_epl_serdes_spico_int(sw, serdes, 0x02, data, &result);
		if (result == 0x02) {
			error = 0;
			break;
		}

		fm10k_udelay(FM10K_SW_SERDES_CONFIG_DELAY_US);
	}

	return (error);
}

static int
fm10k_epl_serdes_dma_reg_write(struct fm10k_switch *sw,
		unsigned int serdes, unsigned int addr, uint32_t data)
{
	int error;

	error = fm10k_epl_serdes_spico_int(sw, serdes, 0x18,
	    0x8000 | (addr & 0xff), NULL);
	if (error)
		goto done;

	error = fm10k_epl_serdes_spico_int(sw, serdes, 0x19, data, NULL);
	if (error)
		goto done;

done:
	return (error);
}

static int
fm10k_epl_serdes_dma_esb_read(struct fm10k_switch *sw,
		unsigned int serdes, unsigned int addr, uint32_t *result)
{
	int error;

	error = fm10k_epl_serdes_spico_int(sw, serdes, 0x18,
	    0x4000 | (addr & 0x3fff), result);
	if (error)
		goto done;

	error = fm10k_epl_serdes_spico_int(sw, serdes, 0x1a, 0x00, result);
	if (error)
		goto done;

done:
	return (error);
}

static int
fm10k_epl_serdes_dma_esb_write(struct fm10k_switch *sw,
		unsigned int serdes, unsigned int addr, uint32_t data)
{
	int error;

	error = fm10k_epl_serdes_spico_int(sw, serdes, 0x18,
	    0x4000 | (addr & 0x3fff), NULL);
	if (error)
		goto done;

	error = fm10k_epl_serdes_spico_int(sw, serdes, 0x19, data, NULL);
	if (error)
		goto done;

done:
	return (error);
}

static int
fm10k_epl_serdes_dma_esb_read_modify_write(struct fm10k_switch *sw,
		unsigned int serdes, unsigned int addr,
		uint32_t data, uint32_t mask, uint32_t *result)
{
	int error;
	uint32_t read_data;

	error = fm10k_epl_serdes_dma_esb_read(sw, serdes, addr, &read_data);
	if (error)
		goto done;

	error = fm10k_epl_serdes_dma_esb_write(sw, serdes, addr,
	    (data & mask) | (read_data & ~mask));
	if (error)
		goto done;
	if (result)
		*result = (data & mask) | (read_data & ~mask);
done:
	return (error);
}

static int
fm10k_epl_serdes_spico_wr_only_int(struct fm10k_switch *sw,
		unsigned int serdes, unsigned int int_num, uint32_t param)
{
	int error;
	uint32_t result;

	error = fm10k_epl_serdes_spico_int(sw, serdes, int_num, param, &result);
	if (error)
		goto done;
	if (result != int_num) {
		error = -1;
		goto done;
	}
done:
	return (error);
}

static void
fm10k_epl_serdes_eplno_lane(struct fm10k_switch *sw,
		unsigned int serdes, unsigned int *eplno, unsigned int *lane)
{
	if (sw) {
		*eplno = serdes / FM10K_SW_EPL_LANES;
		*lane = serdes % FM10K_SW_EPL_LANES;
	}
}

static int
fm10k_epl_serdes_configure_near_loopback(struct fm10k_ext_port_lane *lane)
{
	struct fm10k_switch *sw = lane->port->ports->sw;
	unsigned int serdes = lane->abs_laneno;
	uint32_t data;
	int error;

	FM10K_SW_TRACE("sbus %s: serdes %u: configuring near loopback",
	    sw->epl_sbus->name, serdes);

	/* assuming no lane polarity has been set */
	/* set near loopback mode */
	fm10k_udelay(FM10K_SW_SERDES_CONFIG_DELAY_US);
	data = (1 << 8) | (1 << 0);
	error = fm10k_epl_serdes_spico_wr_only_int(sw, serdes, 0x08, data);
	if (error)
		goto done;

	/* disable tx output, leave tx/ex enabled */
	fm10k_udelay(FM10K_SW_SERDES_CONFIG_DELAY_US);
	error =
	    fm10k_epl_serdes_spico_wr_only_int(sw, serdes, 0x01, 0x03);
	if (error)
		goto done;

	/* set signal detect override to normal */
	fm10k_epl_serdes_set_signal_detect(lane, FM10K_SW_LANE_OVERRIDE_NORMAL);

done:
	return (error);
}

int
fm10k_epl_serdes_configure_for_dfe_tuning(struct fm10k_ext_port_lane *lane)
{
	struct fm10k_switch *sw = lane->port->ports->sw;
	unsigned int serdes = lane->abs_laneno;
	int error;

	FM10K_SW_TRACE("sbus %s: serdes %u: configuring serdes for dfe tuning",
	    sw->epl_sbus->name, serdes);

	/*
	 * Configure the serdes to run DFE tuning
	 */

	error = fm10k_epl_serdes_spico_int(sw, serdes, 0x26,
	    ((2 << 12) | (0 << 8) | (FM10K_SW_SERDES_DFE_DEFAULT_HF & 0xff)),
	    NULL);
	if (error)
		goto done;

	error = fm10k_epl_serdes_spico_int(sw, serdes, 0x26,
	    ((2 << 12) | (1 << 8) | (FM10K_SW_SERDES_DFE_DEFAULT_LF & 0xff)),
	    NULL);
	if (error)
		goto done;

	error = fm10k_epl_serdes_spico_int(sw, serdes, 0x26,
	    ((2 << 12) | (2 << 8) | (FM10K_SW_SERDES_DFE_DEFAULT_DC & 0xff)),
	    NULL);
	if (error)
		goto done;

	error = fm10k_epl_serdes_spico_int(sw, serdes, 0x26,
	    ((2 << 12) | (3 << 8) | (FM10K_SW_SERDES_DFE_DEFAULT_BW & 0xff)),
	    NULL);
	if (error)
		goto done;

	/* allow early link up */
	error = fm10k_epl_serdes_spico_int(sw, serdes, 0x26, 0x5b01, NULL);
	if (error)
		goto done;

done:
	return (error);
}

int
fm10k_epl_serdes_start_dfe_ical(struct fm10k_ext_port_lane *lane)
{
	struct fm10k_switch *sw = lane->port->ports->sw;
	unsigned int serdes = lane->abs_laneno;
	unsigned int speed = lane->port->lane_speed;
	int error;
	uint32_t param;

	/*
	 * Start DFE tuning initial calibration (ical)
	 */
	FM10K_SW_TRACE("sbus %s: serdes %u: starting dfe ical",
	    sw->epl_sbus->name, serdes);

	if (speed == 25)
		param = FM10K_SW_LINK_OPT_PARAM_A25G;
	else
		param = FM10K_SW_LINK_OPT_PARAM_A10G;

	if ((sw->epl_serdes_code_version_build_id >> 16) > 0x1055)
		error = fm10k_epl_serdes_config_dfe_param(sw, serdes, param);

	error = fm10k_epl_serdes_spico_wr_only_int(sw, serdes, 0x0a, 0x01);
	if (error)
		goto done;

done:
	return (error);
}

int
fm10k_epl_serdes_dfe_ical_status(struct fm10k_ext_port_lane *lane,
		uint32_t *status)
{
	struct fm10k_switch *sw = lane->port->ports->sw;
	unsigned int serdes = lane->abs_laneno;
	int error;
	uint32_t dfe_status, converged;

	error = fm10k_epl_serdes_get_dfe_status(sw, serdes, &dfe_status);
	if (error)
		goto done;

	if ((dfe_status & 0x11) == 0x11) {
		*status = FM10K_SW_SERDES_DFE_ICAL_IN_PROGRESS;
	} else {
		error = fm10k_epl_serdes_get_ical_result(sw,
				serdes, &converged);
		if (error)
			goto done;
		if (converged)
			*status = FM10K_SW_SERDES_DFE_ICAL_CONVERGED;
		else
			*status = FM10K_SW_SERDES_DFE_ICAL_FAILED;
	}

done:
	return (error);
}

int
fm10k_epl_serdes_start_dfe_pcal(struct fm10k_ext_port_lane *lane)
{
	struct fm10k_switch *sw = lane->port->ports->sw;
	unsigned int serdes = lane->abs_laneno;
	unsigned int speed = lane->port->lane_speed;
	int error;
	uint32_t param;

	/*
	 * Start DFE tuning periodic calibration (pcal), just one
	 */
	FM10K_SW_TRACE("sbus %s: serdes %u: starting dfe pcal",
	    sw->epl_sbus->name, serdes);
	if (speed == 25)
		param = FM10K_SW_LINK_OPT_PARAM_B25G;
	else
		param = FM10K_SW_LINK_OPT_PARAM_B10G;

	if ((sw->epl_serdes_code_version_build_id >> 16) > 0x1055)
		error = fm10k_epl_serdes_config_dfe_param(sw, serdes, param);

	error = fm10k_epl_serdes_spico_wr_only_int(sw, serdes, 0x0a, 0x02);
	if (error)
		goto done;

done:
	return (error);
}

int
fm10k_epl_serdes_dfe_pcal_status(struct fm10k_ext_port_lane *lane,
		uint32_t *status)
{
	struct fm10k_switch *sw = lane->port->ports->sw;
	unsigned int serdes = lane->abs_laneno;
	int error;
	uint32_t dfe_status;

	error = fm10k_epl_serdes_get_dfe_status(sw, serdes, &dfe_status);
	if (error)
		goto done;
	if (dfe_status & 0x02)
		*status = FM10K_SW_SERDES_DFE_PCAL_COMPLETE;
	else
		*status = FM10K_SW_SERDES_DFE_PCAL_IN_PROGRESS;

done:
	return (error);
}

int
fm10k_epl_serdes_stop_dfe_tuning(struct fm10k_ext_port_lane *lane,
		unsigned int force_reset)
{
	struct fm10k_switch *sw = lane->port->ports->sw;
	unsigned int serdes = lane->abs_laneno;
	int error;
	unsigned int ical_stopped, pcal_stopped;
	unsigned int i;
	uint32_t dfe_status, status;

	error = fm10k_epl_serdes_get_dfe_status(sw, serdes, &dfe_status);
	if (error)
		goto done;

	ical_stopped = 0;
	pcal_stopped = 0;

	if (!force_reset) {
		if (dfe_status & 0x3) {
			/* stop all DFE tuning */
			error = fm10k_epl_serdes_spico_wr_only_int(sw, serdes,
			    0x0a, 0);
			if (error)
				goto done;

			for (i = 0;
			     i < FM10K_SW_SERDES_ICAL_STOP_MAX_CYCLES;
			     i++) {
				error = fm10k_epl_serdes_dfe_ical_status(lane,
				    &status);
				if (error)
					goto done;
				if (status !=
				    FM10K_SW_SERDES_DFE_ICAL_IN_PROGRESS) {
					ical_stopped = 1;
					break;
				}
				fm10k_udelay
				(FM10K_SW_SERDES_DFE_STOP_CYCLE_DELAY_US);
			}

			if (ical_stopped)
				for (i = 0;
				     i < FM10K_SW_SERDES_PCAL_STOP_MAX_CYCLES;
				     i++) {
					error =
						fm10k_epl_serdes_dfe_pcal_status
						(lane, &status);
					if (error)
						goto done;
					if (status !=
					FM10K_SW_SERDES_DFE_PCAL_IN_PROGRESS) {
						pcal_stopped = 0;
						break;
					}
					fm10k_udelay
				(FM10K_SW_SERDES_DFE_STOP_CYCLE_DELAY_US);
				}
		}

		if (ical_stopped && pcal_stopped && (dfe_status & 0x40)) {
			error = fm10k_epl_serdes_start_dfe_pcal(lane);
			if (error)
				goto done;
			pcal_stopped = 0;
			for (i = 0;
				i < FM10K_SW_SERDES_PCAL_STOP_MAX_CYCLES; i++) {
				error = fm10k_epl_serdes_dfe_pcal_status(lane,
				    &status);
				if (error)
					goto done;
				if (status !=
				    FM10K_SW_SERDES_DFE_PCAL_IN_PROGRESS) {
					pcal_stopped = 1;
					break;
				}
				fm10k_udelay
				(FM10K_SW_SERDES_DFE_STOP_CYCLE_DELAY_US);
			}
		}
	}

	if (!ical_stopped || !pcal_stopped) {
		error = -1;
		goto done;
	}

done:
	return (error);
}

static int
fm10k_epl_serdes_config_dfe_param(struct fm10k_switch *sw,
		unsigned int serdes, uint32_t param)
{
	int error;

	error = fm10k_epl_serdes_spico_wr_only_int(sw, serdes, 0x18, 0x07);
	if (error)
		goto done;

	error = fm10k_epl_serdes_spico_wr_only_int(sw, serdes, 0x19,
	    param & 0xffff);
	if (error)
		goto done;

	error = fm10k_epl_serdes_spico_wr_only_int(sw, serdes, 0x19,
	    param >> 16);
	if (error)
		goto done;

done:
	return (error);
}

static int
fm10k_epl_serdes_get_dfe_status(struct fm10k_switch *sw,
		unsigned int serdes, uint32_t *status)
{
	int error;

	error = fm10k_epl_serdes_spico_int(sw, serdes, 0x126, (0x0b << 8),
	    status);
	if (error)
		goto done;

	FM10K_SW_TRACE("sbus %s: serdes %u: dfe status: "
	    "coarse_active=%u fine_active=%u adaptive=%u",
	    sw->epl_sbus->name, serdes, *status & (1 << 0),
	    *status & (1 << 1), *status & (1 << 6));

done:
	return (error);
}

static int
fm10k_epl_serdes_get_ical_result(struct fm10k_switch *sw,
		unsigned int serdes, uint32_t *converged)
{
	uint32_t val1;
	uint32_t val2;
	uint32_t diff;
	int error;

	*converged = 0;

	error = fm10k_epl_serdes_spico_int(sw, serdes, 0x126,
	    (4 << 12), &val1);
	if (error)
		goto done;

	error = fm10k_epl_serdes_spico_int(sw, serdes, 0x126,
	    (4 << 12) | (1 << 8), &val2);
	if (error)
		goto done;

	/* sign extension */
	if (val1 & 0x8000)
		val1 |= 0xffff0000;
	if (val2 & 0x8000)
		val2 |= 0xffff0000;
	diff = abs(val2 - val1);

	if (diff >= FM10K_SW_SERDES_DFE_DATA_LEVEL0_THRESHOLD)
		*converged = 1;

done:
	return (error);
}
