/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright 2019   Silicom Ltd. Connectivity Solutions
 */

#include <rte_malloc.h>
#include <rte_time.h>
#include <rte_kvargs.h>
#include <rte_hash.h>
#include <rte_flow.h>
#include <rte_flow_driver.h>
#include <rte_tm_driver.h>

#include "fm10k_debug.h"
#include "fm10k_regs.h"
#include "fm10k_sbus.h"
#include "fm10k_switch.h"

#define FM10K_SW_SBUS_COMMAND_WAIT_LOCK_US_MAX	4
#define FM10K_SW_SBUS_COMMAND_TIMEOUT_US	500000
#define FM10K_SW_SBUS_RING_DIVIDER		0x01

static int fm10k_sbus_init(struct fm10k_sbus *);
static int fm10k_sbus_exec(struct fm10k_sbus *, struct fm10k_sbus_req *);
static int fm10k_sbus_reset_all(struct fm10k_sbus *);
static int fm10k_sbus_sbm_reset(struct fm10k_sbus *);

struct fm10k_sbus *
fm10k_sbus_attach(struct fm10k_switch *sw,
		const char *name,
		unsigned int cfg_reg)
{
	struct fm10k_sbus *sb;
	int error;

	FM10K_SW_TRACE("sbus %s: attaching", name);

	sb = (struct fm10k_sbus *)rte_zmalloc
			("fm10k_sbus", sizeof(struct fm10k_sbus), 0);
	if (sb == NULL) {
		FM10K_SW_INFO("sbus %s: failed to allocate context", name);
		goto fail;
	}

	sb->sw = sw;
	sb->name = name;
	pthread_mutex_init(&sb->lock, NULL);

	sb->cfg_reg = cfg_reg;
	sb->cmd_reg = cfg_reg + FM10K_SW_REG_OFF(1);
	sb->req_reg = cfg_reg + FM10K_SW_REG_OFF(2);
	sb->resp_reg = cfg_reg + FM10K_SW_REG_OFF(3);

	error = fm10k_sbus_init(sb);
	if (error)
		goto fail;

	FM10K_SW_TRACE("sbus %s: attach successful", name);
	return sb;
fail:
	if (sb)
		fm10k_sbus_detach(sb);
	return NULL;
}

void
fm10k_sbus_detach(struct fm10k_sbus *sb)
{
	FM10K_SW_TRACE("sbus %s: detaching", sb->name);

	rte_free(sb);
}

static int
fm10k_sbus_init(struct fm10k_sbus *sb)
{
	uint32_t data;
	int error;

	error = fm10k_sbus_sbm_reset(sb);
	if (error)
		goto done;

	error = fm10k_sbus_reset_all(sb);
	if (error)
		goto done;

	/* set clock to REFCLK/2 */
	error = fm10k_sbus_write(sb, FM10K_SW_SBUS_ADDR_SBUS_CONTROLLER, 0x0a,
	    FM10K_SW_SBUS_RING_DIVIDER);
	if (error) {
		FM10K_SW_ERR("sbus %s: failed to set ring divider", sb->name);
		goto done;
	}

	error = fm10k_sbus_read(sb, FM10K_SW_SBUS_ADDR_SBUS_CONTROLLER, 0x0a,
	    &data);
	if (error) {
		FM10K_SW_ERR("sbus %s: failed to read back ring divider",
				sb->name);
		goto done;
	}
	if (data != FM10K_SW_SBUS_RING_DIVIDER) {
		FM10K_SW_ERR("sbus %s: ring divider "
				"verify failed (expected %u, got %u)",
				sb->name, FM10K_SW_SBUS_RING_DIVIDER, data);
		error = -1;
		goto done;
	}

done:
	return error;
}

static int
fm10k_sbus_exec(struct fm10k_sbus *sb, struct fm10k_sbus_req *req)
{
	struct fm10k_switch *sw =  sb->sw;
	unsigned int total_usecs;
	unsigned int drop_lock;
	unsigned int delay_time;
	int error = 0;
	uint32_t data;
	uint8_t expected_result;

	FM10K_SW_SBUS_LOCK(sb);

	FM10K_SW_SWITCH_LOCK(sw);
	data = fm10k_read_switch_reg(sw, sb->cmd_reg);
	if (data & FM10K_SW_SBUS_COMMAND_BUSY) {
		FM10K_SW_INFO("sbus %s: bus busy (0x%08x)",
		    sb->name, data);
		error = -1;
		goto done;
	}

	switch (req->op) {
	case FM10K_SW_SBUS_OP_RESET:
		FM10K_SW_TRACE("sbus %s: RESET dev=0x%02x reg=0x%02x data=0x%08x",
		    sb->name, req->dev, req->reg, req->data);
		expected_result = FM10K_SW_SBUS_RESULT_RESET;
		fm10k_write_switch_reg(sw, sb->req_reg, req->data);
		break;
	case FM10K_SW_SBUS_OP_WRITE:
		FM10K_SW_TRACE("sbus %s: WRITE dev=0x%02x reg=0x%02x data=0x%08x",
		    sb->name, req->dev, req->reg, req->data);
		expected_result = FM10K_SW_SBUS_RESULT_WRITE;
		fm10k_write_switch_reg(sw, sb->req_reg, req->data);
		break;
	case FM10K_SW_SBUS_OP_READ:
		FM10K_SW_TRACE("sbus %s: READ dev=0x%02x reg=0x%02x",
		    sb->name, req->dev, req->reg);
		expected_result = FM10K_SW_SBUS_RESULT_READ;
		req->data = 0;
		break;
	default:
		FM10K_SW_INFO("sbus %s: invalid opcode 0x%02x",
		    sb->name, req->op);
		error = -1;
		goto done;
	}

	/* Clear the execute bit */
	fm10k_write_switch_reg(sw, sb->cmd_reg, 0);

	data =
	    FM10K_SW_MAKE_REG_FIELD(SBUS_COMMAND_OP, req->op) |
	    FM10K_SW_MAKE_REG_FIELD(SBUS_COMMAND_ADDRESS, req->dev) |
	    FM10K_SW_MAKE_REG_FIELD(SBUS_COMMAND_REGISTER, req->reg) |
	    FM10K_SW_SBUS_COMMAND_EXECUTE;
	fm10k_write_switch_reg(sw, sb->cmd_reg, data);
	fm10k_write_flush(sw);

	total_usecs = 0;
	delay_time = 1;
	drop_lock = 0;
	do {
		if (drop_lock)
			FM10K_SW_SWITCH_UNLOCK(sw);
		fm10k_udelay(delay_time);
		if (drop_lock)
			FM10K_SW_SWITCH_LOCK(sw);
		total_usecs += delay_time;
		if (total_usecs >= FM10K_SW_SBUS_COMMAND_WAIT_LOCK_US_MAX) {
			drop_lock = 1;
			delay_time <<= 1;
		}

		data = fm10k_read_switch_reg(sw, sb->cmd_reg);
		if (!(data & FM10K_SW_SBUS_COMMAND_BUSY)) {
			if (FM10K_SW_REG_FIELD(data,
					SBUS_COMMAND_RESULT_CODE)
					!= expected_result) {
				FM10K_SW_INFO("sbus %s: expected "
				    "result code %u, got %u", sb->name,
				    expected_result,
				    FM10K_SW_REG_FIELD(data,
					SBUS_COMMAND_RESULT_CODE));
				error = -1;
				goto done;
			}
			if (req->op == FM10K_SW_SBUS_OP_READ) {
				req->data =
				    fm10k_read_switch_reg(sw, sb->resp_reg);
				FM10K_SW_TRACE("sbus %s: READ data=0x%02x",
						sb->name, req->data);
			}
			goto done;
		}

	} while (total_usecs < FM10K_SW_SBUS_COMMAND_TIMEOUT_US);

	error = -1;
	FM10K_SW_INFO("sbus %s: command timed out after %u us "
	    "(op=0x%02x dev=0x%02x reg=0x%02x data=0x%08x)", sb->name,
	    total_usecs, req->op, req->dev, req->reg, req->data);

done:
	FM10K_SW_SWITCH_UNLOCK(sw);
	FM10K_SW_SBUS_UNLOCK(sb);
	return error;
}

static int
fm10k_sbus_reset_all(struct fm10k_sbus *sb)
{
	struct fm10k_sbus_req req;
	int error;

	req.op = FM10K_SW_SBUS_OP_RESET;
	req.dev = FM10K_SW_SBUS_ADDR_BROADCAST;
	req.reg = 0;
	req.data = 0;

	error = fm10k_sbus_exec(sb, &req);

	FM10K_SW_TRACE("sbus %s: broadcast reset %s (%d)", sb->name,
	    error ? "failed" : "succeeded", error);

	return error;
}

static int
fm10k_sbus_sbm_reset(struct fm10k_sbus *sb)
{
	struct fm10k_sbus_req req;
	int error;

	req.op = FM10K_SW_SBUS_OP_RESET;
	req.dev = FM10K_SW_SBUS_ADDR_SPICO;
	req.reg = 0;
	req.data = 0;

	error = fm10k_sbus_exec(sb, &req);

	FM10K_SW_TRACE("sbus %s: SBM reset %s (%d)", sb->name,
	    error ? "failed" : "succeeded", error);

	return error;
}

int
fm10k_sbus_read(struct fm10k_sbus *sb, uint8_t dev, uint8_t reg, uint32_t *data)
{
	struct fm10k_sbus_req req;
	int error;

	req.op = FM10K_SW_SBUS_OP_READ;
	req.dev = dev;
	req.reg = reg;

	error = fm10k_sbus_exec(sb, &req);
	*data = error ? 0 : req.data;

	return error;
}

int
fm10k_sbus_write(struct fm10k_sbus *sb, uint8_t dev, uint8_t reg, uint32_t data)
{
	struct fm10k_sbus_req req;
	int error;

	req.op = FM10K_SW_SBUS_OP_WRITE;
	req.dev = dev;
	req.reg = reg;
	req.data = data;

	error = fm10k_sbus_exec(sb, &req);

	return error;
}
