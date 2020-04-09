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
#include "fm10k_i2c.h"
#include "fm10k_regs.h"
#include "fm10k_switch.h"

static void fm10k_i2c_init(struct fm10k_i2c *);

struct fm10k_i2c *
fm10k_i2c_attach(struct fm10k_switch *sw)
{
	struct fm10k_i2c *i2c;

	FM10K_SW_TRACE("i2c: attaching");

	i2c = (struct fm10k_i2c *)rte_zmalloc("fm10k_i2c",
			sizeof(struct fm10k_i2c), 0);
	if (i2c == NULL) {
		FM10K_SW_INFO("i2c: failed to allocate context");
		goto fail;
	}

	i2c->sw = sw;
	pthread_mutex_init(&i2c->req_lock, NULL);
	pthread_mutex_init(&i2c->bus_lock, NULL);
	sem_init(&i2c->req_cv, 0, 0);

	fm10k_i2c_init(i2c);

	FM10K_SW_TRACE("i2c: attach successful");
	return i2c;
fail:
	if (i2c)
		fm10k_i2c_detach(i2c);
	return NULL;
}

void
fm10k_i2c_detach(struct fm10k_i2c *i2c)
{
	FM10K_SW_TRACE("i2c: detaching");

	rte_free(i2c);
}

static void
fm10k_i2c_init(struct fm10k_i2c *i2c)
{
	struct fm10k_switch *sw = i2c->sw;
	struct fm10k_device_info *cfg = sw->info;
	uint32_t freq = FM10K_SW_I2C_CFG_DIVIDER_400_KHZ;
	uint32_t data;

	if (FM10K_SW_CARD_ID(cfg->subvendor, cfg->subdevice) ==
			FM10K_SW_CARD(SILICOM, PE3100G2DQIRM_QXSL4))
		freq = FM10K_SW_I2C_CFG_DIVIDER_100_KHZ;

	/* clear any pending interrupt */
	fm10k_write_switch_reg(sw, FM10K_SW_I2C_CTRL,
		FM10K_SW_I2C_CTRL_INTERRUPT_PENDING);

	/* 400 KHz, master mode, unmask interrupt */
	data = fm10k_read_switch_reg(sw, FM10K_SW_I2C_CFG);
	data &= ~FM10K_SW_I2C_CFG_SLAVE_ENABLE;
	if (FM10K_SW_CARD_ID(cfg->subvendor, cfg->subdevice) ==
			FM10K_SW_CARD(SILICOM, PE3100G2DQIRM_QXSL4))
		FM10K_SW_REPLACE_REG_FIELD(data, I2C_CFG_DIVIDER, freq, data);
	data &=  ~FM10K_SW_I2C_CFG_INTERRUPT_MASK;
	fm10k_write_switch_reg(sw, FM10K_SW_I2C_CFG, data);

	if (FM10K_SW_CARD_ID(cfg->subvendor, cfg->subdevice) ==
			FM10K_SW_CARD(SILICOM, PE3100G2DQIRM_QXSL4))
		/* reset I2C */
		fm10k_gpio_output_set(sw, 5, 1);
}

unsigned int
fm10k_i2c_intr(struct fm10k_i2c *i2c)
{
	struct fm10k_switch *sw = i2c->sw;
	struct fm10k_i2c_req *req;
	int i;
	uint32_t data[3];
	uint32_t ctrl;

	req = i2c->cur_req;

	FM10K_SW_SWITCH_LOCK(sw);
	ctrl = fm10k_read_switch_reg(sw, FM10K_SW_I2C_CTRL);
	fm10k_write_switch_reg(sw, FM10K_SW_I2C_CTRL,
	    FM10K_SW_I2C_CTRL_INTERRUPT_PENDING);

	req->status = FM10K_SW_REG_FIELD(ctrl, I2C_CTRL_COMMAND_COMPLETED);

	if ((req->cmd == FM10K_SW_I2C_COMMAND_RD ||
			req->cmd == FM10K_SW_I2C_COMMAND_WR_RD) &&
			req->status == FM10K_SW_I2C_COMPLETION_NORMAL) {
		for (i = 0; i < FM10K_SW_HOWMANY(req->read_len, 4, 4); i++)
			data[i] = fm10k_read_switch_reg
						(sw, FM10K_SW_I2C_DATA(i));

		for (i = 0; i < req->read_len; i++)
			req->msg[i] =
				(data[i / 4] >> (24 - (i % 4) * 8)) & 0xff;
	}
	FM10K_SW_SWITCH_UNLOCK(sw);
	sem_post(&i2c->req_cv);

	return 1;
}

int
fm10k_i2c_exec(struct fm10k_i2c *i2c, struct fm10k_i2c_req *req)
{
	struct fm10k_switch *sw = i2c->sw;
	int i;
	uint32_t ctrl;
	uint32_t data[3];

	if (((req->cmd == FM10K_SW_I2C_COMMAND_WR ||
		    req->cmd == FM10K_SW_I2C_COMMAND_WR_RD) &&
		req->write_len > FM10K_SW_I2C_MSG_MAX) ||
	    ((req->cmd == FM10K_SW_I2C_COMMAND_RD ||
		req->cmd == FM10K_SW_I2C_COMMAND_WR_RD) &&
		((req->read_len == 0  ||
		    req->read_len > FM10K_SW_I2C_MSG_MAX))))
		return (-1);

	FM10K_SW_TRACE("i2c: initiating command %u", req->cmd);

	ctrl =
	    FM10K_SW_MAKE_REG_FIELD(I2C_CTRL_ADDR, req->addr << 1) |
	    FM10K_SW_MAKE_REG_FIELD(I2C_CTRL_COMMAND, req->cmd);

	if (req->cmd == FM10K_SW_I2C_COMMAND_WR ||
			req->cmd == FM10K_SW_I2C_COMMAND_WR_RD) {
		ctrl |= FM10K_SW_MAKE_REG_FIELD
				(I2C_CTRL_LENGTH_W, req->write_len);

		data[0] = 0;
		data[1] = 0;
		data[2] = 0;

		for (i = 0; i < req->write_len; i++)
			data[i / 4] |= req->msg[i] << (24 - (i % 4) * 8);

		for (i = 0; i < FM10K_SW_HOWMANY(req->write_len, 4, 4); i++)
			fm10k_write_switch_reg(sw,
					FM10K_SW_I2C_DATA(i), data[i]);
	}

	if (req->cmd == FM10K_SW_I2C_COMMAND_RD ||
	    req->cmd == FM10K_SW_I2C_COMMAND_WR_RD)
		ctrl |= FM10K_SW_MAKE_REG_FIELD
				(I2C_CTRL_LENGTH_R, req->read_len);

	req->status = FM10K_SW_I2C_COMPLETION_RUNNING;
	i2c->cur_req = req;

	FM10K_SW_SWITCH_LOCK(sw);
	/* zero command field */
	fm10k_write_switch_reg(sw, FM10K_SW_I2C_CTRL, 0);
	/* initiate command */
	fm10k_write_switch_reg(sw, FM10K_SW_I2C_CTRL, ctrl);
	FM10K_SW_SWITCH_UNLOCK(sw);

	while (req->status == FM10K_SW_I2C_COMPLETION_RUNNING)
		sem_wait(&i2c->req_cv);

	return 0;
}

int
fm10k_i2c_read8(struct fm10k_i2c *i2c, uint8_t addr, uint8_t *result)
{
	struct fm10k_i2c_req req;
	int error;

	req.addr = addr;
	req.cmd = FM10K_SW_I2C_COMMAND_RD;
	req.read_len = 1;
	req.msg[0] = 0;

	error = fm10k_i2c_exec(i2c, &req);
	if (error)
		goto done;

	if (req.status != FM10K_SW_I2C_COMPLETION_NORMAL) {
		FM10K_SW_INFO("i2c read failed (%u)", req.status);
		error = -1;
		goto done;
	}

	*result = req.msg[0];

done:
	return (error);
}

int
fm10k_i2c_read8_ext(struct fm10k_i2c *i2c,
		uint8_t addr, uint8_t reg, uint8_t *result)
{
	struct fm10k_i2c_req req;
	int error;

	req.addr = addr;
	req.cmd = FM10K_SW_I2C_COMMAND_WR_RD;
	req.write_len = 1;
	req.read_len = 1;
	req.msg[0] = reg;

	error = fm10k_i2c_exec(i2c, &req);
	if (error)
		goto done;

	if (req.status != FM10K_SW_I2C_COMPLETION_NORMAL) {
		FM10K_SW_INFO("i2c read failed (%u)", req.status);
		error = -1;
		goto done;
	}

	*result = req.msg[0];
done:
	return (error);
}

int
fm10k_i2c_write8(struct fm10k_i2c *i2c, uint8_t addr, uint8_t data)
{
	struct fm10k_i2c_req req;
	int error;

	req.addr = addr;
	req.cmd = FM10K_SW_I2C_COMMAND_WR;
	req.write_len = 1;
	req.msg[0] = data;

	error = fm10k_i2c_exec(i2c, &req);
	if (error)
		goto done;

	if (req.status != FM10K_SW_I2C_COMPLETION_NORMAL) {
		error = -1;
		goto done;
	}

done:
	return (error);
}

int
fm10k_i2c_write16(struct fm10k_i2c *i2c,
		uint8_t addr, uint8_t data0, uint8_t data1)
{
	struct fm10k_i2c_req req;
	int error;

	req.addr = addr;
	req.cmd = FM10K_SW_I2C_COMMAND_WR;
	req.write_len = 2;
	req.msg[0] = data0;
	req.msg[1] = data1;

	error = fm10k_i2c_exec(i2c, &req);
	if (error)
		goto done;

	if (req.status != FM10K_SW_I2C_COMPLETION_NORMAL) {
		error = -1;
		goto done;
	}
done:
	return (error);
}

int
fm10k_i2c_probe(struct fm10k_i2c *i2c, uint8_t addr)
{
	struct fm10k_i2c_req req;
	int error;

	req.addr = addr;
	req.cmd = FM10K_SW_I2C_COMMAND_WR;
	req.write_len = 0;

	error = fm10k_i2c_exec(i2c, &req);
	if (error)
		goto done;

	if (req.status != FM10K_SW_I2C_COMPLETION_NORMAL) {
		error = -1;
		goto done;
	}

done:
	return (error);
}
