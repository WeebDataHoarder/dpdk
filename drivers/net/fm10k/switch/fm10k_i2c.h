/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright 2019   Silicom Ltd. Connectivity Solutions
 */

#ifndef _FM10K_SW_I2C_H_
#define _FM10K_SW_I2C_H_

#include <semaphore.h>
#include <pthread.h>

#include "rte_spinlock.h"
#include "fm10k_debug.h"

#define FM10K_SW_I2C_MSG_MAX	12

struct fm10k_i2c_req {
	uint8_t addr; /* 7-bit address */
	uint8_t cmd;  /* FM10K_SW_I2C_COMMAND_* */
	uint8_t write_len;
	uint8_t read_len;
	uint8_t status; /* FM10K_SW_I2C_COMPLETION_ */
	uint8_t msg[FM10K_SW_I2C_MSG_MAX];
};

struct fm10k_i2c {
	struct fm10k_switch *sw;
	pthread_mutex_t bus_lock;
	pthread_mutex_t req_lock;
	sem_t req_cv;
	struct fm10k_i2c_req *cur_req;
};

#define FM10K_SW_I2C_LOCK(i2c_)	\
	pthread_mutex_lock(&(i2c_)->bus_lock)
#define FM10K_SW_I2C_UNLOCK(i2c_)	\
	pthread_mutex_unlock(&(i2c_)->bus_lock)

#define FM10K_SW_I2C_REQ_LOCK(i2c_)	\
			pthread_mutex_lock(&((i2c_)->req_lock))
#define FM10K_SW_I2C_REQ_UNLOCK(i2c_) \
			pthread_mutex_unlock(&((i2c_)->req_lock))

struct fm10k_i2c *fm10k_i2c_attach(struct fm10k_switch *sw);
void fm10k_i2c_detach(struct fm10k_i2c *i2c);
unsigned int fm10k_i2c_intr(struct fm10k_i2c *i2c);
int fm10k_i2c_exec(struct fm10k_i2c *i2c, struct fm10k_i2c_req *req);
int fm10k_i2c_read8(struct fm10k_i2c *i2c, uint8_t addr, uint8_t *result);
int fm10k_i2c_read8_ext(struct fm10k_i2c *i2c,
				uint8_t addr, uint8_t reg, uint8_t *result);
int fm10k_i2c_write8(struct fm10k_i2c *i2c, uint8_t addr, uint8_t data);
int fm10k_i2c_write16(struct fm10k_i2c *i2c,
				uint8_t addr, uint8_t data0, uint8_t data1);
int fm10k_i2c_probe(struct fm10k_i2c *i2c, uint8_t addr);

#endif /* _FM10K_SW_I2C_H_ */
