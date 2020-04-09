/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright 2019   Silicom Ltd. Connectivity Solutions
 */

#ifndef _FM10K_SW_SBUS_H_
#define _FM10K_SW_SBUS_H_

#include <pthread.h>

struct fm10k_sbus {
	struct fm10k_switch *sw;
	const char *name;
	pthread_mutex_t lock;
	unsigned int cfg_reg;
	unsigned int cmd_reg;
	unsigned int req_reg;
	unsigned int resp_reg;
};

#define FM10K_SW_SBUS_LOCK(sb_)		pthread_mutex_lock(&((sb_)->lock))
#define FM10K_SW_SBUS_UNLOCK(sb_)	pthread_mutex_unlock(&((sb_)->lock))

struct fm10k_sbus_req {
	uint8_t op;
	uint8_t dev;
	uint8_t reg;
	uint32_t data;
};

struct fm10k_sbus *fm10k_sbus_attach(struct fm10k_switch *sw,
		const char *name, unsigned int cfg_reg);
void fm10k_sbus_detach(struct fm10k_sbus *sb);
int fm10k_sbus_read(struct fm10k_sbus *sb,
		uint8_t dev, uint8_t reg, uint32_t *data);
int fm10k_sbus_write(struct fm10k_sbus *sb,
		uint8_t dev, uint8_t reg, uint32_t data);

#endif /* _FM10K_SW_SBUS_H_ */
