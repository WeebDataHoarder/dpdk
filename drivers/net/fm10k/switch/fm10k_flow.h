/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright 2019   Silicom Ltd. Connectivity Solutions
 */

#ifndef _FM10K_SW_FLOW_H_
#define _FM10K_SW_FLOW_H_

#include <rte_time.h>
#include <rte_kvargs.h>
#include <rte_hash.h>
#include <rte_flow.h>
#include <rte_flow_driver.h>
#include <rte_tm_driver.h>

/*
 * Struct to store flow created.
 */
struct rte_flow {
	TAILQ_ENTRY(rte_flow) node;
	enum rte_filter_type filter_type;
	void *rule;
};

TAILQ_HEAD(fm10k_flow_list, rte_flow);

#endif /* _FM10K_SW_FLOW_H_ */
