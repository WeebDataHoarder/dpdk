/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright 2019   Silicom Ltd. Connectivity Solutions
 */

#ifndef _FM10K_DEBUG_H_
#define _FM10K_DEBUG_H_

#define FM10K_SW_ERR(...)		PMD_INIT_LOG(ERR, __VA_ARGS__)
#define FM10K_SW_INFO(...)		PMD_INIT_LOG(INFO, __VA_ARGS__)
#define FM10K_SW_TRACE(...)		PMD_INIT_LOG(DEBUG, __VA_ARGS__)

#define FM10K_SW_ASSERT(...)		do {} while (0)

#define FM10K_SW_STATS_TRACE_ENABLE	1
#define FM10K_SW_FFU_CONF_TRACE_ENABLE	0
#define FM10K_SW_MIRROR_TRACE_ENABLE	0

#endif /* _FM10K_DEBUG_H_ */
