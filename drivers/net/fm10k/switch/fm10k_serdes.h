/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright 2019   Silicom Ltd. Connectivity Solutions
 */

#ifndef _FM10K_SW_SERDES_H_
#define _FM10K_SW_SERDES_H_

#define FM10K_SW_SERDES_DFE_ICAL_IN_PROGRESS		0
#define FM10K_SW_SERDES_DFE_ICAL_CONVERGED		1
#define FM10K_SW_SERDES_DFE_ICAL_FAILED			2
#define FM10K_SW_SERDES_DFE_PCAL_IN_PROGRESS		0
#define FM10K_SW_SERDES_DFE_PCAL_COMPLETE		1

int fm10k_epl_serdes_reset_and_load_all(struct fm10k_switch *sw);
int fm10k_epl_serdes_start_bringup(struct fm10k_ext_port_lane *lane);
int fm10k_epl_serdes_continue_bringup(struct fm10k_ext_port_lane *lane);
int fm10k_epl_serdes_disable(struct fm10k_ext_port_lane *lane);
int fm10k_epl_serdes_configure_for_dfe_tuning(struct fm10k_ext_port_lane *lane);
int fm10k_epl_serdes_start_dfe_ical(struct fm10k_ext_port_lane *lane);
int fm10k_epl_serdes_dfe_ical_status
		(struct fm10k_ext_port_lane *lane, uint32_t *status);
int fm10k_epl_serdes_start_dfe_pcal(struct fm10k_ext_port_lane *lane);
int fm10k_epl_serdes_dfe_pcal_status
		(struct fm10k_ext_port_lane *lane, uint32_t *status);
int fm10k_epl_serdes_stop_dfe_tuning
		(struct fm10k_ext_port_lane *lane, unsigned int force_reset);
void fm10k_epl_serdes_set_signal_detect
		(struct fm10k_ext_port_lane *lane, unsigned int override);

#endif /* _FM10K_SW_SERDES_H_ */
