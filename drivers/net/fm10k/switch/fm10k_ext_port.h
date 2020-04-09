/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright 2019   Silicom Ltd. Connectivity Solutions
 */

#ifndef _FM10K_SW_EXT_PORT_H_
#define _FM10K_SW_EXT_PORT_H_

#include "fm10k_regs.h"
#include "fm10k_sm.h"
#include "fm10k_sbus.h"
#include "fm10k_switch.h"

struct fm10k_sbus;
struct fm10k_switch;

#define FM10K_SW_TRACE_ENABLE 0

enum fm10k_ext_port_lane_state {
	FM10K_SW_EXT_PORT_LANE_STATE_DOWN,
	FM10K_SW_EXT_PORT_LANE_STATE_WAIT_PLL_LOCK,
	FM10K_SW_EXT_PORT_LANE_STATE_WAIT_SIGNAL_OK,
	FM10K_SW_EXT_PORT_LANE_STATE_WAIT_DFE_ICAL,
	FM10K_SW_EXT_PORT_LANE_STATE_WAIT_DFE_PCAL,
	FM10K_SW_EXT_PORT_LANE_STATE_UP
};

enum fm10k_ext_port_lane_event {
	FM10K_SW_EXT_PORT_LANE_EVENT_BRING_UP,
	FM10K_SW_EXT_PORT_LANE_EVENT_RX_PLL_LOCK,
	FM10K_SW_EXT_PORT_LANE_EVENT_TX_PLL_LOCK,
	FM10K_SW_EXT_PORT_LANE_EVENT_SIGNAL_OK,
	FM10K_SW_EXT_PORT_LANE_EVENT_SIGNAL_NOK,
	FM10K_SW_EXT_PORT_LANE_EVENT_DFE_TUNING_COMPLETE,
	FM10K_SW_EXT_PORT_LANE_EVENT_DFE_TUNING_FAILED,
	FM10K_SW_EXT_PORT_LANE_EVENT_DFE_TUNING_TIMED_OUT,
	FM10K_SW_EXT_PORT_LANE_EVENT_RESTORE_IM,
	FM10K_SW_EXT_PORT_LANE_EVENT_BRING_DOWN,

	FM10K_SW_EXT_PORT_LANE_NUM_EVENTS
};

enum fm10k_ext_port_lane_timer {
	FM10K_SW_EXT_PORT_LANE_TIMER_PLL_LOCK,
	FM10K_SW_EXT_PORT_LANE_TIMER_SIGNAL_OK,
	FM10K_SW_EXT_PORT_LANE_TIMER_DFE_TUNING,

	FM10K_SW_EXT_PORT_LANE_NUM_TIMERS
};

#define FM10K_SW_EXT_PORT_LANE_PLL_LOCK_TIMEOUT_MS	2000
#define FM10K_SW_EXT_PORT_LANE_SIGNAL_OK_TIMEOUT_MS	2000
#define FM10K_SW_EXT_PORT_LANE_DFE_ICAL_POLL_MS		200
#define FM10K_SW_EXT_PORT_LANE_DFE_ICAL_TIMEOUT_MS	3000
#define FM10K_SW_EXT_PORT_LANE_DFE_PCAL_POLL_MS		100
#define FM10K_SW_EXT_PORT_LANE_DFE_PCAL_TIMEOUT_MS	2000

enum fm10k_ext_port_state {
	FM10K_SW_EXT_PORT_STATE_DOWN,
	FM10K_SW_EXT_PORT_STATE_WAITING_FOR_LANES,
	FM10K_SW_EXT_PORT_STATE_UP
};

enum fm10k_ext_port_event {
	FM10K_SW_EXT_PORT_EVENT_BRING_UP,
	FM10K_SW_EXT_PORT_EVENT_BRING_DOWN,
	FM10K_SW_EXT_PORT_EVENT_LANE_UP,
	FM10K_SW_EXT_PORT_EVENT_LANE_DOWN,

	FM10K_SW_EXT_PORT_NUM_EVENTS
};

struct fm10k_ext_port;

struct fm10k_ext_port_lane {
	struct fm10k_ext_port *port;
	struct fm10k_sm *sm;
	uint32_t im;
	uint16_t dfe_poll_time_ms;
	uint8_t abs_laneno;
	uint8_t rel_laneno;
	uint8_t flags;
};

#define FM10K_SW_EXT_PORT_LANE_FLAG_RX_LOCKED		0x01
#define FM10K_SW_EXT_PORT_LANE_FLAG_TX_LOCKED		0x02

#define FM10K_SW_EXT_PORT_LANE_FLAGS_PLLS_LOCKED	\
		(FM10K_SW_EXT_PORT_LANE_FLAG_RX_LOCKED | \
		FM10K_SW_EXT_PORT_LANE_FLAG_TX_LOCKED)

struct fm10k_ext_ports;

struct fm10k_ext_port {
	struct fm10k_ext_ports *ports;
	struct fm10k_sm *sm;
	struct fm10k_ext_port_lane lanes[FM10K_SW_EPL_LANES];
	char name[10];
	uint8_t portno;
	uint8_t eplno;
	uint8_t lport;		/* filled in by switch */
	uint8_t first_lane; /* range is [0..3] */
	uint8_t num_lanes;
	uint8_t is_quad;
	uint8_t lane_speed;
	uint8_t last_led_flags;
	uint32_t an_im;
	uint32_t link_im;
};

#define FM10K_SW_EXT_PORT_LED_FLAG_UP		0x01
#define FM10K_SW_EXT_PORT_LED_FLAG_ACTIVE	0x02

struct fm10k_ext_ports {
	struct fm10k_switch *sw;
	struct fm10k_ext_port *ports;
	uint16_t epl_mask;
	uint8_t num_ports;
	uint8_t ports_per_epl;
};

#define FM10K_SW_EXT_PORTS_EPL_USED(p_, n_)	((p_)->epl_mask & (1 << (n_)))

struct fm10k_ext_ports *fm10k_ext_ports_attach(struct fm10k_switch *sw);
void fm10k_ext_ports_detach(struct fm10k_ext_ports *ports);
unsigned int fm10k_ext_ports_epl_intrs(struct fm10k_ext_ports *ports,
		uint64_t gid);
void fm10k_ext_port_up(struct fm10k_ext_port *port);
void fm10k_ext_port_down(struct fm10k_ext_port *port);
unsigned int fm10k_ext_port_isup(struct fm10k_ext_port *port);
void fm10k_ext_port_eplidx_lane(struct fm10k_ext_ports *ports,
		unsigned int ext_port_idx, unsigned int *eplidx,
		unsigned int *lane);

#endif /* _FM10K_SW_EXT_PORT_H_ */
