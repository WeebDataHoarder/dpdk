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
#include "fm10k_ext_port.h"
#include "fm10k_i2c.h"
#include "fm10k_regs.h"
#include "fm10k_serdes.h"
#include "fm10k_sm.h"
#include "fm10k_switch.h"

#define LANE_EVENT(e_)			FM10K_SW_EXT_PORT_LANE_EVENT_##e_
#define LANE_TIMER(t_)			FM10K_SW_EXT_PORT_LANE_TIMER_##t_
#define LANE_TIMEOUT(t_)		FM10K_SW_EXT_PORT_LANE_##t_##_TIMEOUT_MS
#define LANE_POLL(t_)			FM10K_SW_EXT_PORT_LANE_##t_##_POLL_MS
#define LANE_STATE(s_)			FM10K_SW_EXT_PORT_LANE_STATE_##s_
#define LANE_FLAG(f_)			FM10K_SW_EXT_PORT_LANE_FLAG_##f_
#define LANE_FLAGS(f_)			FM10K_SW_EXT_PORT_LANE_FLAGS_##f_
#define LANE_FLAG_SET(l_, f_)		((l_)->flags & LANE_FLAG(f_))
#define LANE_FLAGS_SET(l_, f_, f1_)	\
	(((l_)->flags & LANE_FLAGS(f_)) == LANE_FLAGS(f1_))

#define PORT_EVENT(e_)			FM10K_SW_EXT_PORT_EVENT_##e_
#define PORT_STATE(s_)			FM10K_SW_EXT_PORT_STATE_##s_

static void fm10k_ext_ports_epl_intr
		(struct fm10k_ext_ports *ports, unsigned int eplno);
static void fm10k_ext_port_intr(struct fm10k_ext_port *port, uint32_t epl_ip);
static void fm10k_ext_port_process_event(struct fm10k_sm *sm, uint16_t event);
static void fm10k_ext_port_bring_down(struct fm10k_ext_port *port);
static void fm10k_ext_port_lane_process_event
		(struct fm10k_sm *sm, uint16_t event);
static void fm10k_ext_port_lane_process_timer
		(struct fm10k_sm *sm, uint16_t timer);
static void fm10k_ext_port_restart_lane(struct fm10k_ext_port_lane *lane);
static void fm10k_ext_port_disable_lane(struct fm10k_ext_port_lane *lane);
static void fm10k_ext_port_phy_enable(struct fm10k_ext_port *port);
static void fm10k_ext_port_phy_disable(struct fm10k_ext_port *port);

struct fm10k_ext_ports *
fm10k_ext_ports_attach(struct fm10k_switch *sw)
{
	struct fm10k_ext_ports *ports;
	struct fm10k_ext_port *port;
	struct fm10k_ext_port_lane *lane;
	struct fm10k_device_info *cfg = sw->info;
	unsigned int is_quad, lane_speed;
	unsigned int i, j;
	unsigned int eplidx, laneno;

	FM10K_SW_TRACE("attaching external ports\n");

	ports = (struct fm10k_ext_ports *)rte_zmalloc
			("fm10k_ext_ports", sizeof(struct fm10k_ext_ports), 0);
	if (ports == NULL) {
		FM10K_SW_ERR("failed to allocate external ports\n");
		goto fail;
	}

	ports->ports = (struct fm10k_ext_port *)rte_zmalloc
			("fm10k_ext_port_list",
			sizeof(struct fm10k_ext_port) * cfg->num_ext_ports, 0);
	if (ports->ports == NULL) {
		FM10K_SW_ERR("failed to allocate external ports list\n");
		goto fail;
	}
	ports->sw = sw;
	ports->num_ports = cfg->num_ext_ports;
	ports->ports_per_epl = cfg->num_ext_ports / cfg->num_epls;
	ports->epl_mask = 0;

	switch (cfg->ext_port_speed) {
	default:
	case 10:
		is_quad = 0;
		lane_speed = 10;
		break;
	case 25:
		is_quad = 0;
		lane_speed = 25;
		break;
	case 40:
		is_quad = 1;
		lane_speed = 10;
		break;
	case 100:
		is_quad = 1;
		lane_speed = 25;
		break;
	}

	for (i = 0; i < ports->num_ports; i++) {
		port = &ports->ports[i];
		port->ports = ports;
		fm10k_ext_port_eplidx_lane(ports, i, &eplidx, &laneno);
		port->portno = i;
		port->eplno = eplidx ? sw->eplb_no : sw->epla_no;
		snprintf(port->name, sizeof(port->name), "EPL[%u][%u]",
		    port->eplno, port->first_lane);
		port->first_lane = laneno;
		port->is_quad = is_quad;
		port->num_lanes = port->is_quad ? 4 : 1;
		port->lane_speed = lane_speed;
		port->sm = fm10k_sm_attach
			(FM10K_SW_EXT_PORT_NUM_EVENTS, 0,
		    fm10k_ext_port_process_event, NULL,
		    (void *)port);
		if (port->sm == NULL)
			goto fail;
		port->sm->portno = port->portno;
		port->sm->laneno = 0;
		port->an_im = FM10K_SW_AN_IM_ALL;
		port->link_im = FM10K_SW_LINK_IM_ALL;

		for (j = port->first_lane;
		     j < port->first_lane + port->num_lanes; j++) {
			lane = &port->lanes[j];
			lane->port = port;
			lane->abs_laneno = port->eplno * FM10K_SW_EPL_LANES + j;
			lane->rel_laneno = j;
			lane->im = FM10K_SW_SERDES_IM_ALL;
			lane->sm = fm10k_sm_attach
				(FM10K_SW_EXT_PORT_LANE_NUM_EVENTS,
			    FM10K_SW_EXT_PORT_LANE_NUM_TIMERS,
			    fm10k_ext_port_lane_process_event,
			    fm10k_ext_port_lane_process_timer,
			    (void *)lane);
			if (lane->sm == NULL)
				goto fail;
			lane->sm->portno = lane->port->portno;
			lane->sm->laneno = lane->rel_laneno;
		}

		ports->epl_mask |= (1 << port->eplno);
	}

	return (ports);
fail:
	if (ports)
		fm10k_ext_ports_detach(ports);
	return NULL;
}

void
fm10k_ext_ports_detach(struct fm10k_ext_ports *ports)
{
	struct fm10k_ext_port *port;
	struct fm10k_ext_port_lane *lane;
	unsigned int i, j;

	FM10K_SW_TRACE("detaching external ports\n");

	if (ports->ports) {
		for (i = 0; i < ports->num_ports; i++) {
			port = &ports->ports[i];

			if (port->sm)
				fm10k_sm_detach(port->sm);
			for (j = port->first_lane;
			     j < port->first_lane + port->num_lanes; j++) {
				lane = &port->lanes[j];
				if (lane->sm)
					fm10k_sm_detach(lane->sm);
			}

			fm10k_ext_port_phy_disable(port);
		}
		rte_free(ports->ports);
	}

	rte_free(ports);
}

unsigned int
fm10k_ext_ports_epl_intrs(struct fm10k_ext_ports *ports, uint64_t gid)
{
	unsigned int i;
	uint16_t epl_mask;

	/*
	 * Process EPL interrupts for all EPLs that are in use and have
	 * their bit set in global interrupt detect.
	 */
	epl_mask = ports->epl_mask &
	    FM10K_SW_REG_FIELD64(gid, GLOBAL_INTERRUPT_DETECT_EPL);
	for (i = 0; i < FM10K_SW_EPLS_MAX; i++)
		if (epl_mask & (1 << i))
			fm10k_ext_ports_epl_intr(ports, i);

	return ports->epl_mask;
}

static void
fm10k_ext_ports_epl_intr(struct fm10k_ext_ports *ports, unsigned int eplno)
{
	struct fm10k_switch *sw = ports->sw;
	struct fm10k_ext_port *port;
	uint64_t gid;
	unsigned int i, j;
	uint32_t epl_ip;

	/*
	 * The global interrupt detect architecture makes it somewhat
	 * challenging to ensure events are not lost and spurious interrupts
	 * are not generated.
	 *
	 * The EPLs have an internal interrupt hierarchy whose reporting is
	 * rooted at EPL_IP.  Whenever an EPL transitions from having no
	 * unmasked interrupts pending to at least one unmasked interrupt
	 * pending or from having at least one unmasked interrupt pending to
	 * no unmasked interrupts pending, it sends a vector to the
	 * interrupt controller over a serial bus indicating the
	 * at-least-one/none pending state, and the controller updates the
	 * bit for that EPL in the global interrupt detect register
	 * accordingly.
	 *
	 * Because of the update-vector-in-flight potential, there is a race
	 * between re-enabling the EPL PCIe interrupt mask after servicing
	 * the EPL interrupts and the all-clear vector from the EPL reaching
	 * the interrupt controller - it's possible for the software to
	 * clear all the EPL-internal interrupt sources during servicing and
	 * then re-enable the EPL interrupt in the PCIe mask before the
	 * all-clear vector reaches the interrupt controller and clears the
	 * EPL pending bit in the global interrupt detect register.  This
	 * would result in a spurious EPL interrupt, potentially for every
	 * necessary EPL interrupt.
	 *
	 * The approach taken here to avoid spurious interrupts is to first
	 * mask off all EPL-internal sources, then poll the EPL bits in the
	 * global interrupt detect register until they clear (indicating the
	 * update vector from the EPL has reached the interrupt controller).
	 * Then software interrupt processing proceeds, and the EPL-internal
	 * masks are restored to their desired state at the end.  This
	 * ensures that when the EPL PCIe interrupt mask is re-enabled, the
	 * EPL pending bits in the global interrupt detect register will
	 * only be set if there are new, unprocessed EPL events of interest.
	 *
	 */
	FM10K_SW_SWITCH_LOCK(sw);
	/* Grab epl_ip before it is cleared by the masks below */
	epl_ip = fm10k_read_switch_reg(sw, FM10K_SW_EPL_IP(eplno));
	for (i = 0; i < ports->num_ports; i++) {
		port = &ports->ports[i];
		if (port->eplno != eplno)
			continue;

		fm10k_write_switch_reg(sw,
		    FM10K_SW_AN_IM(eplno, port->first_lane),
		    FM10K_SW_AN_IM_ALL);
		fm10k_write_switch_reg(sw,
		    FM10K_SW_LINK_IM(eplno, port->first_lane),
		    FM10K_SW_LINK_IM_ALL);

		for (j = port->first_lane;
		     j < port->first_lane + port->num_lanes; j++) {
			fm10k_write_switch_reg(sw,
			    FM10K_SW_SERDES_IM(eplno, j),
			    FM10K_SW_SERDES_IM_ALL);
		}
	}

	/*
	 * Wait for the all-clear vector to propagate over the internal
	 * interrupt serial bus to the global interrupt detect.
	 */
	do {
		gid = fm10k_read_switch_reg64
				(sw, FM10K_SW_GLOBAL_INTERRUPT_DETECT);
	} while (gid & FM10K_SW_GLOBAL_INTERRUPT_DETECT_EPL(eplno));
	FM10K_SW_SWITCH_UNLOCK(sw);

	/*
	 * Proceed with interrupt processing per-external-port.
	 */
	for (i = 0; i < ports->num_ports; i++) {
		port = &ports->ports[i];
		if (port->eplno != eplno)
			continue;
		fm10k_ext_port_intr(port, epl_ip);
	}
}

static void
fm10k_ext_port_intr(struct fm10k_ext_port *port, uint32_t epl_ip)
{
	struct fm10k_switch *sw = port->ports->sw;
	struct fm10k_ext_port_lane *lane;
	unsigned int i;
	uint32_t serdes_ip, data = 0;

	for (i = port->first_lane;
	     i < port->first_lane + port->num_lanes; i++) {
		lane = &port->lanes[i];
		if (epl_ip & FM10K_SW_EPL_IP_SERDES_INTERRUPT(i)) {
			FM10K_SW_SWITCH_LOCK(sw);
			serdes_ip = fm10k_read_switch_reg(sw,
			    FM10K_SW_SERDES_IP(port->eplno, i));
			serdes_ip &= ~lane->im;
			fm10k_write_switch_reg(sw,
			    FM10K_SW_SERDES_IP(port->eplno, i), serdes_ip);
			if (serdes_ip & FM10K_SW_SERDES_IP_RX_SIGNAL_OK)
				data =
					fm10k_read_switch_reg(sw,
					FM10K_SW_LANE_SERDES_STATUS(port->eplno,
					i));
			FM10K_SW_SWITCH_UNLOCK(sw);

			if (serdes_ip & FM10K_SW_SERDES_IP_RX_SIGNAL_OK) {
				fm10k_sm_send_event(lane->sm,
				    (data &
				FM10K_SW_LANE_SERDES_STATUS_RX_SIGNAL_OK) ?
				LANE_EVENT(SIGNAL_OK) : LANE_EVENT(SIGNAL_NOK));
			}

			if (serdes_ip & FM10K_SW_SERDES_IP_RX_RDY)
				fm10k_sm_send_event(lane->sm,
				    LANE_EVENT(RX_PLL_LOCK));

			if (serdes_ip & FM10K_SW_SERDES_IP_TX_RDY)
				fm10k_sm_send_event(lane->sm,
				    LANE_EVENT(TX_PLL_LOCK));
		} else {
			fm10k_sm_send_event(lane->sm,
			    LANE_EVENT(RESTORE_IM));
		}
	}
}

void
fm10k_ext_port_up(struct fm10k_ext_port *port)
{
	fm10k_sm_send_event(port->sm, PORT_EVENT(BRING_UP));
}

void
fm10k_ext_port_down(struct fm10k_ext_port *port)
{
	fm10k_sm_send_event(port->sm, PORT_EVENT(BRING_DOWN));
}

unsigned int
fm10k_ext_port_isup(struct fm10k_ext_port *port)
{
	/* no locked needed as we are just inspecting
	 * the current value of an int
	 */
	unsigned int port_state = port->sm->state;

	return (port_state == FM10K_SW_EXT_PORT_STATE_UP);
}

static void
fm10k_ext_port_process_event(struct fm10k_sm *sm, uint16_t event)
{
	struct fm10k_ext_port *port = sm->ctx;
	struct fm10k_ext_port_lane *lane;
	unsigned int i;
	unsigned int lanes_up;

	switch (sm->state) {
	case PORT_STATE(DOWN):
		if (event == PORT_EVENT(BRING_UP)) {
			fm10k_ext_port_phy_enable(port);

			for (i = port->first_lane;
			     i < port->first_lane + port->num_lanes; i++) {
				lane = &port->lanes[i];
				fm10k_sm_send_event
					(lane->sm, LANE_EVENT(BRING_UP));
			}
			sm->state = PORT_STATE(WAITING_FOR_LANES);
		}
		break;

	case PORT_STATE(WAITING_FOR_LANES):
		if (event == PORT_EVENT(LANE_UP)) {
			lanes_up = 0;
			for (i = port->first_lane;
			     i < port->first_lane + port->num_lanes; i++) {
				lane = &port->lanes[i];
				if (lane->sm->state != LANE_STATE(UP))
					break;
				lanes_up++;
			}
			if (lanes_up == port->num_lanes) {
				sm->state = PORT_STATE(UP);
				FM10K_SW_INFO
				("Port %d is LINK UP", port->portno);
			}
		} else if (event == PORT_EVENT(BRING_DOWN)) {
			fm10k_ext_port_bring_down(port);
		}
		break;

	case PORT_STATE(UP):
		if (event == PORT_EVENT(LANE_DOWN)) {
			sm->state = PORT_STATE(WAITING_FOR_LANES);
			FM10K_SW_INFO("Port %d is LINK DOWN", port->portno);
		} else if (event == PORT_EVENT(BRING_DOWN)) {
			fm10k_ext_port_bring_down(port);
			FM10K_SW_INFO("Port %d is LINK DOWN", port->portno);
		}
	}
}

static void
fm10k_ext_port_bring_down(struct fm10k_ext_port *port)
{
	struct fm10k_sm *sm = port->sm;
	struct fm10k_ext_port_lane *lane;
	unsigned int i;

	sm->state = PORT_STATE(DOWN);

	fm10k_ext_port_phy_disable(port);

	for (i = port->first_lane;
	     i < port->first_lane + port->num_lanes; i++) {
		lane = &port->lanes[i];
		fm10k_sm_send_event(lane->sm, LANE_EVENT(BRING_DOWN));
	}
}

static void
fm10k_ext_port_lane_process_event(struct fm10k_sm *sm, uint16_t event)
{
	struct fm10k_ext_port_lane *lane = sm->ctx;
	struct fm10k_ext_port *port = lane->port;
	struct fm10k_switch *sw = lane->port->ports->sw;
	int error;

	switch (sm->state) {
	case LANE_STATE(DOWN):
		if (event == LANE_EVENT(BRING_UP)) {
			lane->flags &= ~LANE_FLAGS(PLLS_LOCKED);

			/*
			 * Clear all pending interrupts.
			 */
			FM10K_SW_SWITCH_LOCK(sw);
			fm10k_write_switch_reg(sw,
			    FM10K_SW_SERDES_IP(port->eplno, lane->rel_laneno),
			    FM10K_SW_SERDES_IP_ALL);
			FM10K_SW_SWITCH_UNLOCK(sw);

			error = fm10k_epl_serdes_start_bringup(lane);
			if (error == 0) {
				sm->state = LANE_STATE(WAIT_PLL_LOCK);

				/*
				 * Enable PLL lock interrupts.
				 */
				lane->im &= ~(FM10K_SW_SERDES_IM_RX_RDY |
				    FM10K_SW_SERDES_IM_TX_RDY);
			}
		}
		break;

	case LANE_STATE(WAIT_PLL_LOCK):
		if (event == LANE_EVENT(RX_PLL_LOCK) ||
		    event == LANE_EVENT(TX_PLL_LOCK)) {
			if (event == LANE_EVENT(RX_PLL_LOCK))
				lane->flags |= LANE_FLAG(RX_LOCKED);
			if (event == LANE_EVENT(TX_PLL_LOCK))
				lane->flags |= LANE_FLAG(TX_LOCKED);
			if (LANE_FLAGS_SET(lane, PLLS_LOCKED, PLLS_LOCKED)) {
				error = fm10k_epl_serdes_continue_bringup(lane);
				if (error == 0) {
					sm->state = LANE_STATE(WAIT_SIGNAL_OK);
					/* unmask RX signal OK interrupt */
					lane->im |=
						(FM10K_SW_SERDES_IM_RX_RDY |
					    FM10K_SW_SERDES_IM_TX_RDY);
					lane->im &=
					~FM10K_SW_SERDES_IM_RX_SIGNAL_OK;
				}
			}
		} else if (event == LANE_EVENT(BRING_DOWN)) {
			fm10k_ext_port_disable_lane(lane);
		}
		break;

	case LANE_STATE(WAIT_SIGNAL_OK):
		if (event == LANE_EVENT(SIGNAL_OK)) {
			sm->state = LANE_STATE(WAIT_DFE_ICAL);

			fm10k_epl_serdes_configure_for_dfe_tuning(lane);
			fm10k_epl_serdes_start_dfe_ical(lane);

			lane->dfe_poll_time_ms = 0;
			fm10k_sm_timer_start(sm, LANE_TIMER(DFE_TUNING),
			    LANE_POLL(DFE_ICAL));
		} else if (event == LANE_EVENT(BRING_DOWN)) {
			fm10k_ext_port_disable_lane(lane);
		}
		break;

	case LANE_STATE(WAIT_DFE_ICAL):
		if (event == LANE_EVENT(DFE_TUNING_COMPLETE)) {
			sm->state = LANE_STATE(WAIT_DFE_PCAL);

			fm10k_epl_serdes_start_dfe_pcal(lane);

			lane->dfe_poll_time_ms = 0;
			fm10k_sm_timer_start(sm, LANE_TIMER(DFE_TUNING),
			    LANE_POLL(DFE_PCAL));
		} else if ((event == LANE_EVENT(DFE_TUNING_FAILED)) ||
		    (event == LANE_EVENT(DFE_TUNING_TIMED_OUT))) {
			fm10k_ext_port_restart_lane(lane);
		}  else if (event == LANE_EVENT(BRING_DOWN)) {
			fm10k_ext_port_disable_lane(lane);
		}
		break;

	case LANE_STATE(WAIT_DFE_PCAL):
		if (event == LANE_EVENT(DFE_TUNING_COMPLETE)) {
			fm10k_epl_serdes_set_signal_detect(lane,
			    FM10K_SW_LANE_OVERRIDE_NORMAL);

			sm->state = LANE_STATE(UP);
			fm10k_sm_send_event(port->sm, PORT_EVENT(LANE_UP));
		} else if ((event == LANE_EVENT(DFE_TUNING_FAILED)) ||
		    (event == LANE_EVENT(DFE_TUNING_TIMED_OUT)) ||
		    (event == LANE_EVENT(SIGNAL_NOK))) {
			fm10k_ext_port_restart_lane(lane);
		}  else if (event == LANE_EVENT(BRING_DOWN)) {
			fm10k_ext_port_disable_lane(lane);
		}
		break;

	case LANE_STATE(UP):
		if (event == LANE_EVENT(SIGNAL_NOK))
			fm10k_ext_port_restart_lane(lane);
		else if (event == LANE_EVENT(BRING_DOWN))
			fm10k_ext_port_disable_lane(lane);
		break;
	}

	/* Always restore the interrupt mask */
	FM10K_SW_SWITCH_LOCK(sw);
	fm10k_write_switch_reg(sw,
	    FM10K_SW_SERDES_IM(port->eplno,
		lane->rel_laneno),
	    lane->im);
	FM10K_SW_SWITCH_UNLOCK(sw);
}

static void
fm10k_ext_port_lane_process_timer(struct fm10k_sm *sm, uint16_t timer)
{
	struct fm10k_ext_port_lane *lane = sm->ctx;
	uint32_t status;

	switch (sm->state) {
	case LANE_STATE(WAIT_DFE_ICAL):
		if (timer == LANE_TIMER(DFE_TUNING)) {
			lane->dfe_poll_time_ms += LANE_POLL(DFE_ICAL);
			fm10k_epl_serdes_dfe_ical_status(lane, &status);

			if (status == FM10K_SW_SERDES_DFE_ICAL_IN_PROGRESS) {
				if (lane->dfe_poll_time_ms <
				    LANE_TIMEOUT(DFE_ICAL))
					fm10k_sm_timer_start(sm,
					    LANE_TIMER(DFE_TUNING),
					    LANE_POLL(DFE_ICAL));
				else
					fm10k_sm_send_event(sm,
					    LANE_EVENT(DFE_TUNING_TIMED_OUT));
			} else if (status == FM10K_SW_SERDES_DFE_ICAL_CONVERGED)
				fm10k_sm_send_event(sm,
				    LANE_EVENT(DFE_TUNING_COMPLETE));
			else
				fm10k_sm_send_event(sm,
				    LANE_EVENT(DFE_TUNING_FAILED));
		}
		break;

	case LANE_STATE(WAIT_DFE_PCAL):
		if (timer == LANE_TIMER(DFE_TUNING)) {
			lane->dfe_poll_time_ms += LANE_POLL(DFE_PCAL);

			fm10k_epl_serdes_dfe_pcal_status(lane, &status);
			if (status == FM10K_SW_SERDES_DFE_PCAL_IN_PROGRESS) {
				if (lane->dfe_poll_time_ms <
				    LANE_TIMEOUT(DFE_PCAL))
					fm10k_sm_timer_start(sm,
					    LANE_TIMER(DFE_TUNING),
					    LANE_POLL(DFE_PCAL));
				else
					fm10k_sm_send_event(sm,
					    LANE_EVENT(DFE_TUNING_TIMED_OUT));
			} else {
				fm10k_sm_send_event(sm,
				    LANE_EVENT(DFE_TUNING_COMPLETE));
			}
		}
		break;
	}
}

static void
fm10k_ext_port_restart_lane(struct fm10k_ext_port_lane *lane)
{
	struct fm10k_sm *sm = lane->sm;
	struct fm10k_ext_port *port = lane->port;
	int error;

	error = fm10k_epl_serdes_start_bringup(lane);
	if (error == 0) {
		sm->state = LANE_STATE(WAIT_PLL_LOCK);

		/*
		 * Enable PLL lock interrupts.
		 */
		lane->flags &= ~FM10K_SW_EXT_PORT_LANE_FLAGS_PLLS_LOCKED;
		lane->im |= FM10K_SW_SERDES_IM_RX_SIGNAL_OK;
		lane->im = ~(FM10K_SW_SERDES_IM_RX_RDY |
		    FM10K_SW_SERDES_IM_TX_RDY);
	}

	fm10k_sm_send_event(port->sm, PORT_EVENT(LANE_DOWN));
}

static void
fm10k_ext_port_disable_lane(struct fm10k_ext_port_lane *lane)
{
	struct fm10k_sm *sm = lane->sm;

	fm10k_sm_timer_cancel(sm, LANE_TIMER(DFE_TUNING));
	fm10k_epl_serdes_disable(lane);

	/*
	 * Disable all interrupts.
	 */
	lane->im = FM10K_SW_SERDES_IM_ALL;

	sm->state = LANE_STATE(DOWN);
}

void
fm10k_ext_port_eplidx_lane(struct fm10k_ext_ports *ports,
		unsigned int ext_port_idx, unsigned int *eplidx,
		unsigned int *lane)
{
	FM10K_SW_ASSERT(ext_port_idx < ports->num_ports,
	    ("ext_port_idx out of range"));

	*eplidx = ext_port_idx / ports->ports_per_epl;
	*lane = ext_port_idx % ports->ports_per_epl;
}

static void
fm10k_ext_port_phy_enable(struct fm10k_ext_port *port)
{
	struct fm10k_switch *sw = port->ports->sw;
	struct fm10k_device_info *cfg = sw->info;

	switch (FM10K_SW_CARD_ID(cfg->subvendor, cfg->subdevice)) {
	case FM10K_SW_CARD(SILICOM, PE340G2DBIR_QS41):
	case FM10K_SW_CARD(SILICOM_RB, PE340G2DBIR_QS41):
	case FM10K_SW_CARD(SILICOM, PE340G2DBIR_QS43):
	case FM10K_SW_CARD(SILICOM_RB, PE340G2DBIR_QS43):
	case FM10K_SW_CARD(SILICOM, PE340G2DBIR_QL4):
	case FM10K_SW_CARD(SILICOM_RB, PE340G2DBIR_QL4):
		FM10K_SW_I2C_LOCK(sw->i2c);
		/* Set up the first PCA9545 mux so we can get at the
		 * PCA9505 that the QSFP control lines are connected
		 * to.
		 */
		fm10k_i2c_write8(sw->i2c, 0x70, 0x04);

		/* Configure the I/O pins on the PCA9505 that are
		 * connected to LpMode and ResetL on the QSFP as
		 * outputs, and set LpMode to low to enable high
		 * power mode and set ResetL to high to take it out
		 * of reset.
		 */
		if (port->portno == 0) {
			fm10k_i2c_write16(sw->i2c, 0x20, 0x18, 0xf6);
			fm10k_i2c_write16(sw->i2c, 0x20, 0x08, 0x08);
		} else {
			fm10k_i2c_write16(sw->i2c, 0x20, 0x19, 0xf6);
			fm10k_i2c_write16(sw->i2c, 0x20, 0x09, 0x08);
		}
		FM10K_SW_I2C_UNLOCK(sw->i2c);
		break;

	case FM10K_SW_CARD(SILICOM, PE3100G2DQIR_QXSL4):
	case FM10K_SW_CARD(SILICOM, PE3100G2DQIR_QXSL4_REV_2):
			FM10K_SW_I2C_LOCK(sw->i2c);
			/* Set up the first PCA9545 mux so we can get
			 * at the PCA9505 that the QSFP control lines
			 * are connected to.
			 */
			fm10k_i2c_write8(sw->i2c, 0x70, 0x01);

			/* Configure the I/O pins on the PCA9505 that are
			 * connected to LpMode and ResetL on the QSFP as
			 * outputs, and set LpMode to low to enable high
			 * power mode and set ResetL to high to take it out
			 * of reset.
			 */
			if (port->portno == 0) {
				fm10k_i2c_write16(sw->i2c, 0x20, 0x18, 0xf6);
				fm10k_i2c_write16(sw->i2c, 0x20, 0x08, 0x08);
			} else {
				fm10k_i2c_write16(sw->i2c, 0x20, 0x19, 0xf6);
				fm10k_i2c_write16(sw->i2c, 0x20, 0x09, 0x08);
			}
			FM10K_SW_I2C_UNLOCK(sw->i2c);
			break;

	case FM10K_SW_CARD(SILICOM, PE3100G2DQIRL_QXSL4):
	case FM10K_SW_CARD(SILICOM, PE3100G2DQIRM_QXSL4):
		FM10K_SW_I2C_LOCK(sw->i2c);

		/* Configure the I/O pins on the PCA9538 that are
		 * connected to LpMode and ResetL on the QSFP as
		 * outputs, and set LpMode to low to enable high
		 * power mode and set ResetL to high to take it out
		 * of reset.
		 */
		if (port->portno == 0) {
			fm10k_i2c_write16(sw->i2c, 0x60, 0x03, 0xcf);
			fm10k_i2c_write16(sw->i2c, 0x60, 0x01, 0x10);
		} else {
			fm10k_i2c_write16(sw->i2c, 0x61, 0x03, 0xcf);
			fm10k_i2c_write16(sw->i2c, 0x61, 0x01, 0x10);
		}
		FM10K_SW_I2C_UNLOCK(sw->i2c);
		break;

	default:
		FM10K_SW_ERR("don't know how to enable phy for this card "
		    "(subvendor=0x%04x subdevice=0x%04x)\n",
		    cfg->subvendor, cfg->subdevice);
		break;
	}
}

static void
fm10k_ext_port_phy_disable(struct fm10k_ext_port *port)
{
	struct fm10k_switch *sw = port->ports->sw;
	struct fm10k_device_info *cfg = sw->info;

	switch (FM10K_SW_CARD_ID(cfg->subvendor, cfg->subdevice)) {
	case FM10K_SW_CARD(SILICOM, PE340G2DBIR_QS41):
	case FM10K_SW_CARD(SILICOM_RB, PE340G2DBIR_QS41):
	case FM10K_SW_CARD(SILICOM, PE340G2DBIR_QS43):
	case FM10K_SW_CARD(SILICOM_RB, PE340G2DBIR_QS43):
	case FM10K_SW_CARD(SILICOM, PE340G2DBIR_QL4):
	case FM10K_SW_CARD(SILICOM_RB, PE340G2DBIR_QL4):
		FM10K_SW_I2C_LOCK(sw->i2c);
		/* Set up the first PCA9545 mux so we can get at the PCA9505
		 * that the QSFP control lines are connected to.
		 */
		fm10k_i2c_write8(sw->i2c, 0x70, 0x04);

		/* Configure the I/O pins on the PCA9505 that are
		 * connected to LpMode and ResetL on the QSFP as
		 * outputs, and set LpMode to high to disable high
		 * power mode and set ResetL to low to put it in
		 * reset.
		 */
		if (port->portno == 0) {
			fm10k_i2c_write16(sw->i2c, 0x20, 0x18, 0xf6);
			fm10k_i2c_write16(sw->i2c, 0x20, 0x08, 0x01);
		} else {
			fm10k_i2c_write16(sw->i2c, 0x20, 0x19, 0xf6);
			fm10k_i2c_write16(sw->i2c, 0x20, 0x09, 0x01);
		}
		FM10K_SW_I2C_UNLOCK(sw->i2c);
		break;

	case FM10K_SW_CARD(SILICOM, PE3100G2DQIR_QXSL4):
	case FM10K_SW_CARD(SILICOM, PE3100G2DQIR_QXSL4_REV_2):
	FM10K_SW_I2C_LOCK(sw->i2c);
	/* Set up the first PCA9545 mux so we can get at the PCA9505
	 * that the QSFP control lines are connected to.
	 */
	fm10k_i2c_write8(sw->i2c, 0x70, 0x01);

	/* Configure the I/O pins on the PCA9505 that are
	 * connected to LpMode and ResetL on the QSFP as
	 * outputs, and set LpMode to high to disable high
	 * power mode and set ResetL to low to put it in
	 * reset.
	 */
	if (port->portno == 0) {
		fm10k_i2c_write16(sw->i2c, 0x20, 0x18, 0xf6);
		fm10k_i2c_write16(sw->i2c, 0x20, 0x08, 0x01);
	} else {
		fm10k_i2c_write16(sw->i2c, 0x20, 0x19, 0xf6);
		fm10k_i2c_write16(sw->i2c, 0x20, 0x09, 0x01);
	}
	FM10K_SW_I2C_UNLOCK(sw->i2c);
	break;
	case FM10K_SW_CARD(SILICOM, PE3100G2DQIRL_QXSL4):
	case FM10K_SW_CARD(SILICOM, PE3100G2DQIRM_QXSL4):
		FM10K_SW_I2C_LOCK(sw->i2c);
		/* Configure the I/O pins on the PCA9538 that are
		 * connected to LpMode and ResetL on the QSFP as
		 * outputs, and set LpMode to high to disable high
		 * power mode and set ResetL to low to put it in
		 * reset.
		 */
		if (port->portno == 0) {
			fm10k_i2c_write16(sw->i2c, 0x60, 0x03, 0xcf);
			fm10k_i2c_write16(sw->i2c, 0x60, 0x01, 0x20);
			FM10K_SW_ERR("%s config PCA9538 %#x set LpMode",
					__func__, 0x60);
		} else {
			fm10k_i2c_write16(sw->i2c, 0x61, 0x03, 0xcf);
			fm10k_i2c_write16(sw->i2c, 0x61, 0x01, 0x20);
			FM10K_SW_ERR("%s config PCA9538 %#x set LpMode",
					__func__, 0x61);
		}
		FM10K_SW_I2C_UNLOCK(sw->i2c);
		break;

	default:
		FM10K_SW_ERR("don't know how to disable phy for this card "
		    "(subvendor=0x%04x subdevice=0x%04x)\n",
		    cfg->subvendor, cfg->subdevice);
		break;
	}
}
