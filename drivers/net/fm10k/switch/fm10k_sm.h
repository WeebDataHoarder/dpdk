/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright 2019   Silicom Ltd. Connectivity Solutions
 */

#ifndef _FM10K_SW_SM_H_
#define _FM10K_SW_SM_H_

#include <stdint.h>
#include <signal.h>
#include <sys/time.h>
#include <pthread.h>
#include <semaphore.h>
#include <stdbool.h>

/*
 * This framework provides state machine implementation support for use in
 * the rest of the driver.
 *
 * The model is that for a given state machine, there are N>0 events
 * identified by the integers [0..N-1] and there are M>=0 timers identified
 * by the integers [0..M-1].
 *
 * The timers are one-shot timers.
 *
 * Since all state machine processing occurs in the context of the
 * task queue, if that task queue was created with only a single thread, then
 * no locking is required in the provided event and timer handlers to
 * prevent overlapping execution.
 *
 */

struct fm10k_sm;

struct fm10k_sm_timer {
	struct fm10k_sm *sm;
	pthread_t t;
	sem_t tq;
	struct timeval start;
	uint16_t timeout_ms;
	uint16_t timer_id;
	uint8_t running;
};

typedef void (*fm10k_sm_process_event_t)(struct fm10k_sm *sm, uint16_t event);
typedef void (*fm10k_sm_process_timer_t)(struct fm10k_sm *sm, uint16_t timer);

#define FM10K_SM_ID_HEAP_MAX	128
struct fm10k_sm_event_heap {
	pthread_mutex_t lock;
	sem_t s;
	uint16_t count;
	uint16_t heap[FM10K_SM_ID_HEAP_MAX];
};

struct fm10k_sm {
	bool is_running;
	pthread_t event_task;
	struct fm10k_sm_timer *timers;
	struct fm10k_sm_event_heap events;
	fm10k_sm_process_event_t event_handler;
	fm10k_sm_process_timer_t timer_handler;
	void *ctx;
	unsigned int state;
	uint16_t num_events;
	uint16_t num_timers;
	uint8_t portno;
	uint8_t laneno;
};

struct fm10k_sm *fm10k_sm_attach(uint16_t num_events,
	    uint16_t num_timers, fm10k_sm_process_event_t event_handler,
	    fm10k_sm_process_timer_t timer_handler, void *ctx);
void fm10k_sm_detach(struct fm10k_sm *sm);
void fm10k_sm_send_event(struct fm10k_sm *sm, uint16_t event_id);
void fm10k_sm_timer_start(struct fm10k_sm *sm,
		uint16_t timer_id, unsigned int timeout_ms);
void fm10k_sm_timer_cancel(struct fm10k_sm *sm, uint16_t timer_id);

#endif /* _FM10K_SW_SM_H_ */
