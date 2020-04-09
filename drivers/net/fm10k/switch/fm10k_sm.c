/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright 2019   Silicom Ltd. Connectivity Solutions
 */

#include <stdint.h>
#include <stdlib.h>
#include <rte_malloc.h>

#include "../base/fm10k_osdep.h"
#include "fm10k_debug.h"
#include "fm10k_sm.h"

static void *fm10k_sm_handle_event(void *ptr);
static void *fm10k_sm_handle_timer(void *ptr);
static void fm10k_sm_event_heap_init(struct fm10k_sm_event_heap *heap);
static uint16_t fm10k_sm_event_heap_pop(struct fm10k_sm_event_heap *heap);
static void fm10k_sm_event_heap_push
		(struct fm10k_sm_event_heap *heap, uint16_t id);

struct fm10k_sm *
fm10k_sm_attach(uint16_t num_events,
		uint16_t num_timers, fm10k_sm_process_event_t event_handler,
		fm10k_sm_process_timer_t timer_handler, void *ctx)
{
	struct fm10k_sm *sm;
	struct fm10k_sm_timer *timer;
	unsigned int i;

	sm = (struct fm10k_sm *)rte_zmalloc("fm10k_sm",
					sizeof(struct fm10k_sm), 0);
	if (sm == NULL)
		goto fail;

	sm->is_running = true;
	sm->num_events = num_events;
	fm10k_sm_event_heap_init(&sm->events);
	pthread_create(&sm->event_task, NULL, fm10k_sm_handle_event, sm);
	sm->num_timers = num_timers;
	if (sm->num_timers > 0) {
		sm->timers = (struct fm10k_sm_timer *)rte_zmalloc
			("fm10k_sm_timers",
			sizeof(struct fm10k_sm_timer) * sm->num_timers, 0);
		if (sm->timers == NULL)
			goto fail;
	}

	sm->event_handler = event_handler;
	sm->timer_handler = timer_handler;
	sm->ctx = ctx;

	for (i = 0; i < sm->num_timers; i++) {
		timer = &sm->timers[i];
		sem_init(&timer->tq, 0, 0);
		timer->sm = sm;
		pthread_create(&timer->t, NULL, fm10k_sm_handle_timer, timer);
		timer->timer_id = i;
	}

	return sm;

fail:
	if (sm)
		fm10k_sm_detach(sm);

	return NULL;
}

void
fm10k_sm_detach(struct fm10k_sm *sm)
{
	int i;

	sm->is_running = false;
	pthread_join(sm->event_task, NULL);
	for (i = 0; i < sm->num_timers; i++)
		pthread_join(sm->timers[i].t, NULL);
	rte_free(sm);
}

void
fm10k_sm_send_event(struct fm10k_sm *sm, uint16_t event_id)
{
	fm10k_sm_event_heap_push(&sm->events, event_id);
}

void
fm10k_sm_timer_start(struct fm10k_sm *sm, uint16_t timer_id,
		unsigned int timeout_ms)
{
	struct fm10k_sm_timer *timer = &sm->timers[timer_id];

	gettimeofday(&timer->start, 0);
	timer->timeout_ms = timeout_ms;
	timer->running = 1;
	sem_post(&timer->tq);
}

/*
 * It is possible that an expiration events from this timer will
 * occur after this is called.
 */
void
fm10k_sm_timer_cancel(struct fm10k_sm *sm, uint16_t timer_id)
{
	struct fm10k_sm_timer *timer = &sm->timers[timer_id];
	timer->running = 0;
}

static void *
fm10k_sm_handle_event(void *ctx)
{
	struct fm10k_sm *sm = ctx;
	uint16_t event_id;

	while (sm->is_running) {
		event_id = fm10k_sm_event_heap_pop(&sm->events);
		sm->event_handler(sm, event_id);
	}
	return NULL;
}

static void *
fm10k_sm_handle_timer(void *ctx)
{
	struct fm10k_sm_timer *timer = ctx;
	struct fm10k_sm *sm = timer->sm;
	struct timeval now;
	struct timeval sleep;
	uint64_t cost, timeout;

	while (sm->is_running) {
		sem_wait(&timer->tq);
		if (timer->running)	{
			gettimeofday(&now, 0);
			cost = (now.tv_sec - timer->start.tv_sec) * 1000000;
			cost += (now.tv_usec - timer->start.tv_usec);
			if (cost < timer->timeout_ms * 1000) {
				timeout = timer->timeout_ms * 1000;
				timeout -= cost;
				sleep.tv_sec = timeout / 1000000;
				sleep.tv_usec = timeout % 1000000;
				select(0, NULL, NULL, NULL, &sleep);
			}
			if (timer->running)
				sm->timer_handler(sm, timer->timer_id);
		}
	}

	return NULL;
}

static void
fm10k_sm_event_heap_init(struct fm10k_sm_event_heap *heap)
{
	heap->count = 0;
	pthread_mutex_init(&heap->lock, NULL);
	sem_init(&heap->s, 0, 0);
}

static uint16_t
fm10k_sm_event_heap_pop(struct fm10k_sm_event_heap *heap)
{
	int i;
	uint16_t id;

	sem_wait(&heap->s);
	pthread_mutex_lock(&heap->lock);
	id = heap->heap[0];
	for (i = 1; i < heap->count; i++)
		heap->heap[i - 1] = heap->heap[i];
	heap->count--;
	pthread_mutex_unlock(&heap->lock);
	return id;
}

static void
fm10k_sm_event_heap_push(struct fm10k_sm_event_heap *heap, uint16_t id)
{
	pthread_mutex_lock(&heap->lock);
	if (heap->count >= FM10K_SM_ID_HEAP_MAX) {
		pthread_mutex_unlock(&heap->lock);
		return;
	}
	heap->heap[heap->count] = id;
	heap->count++;
	pthread_mutex_unlock(&heap->lock);
	sem_post(&heap->s);
}
