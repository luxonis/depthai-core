/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * libuvcgadget timer utils
 *
 * Copyright (C) 2022 Daniel Scally
 *
 * Contact: Daniel Scally <dan.scally@ideasonboard.com>
 */

struct timer;

/*
 * timer_new - Create a new timer
 *
 * Allocates and returns a new struct timer. This must be configured with
 * timer_set_fps() and then armed with timer_arm(), following which calls to
 * timer_wait() will block until the expiration of a period as defined by
 * timer_set_fps().
 *
 * Timers allocated with this function should be removed with timer_destroy()
 */
struct timer *timer_new(void);

/*
 * timer_set_fps - Configure the timer's wait period
 *
 * Configure the timer to wait for a period of time such that expirations per
 * second matches @fps
 */
void timer_set_fps(struct timer *timer, int fps);

/*
 * timer_arm
 *
 * Arms the timer such that calls to timer_wait() become blocking until the
 * expiration of a period.
 */
int timer_arm(struct timer *timer);

/*
 * timer_disarm
 *
 * Disarms the timer such that calls to timer_wait() return without blocking.
 */
int timer_disarm(struct timer *timer);

/*
 * timer_wait
 *
 * If the timer is armed, block until the expiration of a period
 */
void timer_wait(struct timer *timer);

/*
 * timer_destroy
 *
 * Close the timer's file descriptor and free the memory
 */
void timer_destroy(struct timer *timer);
