/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * libuvcgadget timer utils
 *
 * Copyright (C) 2022 Daniel Scally
 *
 * Contact: Daniel Scally <dan.scally@ideasonboard.com>
 */

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/timerfd.h>
#include <unistd.h>

#include "timer.h"

struct timer {
        int fd;
        struct itimerspec settings;
};

struct timer *timer_new(void)
{
        struct timer *timer;

        timer = malloc(sizeof(*timer));
        if (!timer)
                return NULL;

        memset(timer, 0, sizeof(*timer));

        timer->fd = timerfd_create(CLOCK_REALTIME, 0);
        if (timer->fd < 0) {
		fprintf(stderr, "failed to create timer: %s (%d)\n",
			strerror(errno), errno);
		goto err_free_timer;
	}

        return timer;

err_free_timer:
        free(timer);

        return NULL;
}

void timer_set_fps(struct timer *timer, int fps)
{
	int ns_per_frame = 1000000000 / fps;

	timer->settings.it_value.tv_nsec = ns_per_frame;
	timer->settings.it_interval.tv_nsec = ns_per_frame;
}

int timer_arm(struct timer *timer)
{
        int ret;

	ret = timerfd_settime(timer->fd, 0, &timer->settings, NULL);
	if (ret)
		fprintf(stderr, "failed to change timer settings: %s (%d)\n",
			strerror(errno), errno);

        return ret;
}

int timer_disarm(struct timer *timer)
{
	static const struct itimerspec disable_settings = {
		{ 0, 0 },
		{ 0, 0 },
	};
	int ret;

	ret = timerfd_settime(timer->fd, 0, &disable_settings, NULL);
	if (ret)
		fprintf(stderr, "failed to disable timer: %s (%d)\n",
			strerror(errno), errno);

	return ret;
}

void timer_wait(struct timer *timer)
{
        char read_buf[8];

        read(timer->fd, read_buf, 8);
}

void timer_destroy(struct timer *timer)
{
        close(timer->fd);
        free(timer);
}
