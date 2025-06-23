/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Generic Event Handling
 *
 * Copyright (C) 2018 Laurent Pinchart
 *
 * This file comes from the omap3-isp-live project
 * (git://git.ideasonboard.org/omap3-isp-live.git)
 *
 * Copyright (C) 2010-2011 Ideas on board SPRL
 *
 * Contact: Laurent Pinchart <laurent.pinchart@ideasonboard.com>
 */

#ifndef __EVENTS_H__
#define __EVENTS_H__

#include <stdbool.h>
#include <sys/select.h>

#include "list.h"

struct events {
	struct list_entry events;
	bool done;

	int maxfd;
	fd_set rfds;
	fd_set wfds;
	fd_set efds;
};

enum event_type {
	EVENT_READ = 1,
	EVENT_WRITE = 2,
	EVENT_EXCEPTION = 4,
};

void events_watch_fd(struct events *events, int fd, enum event_type type,
		     void(*callback)(void *), void *priv);
void events_unwatch_fd(struct events *events, int fd, enum event_type type);

bool events_loop(struct events *events);
void events_stop(struct events *events);

void events_init(struct events *events);
void events_cleanup(struct events *events);

#endif
