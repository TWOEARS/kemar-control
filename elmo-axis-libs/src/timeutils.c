/*
 * Copyright (c) 2005,2011-2014 CNRS/LAAS
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#include <errno.h>
#include <math.h>
#include <time.h>

#include "timeutils.h"

/**
 ** time utilities
 **/
/*----------------------------------------------------------------------*/

/*
 * Convert a struct timespec to a double (in seconds)
 */
double
timeSpecToDouble(struct timespec *ts)
{
	return (double)ts->tv_sec + (double)ts->tv_nsec/1e9;
}
/*----------------------------------------------------------------------*/
/*
 * Convert a double (seconds) to a struct timespec
 */
void
doubleToTimeSpec(double t, struct timespec *ts)
{
	if (!(t < 4e9 && t > 0.0)) {
		ts->tv_sec = 0;
		ts->tv_nsec = 0;
		return;
	}
	ts->tv_sec = (time_t)t;
	ts->tv_nsec = (int)remainder(t, 1e9);
}
/*----------------------------------------------------------------------*/
/*
 * Return the difference in seconds (double) between 2 struct timespec
 */
double
absDiffTime(struct timespec *t1, struct timespec *t2)
{
	struct timespec res;

	res.tv_sec = t1->tv_sec - t2->tv_sec;
	res.tv_nsec = t1->tv_nsec - t2->tv_nsec;
	if (res.tv_nsec < 0) {
		res.tv_sec--;
		res.tv_nsec += 1000000000;
	}
	return timeSpecToDouble(&res);
}

/*----------------------------------------------------------------------*/

int
mssleep(unsigned int ms)
{
	struct timespec s, r;
	
	s.tv_sec = ms / 1000;
	s.tv_nsec = (ms % 1000) * 1000000;
	
	do {
		if (nanosleep(&s, &r) == 0)
			break;
		if (errno != EINTR)
			return -1;
		s.tv_sec = r.tv_sec;
		s.tv_nsec = r.tv_nsec;
	} while (r.tv_sec != 0 || r.tv_nsec != 0);
	return 0;
}
