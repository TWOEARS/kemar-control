/*
 * Copyright (c) 2011-2014 CNRS/LAAS
 *
 * Permission to use, copy, modify, and/or distribute this software for any
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

#include <sys/types.h>
#include <err.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <elmo-axis/socketcan.h>
#include <elmo-axis/harmonica.h>

int done = 0;

/*
 * For this motor, the position and speed are in m and m/s respectively
 */
const struct CAN_DRIVE_PARAMS axisParams = {
	.encIncrPerRevMot = 2048,
	.gearRatio = 9,
	.beltRatio = 1,
	.sign = 1.0,
	.velMaxEncIncrS = 200000,
	.accIncrS = 300000,
	.decIncrS = 300000,
	.encOffsetIncr = 0,
	.currToTorque = 0.10065,
	.currMax = 9.62,
	.homingDigIn = 7,
	.homingSpeed = -20000,	/* move downwards for calibration */
	.homingDelay = 40,	/* theorical 25s at maximum course */
};

static void *
axisReadTask(void *p)
{
	CAN_HARMONICA_STR *h = (CAN_HARMONICA_STR *)p;
	CAN_MSG m;

	do {
		if (socketcanReceiveMsgWait(h->dev, &m, 1000) != 0) {
			fprintf(stderr, "can receive error\n");
		}
		harmonicaDecode(h, &m);
	} while (!done);

	return NULL;
}

int
main(int argc, char *argv[])
{
	pthread_t readTask;
	int dev, i, status;
	CAN_HARMONICA_STR *h;
	char *name;

	if (argc == 2)
		name = argv[1];
	else
		name = "can0";

	dev = socketcanInit(name);
	if (dev == -1) {
		errx(2, "socketcanInit");
	}
	h = harmonicaInit(dev, 127, MOTIONTYPE_VELCTRL, &axisParams);

	if (h == NULL) {
		socketcanEnd(dev);
		errx(2, "harmonicaInit");
	}

	/* init the watchdog, but disable it */
	harmonicaStartWatchdog(h, true);
	usleep(10000);
	harmonicaStartWatchdog(h, false);
	printf("watchdog initialized\n");


	/* Init motor */
	if (harmonicaInitCtrl(h) == -1) {
		errx(2, "harmonicaInitCtrl");
	}
	printf("harmonica control started\n");

	if ((status = pthread_create(&readTask, NULL, axisReadTask, h)) != 0) {
		harmonicaEnd(h);
		socketcanEnd(dev);
		errx(2, "pthread_create: %s", strerror(status)); 
	}

	/* To be sure */
	harmonicaSetGearVelRadS(h, 0.0);
	/* Start motor */
	if (harmonicaStart(h) == -1) {
		errx(2, "harmonicaStart");
	}
	printf("motor started\n");

	harmonicaStartWatchdog(h, true);
	for (i = 0; i < 100; i++) {
		harmonicaSetGearVelRadS(h, 1.0);
		if (i % 10 == 0)
			harmonicaShow(h);
		usleep(100000);
	}

	harmonicaSetGearVelRadS(h, 0.0);

	done = 1;
	pthread_join(readTask, NULL);
	harmonicaStop(h);
	harmonicaEnd(h);
	socketcanEnd(dev);
	exit(0);
}
