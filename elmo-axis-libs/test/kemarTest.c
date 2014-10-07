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
#include <elmo-axis/kemar.h>

int
main(int argc, char *argv[])
{
	double target = 0.0;
	int i = 0;
	CAN_HARMONICA_STR *h;
	KEMAR_POS_VEL_STR *k;
	CAN_DRIVE_PARAMS *p;


	/*------------------------ Init fonctions ----------------------------------*/
	/* init kemar head */
	h = kemarInit(MOTIONTYPE_POSCTRL);
	p = &h->driveParam;
	/* homing procedure */
	(kemarHoming(h) == -1) ? errx(2, "kemarHoming") : printf("Homing done\n");
	/* init kemar head struct using homing init */
	k = kemarStructInit(h);
	/*--------------------------------------------------------------------------*/



	while(1) {
		/*----------------------------------- Control in Position -------------------*/

		/* Set speed to 1.0 rad/s (max speed between 2 positions) and set control as control position */ 
		kemarSetGearVelRadS(h, k, 1.0, MOTIONTYPE_POSCTRL);

		/* set 2 targets 0.5rad & -0.7rad (absolute) */
		for(i = 0; i < 4 ; i++) {
			target = !(i % 2) ? (0.5) : (-0.7);
			kemarSetGearPosAbsRad(h, k, target);
			kemarWaitMsgValid(h, MSG_PX_TARGET, target);
			kemarShow(h, k);
		}

		/* back to 0.0rad & wait to reach position 0.0rad (locking fct) & show positions current pos */
		kemarSetGearPosAbsRad(h, k, 0.0);
		kemarWaitMsgValid(h, MSG_PX_TARGET, 0.0);
		kemarShow(h, k);

		/* Set speed to 5.0 rad/s (max speed between 2 positions) and set control as control position */ 
		kemarSetGearVelRadS(h, k, 5.0, MOTIONTYPE_POSCTRL);

		/* set 2 targets 8753Increments & -4357Increments of the encoder at the back of the motor (absolute) */
		for(i = 0; i < 4 ; i++) {
			target = !(i % 2) ? (8753.0) : (-4357.0);
			kemarSetEncPosAbsIncr(h, k, (int)target);
			kemarWaitMsgValid(h, MSG_PX_TARGET, driveParamPosMotIncrToPosGearRad(p, (int)target));
			kemarShow(h, k);
		}

		/* back to 0.0rad & wait to reach position 0.0rad (locking fct) & show positions current pos */
		kemarSetGearPosAbsRad(h, k, 0.0);
		kemarWaitMsgValid(h, MSG_PX_TARGET, 0.0);
		kemarShow(h, k);

		/* Set speed to 0.5 rad/s (max speed between 2 positions) and set control as control position */ 
		kemarSetGearVelRadS(h, k, 0.5, MOTIONTYPE_POSCTRL);

		/* relative position command : current pos is incremented with 0.2rad */
		for(i = 0; i < 4 ; i++) {
			target = 0.2;
			kemarSetGearPosRelRad(h, k, target);
			kemarWaitMsgValid(h, MSG_PX_TARGET, (target * (i + 1)));
			kemarShow(h, k);
		}

		/* relative position command : current pos is decremented with -1000Increments */
		for(i = 0; i < 4 ; i++) {
			target = -1000;
			kemarSetEncPosRelIncr(h, k, target);
			sleep(1);
			kemarShow(h, k);
		}

		/*----------------------------------- Control in Speed ---------------------------*/
		for(i = 0; i < 4 ; i++) {
			/* back to 0.0rad & wait to reach position 0.0rad (locking fct) & show positions current pos */
			kemarSetGearPosAbsRad(h, k, 0.0);
			kemarWaitMsgValid(h, MSG_PX_TARGET, 0.0);
			kemarShow(h, k);
		
			/* set speed to -0.4rad/s and wait 1s + show current position */
			kemarSetGearVelRadS(h, k, -0.4, MOTIONTYPE_VELCTRL);
			sleep(1);
			kemarShow(h, k);

			/* set speed to 0.1d/s and wait 2s + show current position */
			kemarSetGearVelRadS(h, k, 0.1, MOTIONTYPE_VELCTRL);
			kemarWaitMsgValid(h, MSG_PX, 0.0);
			sleep(2);
			kemarShow(h, k);

			/* set speed to 0.0d/s and wait 100ms+  show current position */
			kemarSetGearVelRadS(h, k, 0.0, MOTIONTYPE_VELCTRL);
			kemarWaitMsgValid(h, MSG_PX, 0.0);
			mssleep(100);
			kemarShow(h, k);
		}
	}

	/*------------------------ Stop fonctions ----------------------------------*/
	kemarStructEnd(k);	
	harmonicaStop(h);
	harmonicaEnd(h);
	socketcanEnd(h->dev);
	exit(0);
}
