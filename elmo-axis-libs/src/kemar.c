/*
 * Copyright (c) 2005-2011 CNRS/LAAS
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

#include <sys/types.h>

#include <err.h>
#include <inttypes.h>
#include <math.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include <elmo-axis/socketcan.h>
#include <elmo-axis/harmonica.h>
#include <elmo-axis/kemar.h>

#include "timeutils.h"

//#define CAN_BUS_DEBUG 1

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
	.homingSpeed = 10000,	/* move downwards for calibration */
	.homingDelay = 40,	/* theorical 25s at maximum course */
};

/**
 ** Interface with kemar head
 **/

/*----------------------------------------------------------------------*/
/*
 * initalize a homing proccedure to detect RLS & FLS 
 */
int 
kemarHoming(CAN_HARMONICA_STR *h) 
{
	/* Set DIN#5 & DIN#6 as RLS & FLS switchs (active high) */
	if(kemarSwitchesInit(h))
		return -1;
	
	/* Request Mask IT & save previous mask & disable all IT */
	if(kemarWaitMsgValid(h, MSG_MI, 0.0))
		return -1;
	intprtSetInt(h, 8, 'M', 'I', 0, MASK_IT_ALL_DISABLE);
  
	/* configure homing for FLS event & wait FLS reached */
	if(kemarHomingRegConfig(h, FLS))
		return -1;

	/* configure homing for RLS event & wait RLS reached */
	if(kemarHomingRegConfig(h, RLS))
		return -1;

	/* Request absolute position & save it */
	if(kemarWaitMsgValid(h, MSG_PX, 0.0))
		return -1;

	/* go to central position -> PX/2 + start motion & wait center reached */
	if(kemarHomingRegConfig(h, TARGET))
		return -1;
	mssleep(500);

	/* reset PX */
	if(kemarHomingRegConfig(h, ZERO))
		return -1;

	/* Set  AUTO_IT routine to previous version */
	intprtSetInt(h, 8, 'M', 'I', 0, h->maskIT);

	/* Purge incoming messages */
	kemarWaitMsgValid(h, MSG_PULL, 0.0);

	/* homing done */
	h->homePos = true;
  
	return 0;
}

/*----------------------------------------------------------------------*/
/*
 * Set registers for homing 
 */
int
kemarHomingRegConfig(CAN_HARMONICA_STR *h, HOMING_STATE homingState)
{
	double jogVel = 0.0, targetPos = 0.0;
	MSG_HEADER msgHeader;
	CAN_DRIVE_PARAMS *p = &h->driveParam;

	switch(homingState) {
		case TARGET : 
			targetPos = h->posGearMeasRad / 2.0;
			p->rightIncrMax = driveParamPosGearRadToPosMotIncr(p, targetPos);
			p->leftIncrMax = -driveParamPosGearRadToPosMotIncr(p, targetPos);
			p->rightRadMax = targetPos;
			p->leftRadMax = -targetPos;
			intprtSetInt(h, 8, 'M', 'O', 0, MO_MOTOR_ENABLE);
			intprtSetInt(h, 8, 'P', 'A', 0, driveParamPosGearRadToPosMotIncr(p, targetPos));
			intprtSetInt(h, 4, 'B', 'G', 0, 0);
			intprtSetInt(h, 4, 'B', 'G', 0, 0);
			msgHeader = MSG_PX_TARGET;
			break;

		case FLS :
			intprtSetInt(h, 8, 'H', 'M', 2, 0);
			intprtSetInt(h, 8, 'H', 'M', 3, HM_3_EVT_FLS_SW);
			intprtSetInt(h, 8, 'H', 'M', 4, HM_4_AFT_EVT_STOP);
			intprtSetInt(h, 8, 'H', 'M', 5, HM_5_SET_PX_ABSOLUTE);
			intprtSetInt(h, 8, 'H', 'M', 1, HM_1_ARM_HOM_PROC);
			jogVel = h->driveParam.sign * h->driveParam.homingSpeed;
			intprtSetInt(h, 8, 'M', 'O', 0, MO_MOTOR_ENABLE);
			intprtSetInt(h, 8, 'J', 'V', 0, (int)jogVel);
			intprtSetInt(h, 4, 'B', 'G', 0, 0);
			msgHeader = MSG_IB_FLS;
			break;

		case RLS :
			intprtSetInt(h, 8, 'H', 'M', 2, 0);
			intprtSetInt(h, 8, 'H', 'M', 3, HM_3_EVT_RLS_SW);
			intprtSetInt(h, 8, 'H', 'M', 4, HM_4_AFT_EVT_STOP);
			intprtSetInt(h, 8, 'H', 'M', 5, HM_5_SET_PX_NOTHING);
			intprtSetInt(h, 8, 'H', 'M', 1, HM_1_ARM_HOM_PROC);
			jogVel = -h->driveParam.sign * h->driveParam.homingSpeed;
			intprtSetInt(h, 8, 'M', 'O', 0, MO_MOTOR_ENABLE);
			intprtSetInt(h, 8, 'J', 'V', 0, (int)jogVel);
			intprtSetInt(h, 4, 'B', 'G', 0, 0);
			msgHeader = MSG_IB_RLS;
			break;

		case ZERO :
			intprtSetInt(h, 8, 'M', 'O', 0, MO_MOTOR_DISABLE);
			intprtSetInt(h, 8, 'P', 'X', 0, 0);
			intprtSetInt(h, 8, 'M', 'O', 0, MO_MOTOR_ENABLE);
			msgHeader = MSG_PX;
			break;

		default :
			return -1;
	}
	return (kemarWaitMsgValid(h, msgHeader, targetPos));
}

/*----------------------------------------------------------------------*/
/*
 * Init CAN bus + harmonica structure + start motor & init homing.
 */
CAN_HARMONICA_STR *
kemarInit(CAN_MOTION_TYPE motionType)
{
	int dev;
	CAN_HARMONICA_STR *h;

	/* Init CAN bus controller */
	((dev = socketcanInit(CAN_ID)) == -1) ? errx(2, "socketcanInit") : printf("CAN bus initialized\n");

	/* Init Harmonica struct */
	if((h = harmonicaInit(dev, HARMONICA_ID, motionType, &axisParams)) == NULL) {
		socketcanEnd(dev);
		errx(2, "harmonicaInit");
	}

	/* Init motor */
	(harmonicaInitCtrl(h) == -1) ? errx(2, "harmonicaInitCtrl") : printf("harmonica control started\n");

	/* Start motor */
	(harmonicaStart(h) == -1) ? errx(2, "harmonicaStart") : printf("motor started\n");

	return h;
}

/*----------------------------------------------------------------------*/
/*
 * Set DIN#5 & DIN#6 as RLS & FLS switches stop  
 */
int 
kemarSwitchesInit(CAN_HARMONICA_STR *h) 
{
	/* Set DIN#5 & DIN#6 as RLS & FLS switchs (active high) */
	intprtSetInt(h, 8, 'I', 'L', 5, IL_ACTIVE_HIGH | IL_FCT_HARD_EN_RLS);
	intprtSetInt(h, 8, 'I', 'L', 6, IL_ACTIVE_HIGH | IL_FCT_HARD_EN_FLS);
	/* Wait for RLS & FLS switchs actives */
	return (kemarWaitMsgValid(h, MSG_IL, 0.0));
}

/*----------------------------------------------------------------------*/
/*
 * Wait loop & request messages generation 
 */
int
kemarWaitMsgValid(CAN_HARMONICA_STR *h, MSG_HEADER msgHeader, double targetPos)
{
	struct timespec send, reply;
	double diffTime = 0.0;
	int ret = -1, diffPos = DIFF_VALID_POS_INCR + 1;
	CAN_MSG msg;
	CAN_DRIVE_PARAMS *p = &h->driveParam;

	clock_gettime(CLOCK_REALTIME, &send);
	clock_gettime(CLOCK_REALTIME, &reply);

	switch(msgHeader) {
		case MSG_IB_FLS :
			h->RxValidEvt.msgIBValid &= 2;
			while(((h->RxValidEvt.msgIBValid) != 1) && (diffTime < DIFF_VALID_MVT_S)) {
				!socketcanReceiveMsgWait(h->dev, &msg, MSG_REQ_DELAY_MS) ? harmonicaDecode(h, &msg) : harmonicaRequestDigIn(h);
				clock_gettime(CLOCK_REALTIME, &reply);
				diffTime = absDiffTime(&reply, &send);
			}
			ret = ((h->RxValidEvt.msgIBValid) == 1) ? 0 : -1;
			break;

		case MSG_IB_RLS :
			h->RxValidEvt.msgIBValid &= 1;
			while(((h->RxValidEvt.msgIBValid) != 2) && (diffTime < DIFF_VALID_MVT_S)) {
				!socketcanReceiveMsgWait(h->dev, &msg, MSG_REQ_DELAY_MS) ? harmonicaDecode(h, &msg) : harmonicaRequestDigIn(h);
				clock_gettime(CLOCK_REALTIME, &reply);
				diffTime = absDiffTime(&reply, &send);
			}
			ret = ((h->RxValidEvt.msgIBValid) == 2) ? 0 : -1;
			break;

		case MSG_IL :
			h->RxValidEvt.msgILValid = 0;
			while(((h->RxValidEvt.msgILValid) != 3) && (diffTime < DIFF_VALID_MSG_S)) {
				!socketcanReceiveMsgWait(h->dev, &msg, MSG_REQ_DELAY_MS) ? harmonicaDecode(h, &msg) : 0;
				clock_gettime(CLOCK_REALTIME, &reply);
				diffTime = absDiffTime(&reply, &send);
			}
			ret = ((h->RxValidEvt.msgILValid) == 3) ? 0 : -1;
			break;

		case MSG_MI :
			while(((h->RxValidEvt.msgMIValid) != 1) && (diffTime < DIFF_VALID_MSG_S)) {
				!socketcanReceiveMsgWait(h->dev, &msg, MSG_REQ_DELAY_MS) ? harmonicaDecode(h, &msg) : intprtSetInt(h, 4, 'M', 'I', 0, 0);
				clock_gettime(CLOCK_REALTIME, &reply);
				diffTime = absDiffTime(&reply, &send);
			}
			ret = ((h->RxValidEvt.msgMIValid) == 1) ? 0 : -1;
			break;

		case MSG_PULL :
			while(!socketcanReceiveMsgWait(h->dev, &msg, MSG_REQ_DELAY_MS))
				harmonicaDecode(h, &msg);
			ret = 0;
			break;

		case MSG_PX :
            intprtSetInt(h, 4, 'P', 'X', 0, 0);
			h->RxValidEvt.msgPXValid = 0;
			while(((h->RxValidEvt.msgPXValid) != 1) && (diffTime < DIFF_VALID_MSG_S)) {
				!socketcanReceiveMsgWait(h->dev, &msg, MSG_REQ_DELAY_MS) ? harmonicaDecode(h, &msg) : intprtSetInt(h, 4, 'P', 'X', 0, 0);
				clock_gettime(CLOCK_REALTIME, &reply);
				diffTime = absDiffTime(&reply, &send);
			}
			ret = ((h->RxValidEvt.msgPXValid) == 1) ? 0 : -1;
			break;

		case MSG_PX_TARGET :		
            intprtSetInt(h, 4, 'P', 'X', 0, 0);	
			while((diffPos > DIFF_VALID_POS_INCR) && (diffTime < DIFF_VALID_MVT_S)) {
				!socketcanReceiveMsgWait(h->dev, &msg, MSG_REQ_DELAY_MS) ? harmonicaDecode(h, &msg) : intprtSetInt(h, 4, 'P', 'X', 0, 0);
  				clock_gettime(CLOCK_REALTIME, &reply);
				diffTime = absDiffTime(&reply, &send);
				diffPos = abs(driveParamPosGearRadToPosMotIncr(p, (targetPos - h->posGearMeasRad)));
			}
			ret = (diffPos <= DIFF_VALID_POS_INCR) ? 0 : -1;
			break;

		case MSG_SR :
            intprtSetInt(h, 4, 'S', 'R', 0, 0);
			h->RxValidEvt.msgSRValid = 0;
			while(((h->RxValidEvt.msgSRValid) != 1) && (diffTime < DIFF_VALID_MSG_S)) {
				!socketcanReceiveMsgWait(h->dev, &msg, MSG_REQ_DELAY_MS) ? harmonicaDecode(h, &msg) : intprtSetInt(h, 4, 'S', 'R', 0, 0);
				clock_gettime(CLOCK_REALTIME, &reply);
				diffTime = absDiffTime(&reply, &send);
			}
			ret = ((h->RxValidEvt.msgSRValid) == 1) ? 0 : -1;
			break;

	}
	return (ret);	
}

/*----------------------------------------------------------------------*/
/*
 * Initialize the kemar head structure
 */
KEMAR_POS_VEL_STR *
kemarStructInit(CAN_HARMONICA_STR *h)
{
	KEMAR_POS_VEL_STR *k;
 
	k = (KEMAR_POS_VEL_STR *)malloc(sizeof(KEMAR_POS_VEL_STR));
	if (k == NULL)
		return NULL;

	k->posTargetEncIncr = 0;
	k->posTargetGearRad = 0.0;

	k->velTargetEncIncrPeriod = 0;
	k->velTargetGearRadS = 0.0;

	k->posEncIncr[0] = 0;
	k->posEncIncr[1] = 0;
	k->velEncIncrPeriod = 0;
	k->posGearRad[0] = 0.0;
	k->posGearRad[1] = 0.0;
	k->velGearRadS = 0.0;
	return k;
}

/*----------------------------------------------------------------------*/
/*
 * Ends the commnunication with a motor controller
 */
void
kemarStructEnd(KEMAR_POS_VEL_STR *k)
{
	free(k);
}

/*----------------------------------------------------------------------*/
/*
 * Set an absolute position in Rad - Position control
 */
void
kemarSetGearPosAbsRad(CAN_HARMONICA_STR *h, KEMAR_POS_VEL_STR *k, double posAbsGearRad)
{
	CAN_DRIVE_PARAMS *p = &h->driveParam;
	/* Is position control set ? if not set position control */
	kemarSetMotionType(h, MOTIONTYPE_POSCTRL);
	/* Absolute position into the limits ? */
	k->posTargetGearRad = (posAbsGearRad > h->driveParam.leftRadMax)  ? h->driveParam.leftRadMax  : posAbsGearRad;
	k->posTargetGearRad = (posAbsGearRad < h->driveParam.rightRadMax) ? h->driveParam.rightRadMax : posAbsGearRad;
	k->posTargetEncIncr = driveParamPosGearRadToPosMotIncr(p, k->posTargetGearRad);
	/* Set position & PTP velocity */
	intprtSetInt(h, 8, 'M', 'O', 0, MO_MOTOR_ENABLE);
	intprtSetInt(h, 8, 'M', 'O', 0, MO_MOTOR_ENABLE);
	intprtSetInt(h, 8, 'S', 'P', 0, k->velTargetEncIncrPeriod);
	intprtSetInt(h, 8, 'P', 'A', 0, k->posTargetEncIncr);
	intprtSetInt(h, 4, 'B', 'G', 0, 0);
	intprtSetInt(h, 4, 'B', 'G', 0, 0);
}

/*----------------------------------------------------------------------*/
/*
 * Set a relative position in Rad - Position control
 */
void
kemarSetGearPosRelRad(CAN_HARMONICA_STR *h, KEMAR_POS_VEL_STR *k, double posRelGearRad)
{
	CAN_DRIVE_PARAMS *p = &h->driveParam;
	/* Is position control set ? if not set position control */
	kemarSetMotionType(h, MOTIONTYPE_POSCTRL);
	/* Relative position into the limits ? */
	posRelGearRad += k->posGearRad[0];
	k->posTargetGearRad = (posRelGearRad > h->driveParam.leftRadMax)  ? h->driveParam.leftRadMax  : posRelGearRad;
	k->posTargetGearRad = (posRelGearRad < h->driveParam.rightRadMax) ? h->driveParam.rightRadMax : posRelGearRad;
	k->posTargetEncIncr = driveParamPosGearRadToPosMotIncr(p, k->posTargetGearRad);
	/* Set position & PTP velocity */
	intprtSetInt(h, 8, 'M', 'O', 0, MO_MOTOR_ENABLE);
	intprtSetInt(h, 8, 'M', 'O', 0, MO_MOTOR_ENABLE);
	intprtSetInt(h, 8, 'S', 'P', 0, k->velTargetEncIncrPeriod);
	intprtSetInt(h, 8, 'P', 'A', 0, k->posTargetEncIncr);
	intprtSetInt(h, 4, 'B', 'G', 0, 0);
	intprtSetInt(h, 4, 'B', 'G', 0, 0);
}

/*----------------------------------------------------------------------*/
/*
 * Set a speed in RadS- Position & speed controls
 */
void
kemarSetGearVelRadS(CAN_HARMONICA_STR *h, KEMAR_POS_VEL_STR *k, double VelGearRadS, CAN_MOTION_TYPE motionType)
{
	CAN_DRIVE_PARAMS *p = &h->driveParam;
	/* PTP velocity into the limits ? */
	k->velTargetEncIncrPeriod = driveParamVelGearRadSToVelMotIncrPeriod(p, VelGearRadS);
	k->velTargetEncIncrPeriod = (k->velTargetEncIncrPeriod > driveParamGetVelMax(p))  ? driveParamGetVelMax(p)  : k->velTargetEncIncrPeriod;
	k->velTargetEncIncrPeriod = (k->velTargetEncIncrPeriod < -driveParamGetVelMax(p)) ? -driveParamGetVelMax(p) : k->velTargetEncIncrPeriod;
	k->velTargetGearRadS = driveParamVelMotIncrPeriodToVelGearRadS(p, k->velTargetEncIncrPeriod);
	if (kemarSetMotionType(h, motionType) == MOTIONTYPE_VELCTRL) {
		intprtSetInt(h, 8, 'J', 'V', 0, k->velTargetEncIncrPeriod);
		intprtSetInt(h, 8, 'J', 'V', 0, k->velTargetEncIncrPeriod);
		intprtSetInt(h, 8, 'M', 'O', 0, MO_MOTOR_ENABLE);
		intprtSetInt(h, 4, 'B', 'G', 0, 0);
		intprtSetInt(h, 4, 'B', 'G', 0, 0);
		k->posTargetGearRad = 0.0;
		k->posTargetEncIncr = 0;
	}
}

/*----------------------------------------------------------------------*/
/*
 * Set an absolute position in Incr - Position control
 */
void
kemarSetEncPosAbsIncr(CAN_HARMONICA_STR *h, KEMAR_POS_VEL_STR *k, int posAbsEncIncr)
{
	CAN_DRIVE_PARAMS *p = &h->driveParam;
	/* Is position control set ? if not set position control */
	kemarSetMotionType(h, MOTIONTYPE_POSCTRL);
	/* Absolute position into the limits ? */
	k->posTargetEncIncr = (posAbsEncIncr > h->driveParam.leftIncrMax)  ? h->driveParam.leftIncrMax  : posAbsEncIncr;
	k->posTargetEncIncr = (posAbsEncIncr < h->driveParam.rightIncrMax) ? h->driveParam.rightIncrMax : posAbsEncIncr;
	k->posTargetGearRad = driveParamPosMotIncrToPosGearRad(p, k->posTargetEncIncr);
	/* Set position & PTP velocity */
	intprtSetInt(h, 8, 'M', 'O', 0, MO_MOTOR_ENABLE);
	intprtSetInt(h, 8, 'M', 'O', 0, MO_MOTOR_ENABLE);
	intprtSetInt(h, 8, 'S', 'P', 0, k->velTargetEncIncrPeriod);
	intprtSetInt(h, 8, 'P', 'A', 0, k->posTargetEncIncr);
	intprtSetInt(h, 4, 'B', 'G', 0, 0);
	intprtSetInt(h, 4, 'B', 'G', 0, 0);
}

/*----------------------------------------------------------------------*/
/*
 * Set a relative position in Incr - Position control
 */
void
kemarSetEncPosRelIncr(CAN_HARMONICA_STR *h, KEMAR_POS_VEL_STR *k, int posRelEncIncr)
{
	CAN_DRIVE_PARAMS *p = &h->driveParam;
	/* Is position control set ? if not set position control */
	kemarSetMotionType(h, MOTIONTYPE_POSCTRL);
	/* Relative position into the limits ? */
	posRelEncIncr += k->posEncIncr[0];
	k->posTargetEncIncr = (posRelEncIncr > h->driveParam.leftIncrMax)  ? h->driveParam.leftIncrMax  : posRelEncIncr;
	k->posTargetEncIncr = (posRelEncIncr < h->driveParam.rightIncrMax) ? h->driveParam.rightIncrMax : posRelEncIncr;
	k->posTargetGearRad = driveParamPosMotIncrToPosGearRad(p, k->posTargetEncIncr);
	/* Set position & PTP velocity */
	intprtSetInt(h, 8, 'M', 'O', 0, MO_MOTOR_ENABLE);
	intprtSetInt(h, 8, 'M', 'O', 0, MO_MOTOR_ENABLE);
	intprtSetInt(h, 8, 'S', 'P', 0, k->velTargetEncIncrPeriod);
	intprtSetInt(h, 8, 'P', 'A', 0, k->posTargetEncIncr);
	intprtSetInt(h, 4, 'B', 'G', 0, 0);
	intprtSetInt(h, 4, 'B', 'G', 0, 0);
}

/*----------------------------------------------------------------------*/
/*
 * Set a speed in IncrS- Position & speed controls
 */
void
kemarSetEncVelIncrS(CAN_HARMONICA_STR *h, KEMAR_POS_VEL_STR *k, int velEncIncrS, CAN_MOTION_TYPE motionType)
{
	CAN_DRIVE_PARAMS *p = &h->driveParam;
	/* PTP velocity into the limits ? */
	k->velTargetEncIncrPeriod = (velEncIncrS > driveParamGetVelMax(p))  ? driveParamGetVelMax(p)  : velEncIncrS;
	k->velTargetEncIncrPeriod = (velEncIncrS < -driveParamGetVelMax(p)) ? -driveParamGetVelMax(p) : velEncIncrS;
	k->velTargetGearRadS = driveParamVelMotIncrPeriodToVelGearRadS(p, k->velTargetEncIncrPeriod);
	if (kemarSetMotionType(h, motionType) == MOTIONTYPE_VELCTRL) {
		intprtSetInt(h, 8, 'J', 'V', 0, k->velTargetEncIncrPeriod);
		intprtSetInt(h, 8, 'M', 'O', 0, MO_MOTOR_ENABLE);
		intprtSetInt(h, 4, 'B', 'G', 0, 0);
		intprtSetInt(h, 4, 'B', 'G', 0, 0);
		k->posTargetGearRad = 0.0;
		k->posTargetEncIncr = 0;
	}
}

/*----------------------------------------------------------------------*/
/*
 * Set an absolute position in Rad & velocity in RadS- Position control
 */
void
kemarSetGearPosAbsVelRadS(CAN_HARMONICA_STR *h, KEMAR_POS_VEL_STR *k, double posAbsGearRad, double velGearRadS)
{
	kemarSetGearVelRadS(h, k, velGearRadS, MOTIONTYPE_POSCTRL);
	kemarSetGearPosAbsRad(h, k, posAbsGearRad);
}

/*----------------------------------------------------------------------*/
/*
 * Set a relative position in Rad & velocity in RadS- Position control
 */
void
kemarSetGearPosRelVelRadS(CAN_HARMONICA_STR *h, KEMAR_POS_VEL_STR *k, double posRelGearRad, double velGearRadS)
{
	kemarSetGearVelRadS(h, k, velGearRadS, MOTIONTYPE_POSCTRL);
	kemarSetGearPosRelRad(h, k, posRelGearRad);
}

/*----------------------------------------------------------------------*/
/*
 * Set an absolute position in Incr & velocity in IncrS- Position control
 */
void
kemarSetEncPosAbsVelIncrS(CAN_HARMONICA_STR *h, KEMAR_POS_VEL_STR *k, int posAbsEncIncr, int velEncIncrS)
{
	kemarSetEncVelIncrS(h, k, velEncIncrS, MOTIONTYPE_POSCTRL);
	kemarSetEncPosAbsIncr(h, k, posAbsEncIncr);
}

/*----------------------------------------------------------------------*/
/*
 * Set a relative position in Incr & velocity in IncrS- Position control
 */
void
kemarSetEncPosRelVelIncrS(CAN_HARMONICA_STR *h, KEMAR_POS_VEL_STR *k, int posRelEncIncr, int velEncIncrS)
{
	kemarSetEncVelIncrS(h, k, velEncIncrS, MOTIONTYPE_POSCTRL);
	kemarSetEncPosRelIncr(h, k, posRelEncIncr);
}

/*----------------------------------------------------------------------*/
/*
 * Set motion type dynamically
 */
CAN_MOTION_TYPE
kemarSetMotionType(CAN_HARMONICA_STR *h, CAN_MOTION_TYPE motionType)
{
	if (h->motionType != motionType) {
		h->motionType = motionType;
		harmonicaSetMotionType(h);
	}
	return h->motionType;
}


/*----------------------------------------------------------------------*/
/*
 * Request position, speed, status,...
 */
void
kemarGetInfo(CAN_HARMONICA_STR *h, KEMAR_POS_VEL_STR *k)
{
	CAN_DRIVE_PARAMS *p = &h->driveParam;

	//kemarWaitMsgValid(h, MSG_SR, 0.0);
	kemarWaitMsgValid(h, MSG_PX, 0.0);
	k->posEncIncr[1] = k->posEncIncr[0];
	k->posEncIncr[0] = driveParamPosGearRadToPosMotIncr(p, h->posGearMeasRad);
	k->posGearRad[1] = k->posGearRad[0];
	k->posGearRad[0] = h->posGearMeasRad;
	k->velEncIncrPeriod = driveParamVelGearRadSToVelMotIncrPeriod(p, h->velGearMeasRadS);
	k->velGearRadS = h->velGearMeasRadS;
}

/*----------------------------------------------------------------------*/
/*
 * Display status of a kemar structure
 */
void
kemarShow(CAN_HARMONICA_STR *h, KEMAR_POS_VEL_STR *k)
{
	kemarGetInfo(h, k);
	printf("------------------------------------------------------\n");
	h->motionType ? printf("motionType: MOTIONTYPE_POSCTRL\n") : printf("motionType: MOTIONTYPE_VELCTRL\n");
	printf("statusCtrl: 0x%08x\n", h->statusCtrl);
	printf("Max Left : %d (%f) - Max Right : %d (%f)\n", h->driveParam.leftIncrMax, h->driveParam.leftRadMax, 
		h->driveParam.rightIncrMax, h->driveParam.rightRadMax);
	
	if(h->motionType == MOTIONTYPE_POSCTRL) {
		printf("target Position:  %6d (%3.8f)\n", k->posTargetEncIncr, k->posTargetGearRad);
		printf("current Position: %6d (%3.8f)\n", k->posEncIncr[0], k->posGearRad[0]);
	} else {
		printf("target velocity:  %6d (%3.8f)\n", k->velTargetEncIncrPeriod, k->velTargetGearRadS);
		printf("current velocity: %6d (%3.8f)\n", k->velEncIncrPeriod, k->velGearRadS);
	}
	switch (h->motorState) {
	case ST_PRE_INITIALIZED:
		printf("state: PRE_INITIALIZED\n");
		break;
	case ST_OPERATION_ENABLED:
		printf("state: OPERATION_ENABLED\n");
		break;
	case ST_OPERATION_DISABLED:
		printf("state: OPERATION_DISABLED\n");
		break;
	case ST_MOTOR_FAILURE:
		printf("state: MOTOR_FAILURE\n");
		break;
	default:
		printf("state: %d (unknown)\n", h->motorState);
	}
	printf("------------------------------------------------------\n");
};

