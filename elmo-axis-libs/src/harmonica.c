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

#include <errno.h>
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

/* Is a bit set in a word? */
#define isBitSet(x,b) (((0x1<<(b))&(x))!=0)

#define Qencode 4
#define NMTNodeID 0x00
#define HeartbeatTimeMS 1000

//#define CAN_MSG_DEBUG 1

/**
 ** Interface with the Elmo 'harmonica' motor controller
 **/
/*----------------------------------------------------------------------*/
/*
 * Send a command to the byte interpreter (PDO2)
 */
int
intprtSetInt(CAN_HARMONICA_STR *h, int dataLen, char cmdChar1, char cmdChar2,
    int index, int data)
{
	CAN_MSG m;

	m.id = rxPDO2(h->driveParam);
	m.len = dataLen;
	m.type = 0;
	m.data[0] = cmdChar1;
	m.data[1] = cmdChar2;
	m.data[2] = index & 0xff;
	m.data[3] = index >> 8;
	m.data[4] = data & 0x000000ff;
	m.data[5] = (data & 0x0000ff00) >> 8;
	m.data[6] = (data & 0x00ff0000) >> 16;
	m.data[7] = (data & 0xff000000) >> 24;
	/* printf("intprtSetInt: ");
	   canMsgShow(&m);*/
	return socketcanTransmitMsg(h->dev, &m);
}

/*----------------------------------------------------------------------*/
/*
 * Service Data Object (SDO) transmission
 *
 * CANopen DS301 Implementation guide page 4-2 
 */
static void
sendSDODownload(CAN_HARMONICA_STR *h, int objIndex, int subIndex, int data)
{
	CAN_MSG m;
	
	const int ciInitDownloadReq = 0x20;
	const int ciNrBytesNoData = 0x00;
	const int ciExpedited = 0x02;
	const int ciDataSizeInd = 0x01;

	m.len = 8;
	m.type = 0;
	m.id = rxSDO(h->driveParam);
	m.data[0] = ciInitDownloadReq | (ciNrBytesNoData << 2) | ciExpedited | ciDataSizeInd;
	m.data[1] = (objIndex & 0x00ff);
	m.data[2] = (objIndex & 0xff00) >> 8;
	m.data[3] = subIndex;
	m.data[4] = (data & 0x000000ff);
	m.data[5] = (data & 0x0000ff00) >> 8;
	m.data[6] = (data & 0x00ff0000) >> 16;
	m.data[7] = (data & 0xff000000) >> 24;
	socketcanTransmitMsg(h->dev, &m);
}

/*----------------------------------------------------------------------*/
int
harmonicaSetMotionType(CAN_HARMONICA_STR *h)
{
	int maxAcc, maxDcc, NomSpeed;

	maxAcc = h->driveParam.accIncrS;
	maxDcc = h->driveParam.decIncrS;
	NomSpeed = h->driveParam.velMaxEncIncrS / 10;

	switch (h->motionType) {
	case MOTIONTYPE_POSCTRL:
		harmonicaStop(h);
		mssleep(20);
		/* switch unit-mode */
		intprtSetInt(h, 8, 'U', 'M', 0, 5);
		/* Set target radius to X increments */
		intprtSetInt(h, 8, 'T', 'R', 1, 15);
		/* Set target time to X ms */
		intprtSetInt(h, 8, 'T', 'R', 2, 200);

		/* Set speed between acceleration & decelaration inc/s */
		intprtSetInt(h, 8, 'S', 'P', 0, NomSpeed);
		/* Set maximum acceleration to X inc/s^2 */
		intprtSetInt(h, 8, 'A', 'C', 0, maxAcc);
		/* Set maximum deceleration to X inc/s^2 */
		intprtSetInt(h, 8, 'D', 'C', 0, maxDcc);
		break;

	case MOTIONTYPE_VELCTRL:
		harmonicaStop(h);
		/* switch unit mode */
		intprtSetInt(h, 8, 'U', 'M', 0, 2);
		/* set profiler mode (use AC, DC & SF) */
		intprtSetInt(h, 8, 'P', 'M', 0, 1);
		/* Use internal reference mode */
		intprtSetInt(h, 8, 'R', 'M', 0, 0);

		/* Set maximum acceleration to X inc/s^2 */
		intprtSetInt(h, 8, 'A', 'C', 0, maxAcc);
		/* Set maximum deceleration to X inc/s^2 */
		intprtSetInt(h, 8, 'D', 'C', 0, maxDcc);
		break;
	default:
		return -1;
	}
	mssleep(20);
	return 0;
}

/*----------------------------------------------------------------------*/
/*
 * Request status from the motor contoller
 */
static void
harmonicaRequestStatus(CAN_HARMONICA_STR *h)
{
	intprtSetInt(h, 4, 'S', 'R', 0, 0);
}

void
harmonicaRequestCurrent(CAN_HARMONICA_STR *h)
{
	intprtSetInt(h, 4, 'I', 'Q', 0, 0);
}

void
harmonicaRequestDigIn(CAN_HARMONICA_STR *h)
{
	intprtSetInt(h, 4, 'I', 'P', 0, 0);
}

/*----------------------------------------------------------------------*/
/*
 * Compute a velocity estimation, based on the position difference
 */
static double
estimVel(CAN_HARMONICA_STR *h, double pos)
{
	double vel;
	double dt;

	clock_gettime(CLOCK_REALTIME, h->currentTime);
	dt = absDiffTime(h->currentTime, h->velCalcTime);
	vel = (pos - h->oldPos)/dt;
	h->oldPos = pos;
	clock_gettime(CLOCK_REALTIME, h->velCalcTime);
	return vel;
}

/*----------------------------------------------------------------------*/
/*
 * Interpret the contents of the controller's status register
 */
static int
evalStatusRegister(CAN_HARMONICA_STR *h, int status)
{
	int ret;

	if (isBitSet(status, 0)) {
		if (h->motorState != ST_MOTOR_FAILURE) {
			fprintf(stderr, "Drive %d error: ",
				h->driveParam.driveIdent);
			if ((status & 0x0000000e) == 2)
				fprintf(stderr,
				    " under voltage\n");
			if ((status & 0x0000000e) == 4)
				fprintf(stderr,
				    " over voltage\n");
			if ((status & 0x0000000e) == 10)
				fprintf(stderr,
				    " short circuit\n");
			if ((status & 0x0000000e) == 12)
				fprintf(stderr,
				    " overheating\n");
		}
		h->newMotorState = ST_MOTOR_FAILURE;
		ret = -1;
	} else if (isBitSet(status, 6)) {
		/* Motor failure */
		if (h->motorState != ST_MOTOR_FAILURE) {
			/* request detailed description of failure */
			intprtSetInt(h, 4, 'M', 'F', 0, 0);
			clock_gettime(CLOCK_REALTIME, h->failureStartTime);
		}
		h->newMotorState = ST_MOTOR_FAILURE;
		ret = -1;
	} else {
		/* no error */
		ret = 0;
		/* general status bits */
		if (isBitSet(status, 4)) {
			if (h->motorState != ST_OPERATION_ENABLED) {
				fprintf(stderr, "Motor %x operation enabled\n",
				    h->driveParam.driveIdent);
				clock_gettime(CLOCK_REALTIME,
				    h->failureStartTime);
			}
			h->newMotorState = ST_OPERATION_ENABLED;
		} else {
			if (h->motorState != ST_OPERATION_DISABLED) {
				fprintf(stderr,
				    "Motor %d operation disabled\n",
				    h->driveParam.driveIdent);
			}
			h->newMotorState = ST_OPERATION_DISABLED;
		}
		/* current limit */
		if (isBitSet(status, 13)) {
			if (!h->currentLimitOn) {
				fprintf(stderr, "Motor %d current limit on\n",
				    h->driveParam.driveIdent);
			}
			h->currentLimitOn = true;
		} else {
			h->currentLimitOn = false;
		}
	}
	/* change state */
	h->motorState = h->newMotorState;
	return ret;
}

/*----------------------------------------------------------------------*/
/*
 * Interprets motor failure error codes
 */
static void
evalMotorFailure(CAN_HARMONICA_STR *h, int failure)
{
	fprintf(stderr, "Motor %d: ", h->driveParam.driveIdent);
	if (isBitSet(failure, 2)) {
		fprintf(stderr, "feedback loss\n");
	} else if (isBitSet(failure, 3)) {
		fprintf(stderr, "peak current exceeded\n");
	} else 	if (isBitSet(failure, 7)) {
		fprintf(stderr, "speed track error\n");
	} else 	if (isBitSet(failure, 8)) {
		fprintf(stderr, "position track error\n");
	} else 	if (isBitSet(failure, 17)) {
		fprintf(stderr, "speed limit exceeded\n");
	} else if (isBitSet(failure, 21)) {
		fprintf(stderr, "motor stuck");
	} else if (isBitSet(failure, 12)) {
		switch ((failure & 0xe000) >> 13) {
		case 0:
			fprintf(stderr, "OK\n");
			break;
		case 1:
			fprintf(stderr, "under voltage\n");
			break;
		case 2:
			fprintf(stderr, "over voltage\n");
			break;
		case 5:
			fprintf(stderr, "short circuit\n");
			break;
		case 6:
			fprintf(stderr, "overheating\n");
			break;
		default:
			fprintf(stderr, "reserved error code ??\n");
			break;
		}
	} else {
		fprintf(stderr, "unknown failure 0x%x\n", failure);
	}
}

/*----------------------------------------------------------------------*/
/*
 * Initialize communication with the motor controller
 */
CAN_HARMONICA_STR *
harmonicaInit(int dev, int id,
    CAN_MOTION_TYPE motionType,
    const CAN_DRIVE_PARAMS *driveParam)
{
	CAN_HARMONICA_STR *h;
	CAN_MSG m;
 
	h = (CAN_HARMONICA_STR *)malloc(sizeof(CAN_HARMONICA_STR));
	if (h == NULL) {
		return NULL;
	}
	h->dev = dev;
	h->divForRequestStatus = 10;
	h->canTimeout = 6;
	h->statusCtrl = 0;
	h->posGearMeasRad = 0;
	h->prevPosGearMeasRad = 0;
	h->velGearMeasRadS = 0;
	h->motionType = motionType;
	h->maskIT = 0;
	h->digIn = 0;
	h->homePos = false;
	h->oldPos = 0;
	h->watchdogActive = false;
	h->RxValidEvt.msgIBValid = 0;
	h->RxValidEvt.msgILValid = 0;
	h->RxValidEvt.msgMIValid = false;
	h->RxValidEvt.msgPXValid = false;
	h->currentTime = (struct timespec *)malloc(sizeof(struct timespec));
	h->watchdogTime = (struct timespec *)malloc(sizeof(struct timespec));
	h->velCalcTime = (struct timespec *)malloc(sizeof(struct timespec));
	h->failureStartTime = (struct timespec *)malloc(sizeof(struct timespec));
	h->sendTime = (struct timespec *)malloc(sizeof(struct timespec));

	clock_gettime(CLOCK_REALTIME, h->velCalcTime);

	h->countRequestDiv = 0;

	/* Switch controller to operational mode */
	/* page 7-1 of CANopen DS301 Implementation Guide */
 	canMsgSet(&m, 0, 1, id, 0, 0, 0, 0, 0, 0);
	m.len = 2;
	socketcanTransmitMsg(dev, &m);

	h->motorState = ST_PRE_INITIALIZED;
	h->currentLimitOn = false;

	memcpy(&h->driveParam, driveParam, sizeof(struct CAN_DRIVE_PARAMS));
	h->driveParam.posGearRadToPosMotIncr = (driveParam->encIncrPerRevMot * 
						driveParam->gearRatio * 
						driveParam->beltRatio * 
						Qencode / (2.0 * M_PI));
	h->driveParam.driveIdent = id;

	/* stop all emissions of TPD01 */
	sendSDODownload(h, 0x1a00, 0, 0);

	clock_gettime(CLOCK_REALTIME, h->sendTime);
	clock_gettime(CLOCK_REALTIME, h->currentTime);

	return h;
}

/*----------------------------------------------------------------------*/
/*
 * Ends the commnunication with a motor controller
 */
void
harmonicaEnd(CAN_HARMONICA_STR *h)
{
	harmonicaStop(h);

	free(h->currentTime);
	free(h->watchdogTime);
	free(h->velCalcTime);
	free(h->failureStartTime);
	free(h->sendTime);

	free(h);
}

/*----------------------------------------------------------------------*/
/*
 * request initialization of the motor controller
 */
int
harmonicaInitCtrl(CAN_HARMONICA_STR *h)
{
	CAN_MSG msg;
	int incrRevWheel = (h->driveParam.encIncrPerRevMot * 
			    h->driveParam.gearRatio * 
			    h->driveParam.beltRatio * 
			    Qencode);
	int count,  posCount;
	int ret = 0;

	// printf("-- initCtrl %d\n", h->driveParam.driveIdent);
	h->motorState = ST_PRE_INITIALIZED;

	harmonicaStop(h);
	mssleep(20);
	intprtSetInt(h, 8, 'X', 'M', 2, incrRevWheel * 5000);
	mssleep(20);
	intprtSetInt(h, 8, 'X', 'M', 1, -incrRevWheel * 5000);
	mssleep(20);

	harmonicaSetMotionType(h);

	/* Set position counter to zero */
	intprtSetInt(h, 8, 'P', 'X', 0, 0);
	count = 0;
	while (1) {
		socketcanReceiveMsg(h->dev, &msg);
		if (msg.data[0] == 'P' && msg.data[1] == 'X') {
			posCount = canMsgGetInt(&msg, 4);
			h->posGearMeasRad = (h->driveParam.sign * 
					     driveParamPosMotIncrToPosGearRad(&h->driveParam, posCount));
			h->prevPosGearMeasRad = h->posGearMeasRad;
			break;
		}
		if (count > 300) {
			printf("drive %d: initial position not set\n",
			    h->driveParam.driveIdent);
			ret = -1;
			break;
		}
		mssleep(10);
		count++;
	}

	/*
	 * Set PDO mapping :
	 * TPD01:
	 *  - position
	 *  - velocity
	 */
	/* stop all emissions of TPD01 */
	sendSDODownload(h, 0x1a00, 0, 0);

	/* position 4 byte of TPD01 */
	sendSDODownload(h, 0x1a00, 1, 0x60640020);

	/* velocity 4 byte of TPD01 */
	sendSDODownload(h, 0x1a00, 2, 0x60690020);

	/* transmission type "synch" */
	sendSDODownload(h, 0x1800, 2, 1);

	/* activate mapped objects */
	sendSDODownload(h, 0x1a00, 0, 2);

	return ret;
}

/*----------------------------------------------------------------------*/
/*
 * start the motor controller
 */
int
harmonicaStart(CAN_HARMONICA_STR *h)
{
	CAN_MSG m;
	struct timespec ts;
	int cnt, rc;

	/* start motor */
	// printf("-- start motor %d\n", h->driveParam.driveIdent);
	intprtSetInt(h, 8, 'M', 'O', 0, 1);
	mssleep(20);
	/* flush CAN bus */
	while (!socketcanReceiveMsg(h->dev, &m));

	harmonicaRequestStatus(h);

	/* wait a most 10s for motor to become enabled */
	cnt = 0;
	while (h->motorState != ST_OPERATION_ENABLED && cnt < 10) {
		socketcanReceiveMsgWait(h->dev, &m, 1000);
		if (m.data[0] == 'S' && m.data[1] == 'R') {
			h->statusCtrl = canMsgGetInt(&m, 4);
			evalStatusRegister(h, h->statusCtrl);
			break;
		}
		cnt++;
		clock_gettime(CLOCK_REALTIME, &ts);
		ts.tv_sec++;
		do {
			rc = clock_nanosleep(CLOCK_REALTIME,
			    TIMER_ABSTIME, &ts, NULL);
		} while (rc == -1 && errno == EINTR);
	} /* while */
	if (h->motorState != ST_OPERATION_ENABLED) {
		return -1;
	}
	clock_gettime(CLOCK_REALTIME, h->watchdogTime);
	clock_gettime(CLOCK_REALTIME, h->sendTime);
	return 0;
}

/*----------------------------------------------------------------------*/
/*
 * shut down the motor controller
 */
void
harmonicaStop(CAN_HARMONICA_STR *h)
{
	/* Motor off */
	intprtSetInt(h, 8, 'M', 'O', 0, 0);
}

/*----------------------------------------------------------------------*/
/*
 * reset the motor controller
 */
int
harmonicaReset(CAN_HARMONICA_STR *h)
{
	CAN_MSG msg;

	printf("Resetting drive %d\n", h->driveParam.driveIdent);
	canMsgSet(&msg, 0, 1, h->driveParam.driveIdent, 0, 0, 0, 0, 0, 0);
	msg.len = 2;
	socketcanTransmitMsg(h->dev, &msg);

	/* Init motor */
	if (harmonicaInitCtrl(h) == -1) {
		return -1;
	}
	/* Start motor */
	if (harmonicaStart(h) == -1) {
		return -1;
	}
	return 0;
}

/*----------------------------------------------------------------------*/
/*
 * Activate the hardware watchdog
 */
int
harmonicaStartWatchdog(CAN_HARMONICA_STR *h, bool started)
{

	if (started) {
		h->watchdogActive = true;
		/* consumer (PC) heartbeat time */
		sendSDODownload(h, 0x1016, 1, (NMTNodeID << 16) | HeartbeatTimeMS);
		/* error behaviour after heartbeat failure: "CAN communication stopped" */
		sendSDODownload(h, 0x1029, 1, 2);
		/* motor behaviour after heartbeat failure: "quick stop" */
		sendSDODownload(h, 0x6007, 0, 3);
		/* activate emergency events: "heartbeat event" */
		sendSDODownload(h, 0x2f21, 0, 0x8);
		mssleep(20);
	} else {
		/* Stop watchdog */
		h->watchdogActive = false;
		/* error behaviour after heartbeat failure: "No state change" */
		sendSDODownload(h, 0x1029, 1, 1);
		/* motor behaviour after heartbead failure: no action */
		sendSDODownload(h, 0x6007, 0, 0);
		/* de-activiate emergency events */
		sendSDODownload(h, 0x2f21, 0, 0x0);
		mssleep(20);
	}
	return 0;
}

/*----------------------------------------------------------------------*/
/*
 * Request a given position and velocity and ask for data updates
 */
void
harmonicaSetGearPosVelRadS(CAN_HARMONICA_STR *h, double posGearRad, double velGearRadS)
{
	int posEncIncr;
	int velEncIncrPeriod;
	CAN_DRIVE_PARAMS *p = &h->driveParam;
	CAN_MSG msg;
	double dt;

	driveParamPosVelRadToIncr(p, posGearRad, velGearRadS, &posEncIncr, &velEncIncrPeriod);
	posEncIncr *= driveParamGetSign(p);
	if (velEncIncrPeriod > driveParamGetVelMax(p)) {
		velEncIncrPeriod = driveParamGetVelMax(p);
	}
	if (velEncIncrPeriod < -driveParamGetVelMax(p)) {
		velEncIncrPeriod = -driveParamGetVelMax(p);
	}
	h->refVelGearIncS = velEncIncrPeriod;
	if (h->motionType == MOTIONTYPE_POSCTRL) {
		intprtSetInt(h, 8, 'S', 'P', 0, velEncIncrPeriod);
		intprtSetInt(h, 8, 'P', 'A', 0, posEncIncr);
		intprtSetInt(h, 4, 'B', 'G', 0, 0);
	} else if (h->motionType == MOTIONTYPE_VELCTRL) {
		velEncIncrPeriod *= driveParamGetSign(p);
		intprtSetInt(h, 8, 'J', 'V', 0, velEncIncrPeriod);
		intprtSetInt(h, 4, 'B', 'G', 0, 0);
	}
	/* send heartbeat */
	msg.id = 0x700;
	msg.len = 5;
	msg.type = 0;
	memset(msg.data, 0, 8);
	socketcanTransmitMsg(h->dev, &msg);

	/* request pos and vel by TPDO1, triggerd by SYNC msg */
	canMsgSet(&msg, 0x80, 0, 0, 0, 0, 0, 0, 0, 0);
	msg.len = 0;

	socketcanTransmitMsg(h->dev, &msg);

	clock_gettime(CLOCK_REALTIME, h->currentTime);
	dt = absDiffTime(h->currentTime, h->sendTime);
	if (dt > 1.0) {
		fprintf(stderr,
		    "Time between send velocity of motor %x is too large %f\n",
		    driveParamGetDriveIdent(p), dt);
	}
	clock_gettime(CLOCK_REALTIME, h->sendTime);
	/* request status */
	h->countRequestDiv++;
	if (h->countRequestDiv > h->divForRequestStatus) {
		harmonicaRequestStatus(h);
		h->countRequestDiv = 0;
	}
}

/*----------------------------------------------------------------------*/
/*
 * request a given velocity, and ask for data updates
 * also ask for status every divForRequestStatus calls
 */
void
harmonicaSetGearVelRadS(CAN_HARMONICA_STR *h, double vel)
{
	int velEncIncrPeriod;
	CAN_DRIVE_PARAMS *p = &h->driveParam;
	CAN_MSG m;
	double dt;

	velEncIncrPeriod = p->sign
	    * driveParamVelGearRadSToVelMotIncrPeriod(p, vel);

	if (velEncIncrPeriod > driveParamGetVelMax(p))
		velEncIncrPeriod = driveParamGetVelMax(p);
	if (velEncIncrPeriod < -driveParamGetVelMax(p))
		velEncIncrPeriod = -driveParamGetVelMax(p);
	h->refVelGearIncS = velEncIncrPeriod;

	intprtSetInt(h, 8, 'J', 'V', 0, velEncIncrPeriod);
	intprtSetInt(h, 4, 'B', 'G', 0, 0);

	/* request pos and vel by TPDO1, triggered by SYNC msg */
	m.id = COB_SYNC;//0x80;
	m.len = 0;
	m.type = 0;
	memset(m.data, 0, 8);
	socketcanTransmitMsg(h->dev, &m);

	/* send heartbeat */
	m.id = 0x700;
	m.len = 5;
	m.type = 0;
	memset(m.data, 0, 8);
	socketcanTransmitMsg(h->dev, &m);

	clock_gettime(CLOCK_REALTIME, h->currentTime);
	dt = absDiffTime(h->currentTime, h->sendTime);
	if (h->watchdogActive && dt > 1.0) {
		fprintf(stderr,
		    "Time between send velocity of motor %d is too large %f\n",
		    driveParamGetDriveIdent(p), dt);
	}
	clock_gettime(CLOCK_REALTIME, h->sendTime);
	/* request status */
	h->countRequestDiv++;
	if (h->countRequestDiv > h->divForRequestStatus) {
		harmonicaRequestStatus(h);
		h->countRequestDiv = 0;
	}
}

/*----------------------------------------------------------------------*/
/*
 * request a given velocity, and ask for data updates
 * also ask for status every divForRequestStatus calls
 */
void
harmonicaSetGearVelIncr(CAN_HARMONICA_STR *h, int velEncIncrPeriod)
{
	CAN_DRIVE_PARAMS *p = &h->driveParam;
	CAN_MSG m;
	double dt;

	if (velEncIncrPeriod > driveParamGetVelMax(p)) {
		velEncIncrPeriod = driveParamGetVelMax(p);
		printf("limit %d > %d\n",
		    velEncIncrPeriod, driveParamGetVelMax(p));
	}
	if (velEncIncrPeriod < -driveParamGetVelMax(p)) {
		velEncIncrPeriod = -driveParamGetVelMax(p);
		printf("limit %d < %d\n",
		    velEncIncrPeriod, -driveParamGetVelMax(p));
	}
	h->refVelGearIncS = velEncIncrPeriod;

	intprtSetInt(h, 8, 'J', 'V', 0, velEncIncrPeriod);
	intprtSetInt(h, 4, 'B', 'G', 0, 0);

	/* request pos and vel by TPDO1, triggered by SYNC msg */
	m.id = 0x80;
	m.len = 0;
	m.type = 0;
	memset(m.data, 0, 8);
	socketcanTransmitMsg(h->dev, &m);

	/* send heartbeat */
	m.id = 0x700;
	m.len = 5;
	m.type = 0;
	memset(m.data, 0, 8);
	socketcanTransmitMsg(h->dev, &m);

	clock_gettime(CLOCK_REALTIME, h->currentTime);
	dt = absDiffTime(h->currentTime, h->sendTime);
	if (dt > 1.0) {
		fprintf(stderr,
		    "Time between send velocity of motor %d is too large %f\n",
		    driveParamGetDriveIdent(p), dt);
	}
	clock_gettime(CLOCK_REALTIME, h->sendTime);
	/* request status */
	h->countRequestDiv++;
	if (h->countRequestDiv > h->divForRequestStatus) {
		harmonicaRequestStatus(h);
		h->countRequestDiv = 0;
	}
}

/*----------------------------------------------------------------------*/

void
harmonicaGetPosRad(CAN_HARMONICA_STR *h, double *pPos)
{
	*pPos = h->posGearMeasRad;
}

void
harmonicaGetPosVelRadS(CAN_HARMONICA_STR *h, double *pPos, double *pVel)
{
	*pPos = h->posGearMeasRad;
	*pVel = h->velGearMeasRadS;
}

void
harmonicaGetDeltaPosVelRadS(CAN_HARMONICA_STR *h,
    double *pDeltaPos, double *pVel)
{
	*pDeltaPos = h->posGearMeasRad - h->prevPosGearMeasRad;
	*pVel = h->velGearMeasRadS;
	h->prevPosGearMeasRad = h->posGearMeasRad;
}

CAN_MOTOR_STATE
harmonicaGetMotorState(CAN_HARMONICA_STR *h)
{
	return h->motorState;
}

void
harmonicaGetCurrent(CAN_HARMONICA_STR *h, double *pCurr)
{
	*pCurr = h->motorCurr;
}

/*----------------------------------------------------------------------*/
/*
 * Display status of a motor controller on stdout
 */
void
harmonicaShow(CAN_HARMONICA_STR *h)
{
	printf("driveIdent: %d\n", driveParamGetDriveIdent(&h->driveParam));
	(h->motionType) ? printf("motionType: POSITION CTRL\n") : printf("motionType: VELOCITY CTRL\n");
	printf("statusCtrl: 0x%08x\n", h->statusCtrl);
	printf("ref velocity: %d (%f)\n", h->refVelGearIncS,
	       driveParamVelMotIncrPeriodToVelGearRadS(&h->driveParam,
						       h->refVelGearIncS));
	printf("velocity (rad/S): %f\n", h->velGearMeasRadS);
	printf("position (rad): %f\n", h->posGearMeasRad);
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
	printf("current limit: %s\n", h->currentLimitOn ? "On" : "Off");
	printf("------------------------------------------------------\n");
};

/*----------------------------------------------------------------------*/

/*
 * Decode CAN messages sent by the motor controller
 *
 */

#define HARMONICA_MSG(a,b) ((a)<<8|(b))

bool
harmonicaDecode(CAN_HARMONICA_STR *h, CAN_MSG *msg)
{
	CAN_DRIVE_PARAMS *p = &h->driveParam;  
	int temp1, temp2, posCnt, failure, para;
	bool ret = false;
  
	if (h == NULL)
		return false;
  
	if (msg->id == txPDO1(h->driveParam)) {
	/* Position and velocity update */
		temp1 = canMsgGetInt(msg, 0);
   		 h->posGearMeasRad = (driveParamGetSign(p) * driveParamPosMotIncrToPosGearRad(p, temp1));
    		temp2 = canMsgGetInt(msg, 4);
    		h->velGearMeasRadS = (driveParamGetSign(p) * driveParamVelMotIncrPeriodToVelGearRadS(p, temp2));
		clock_gettime(CLOCK_REALTIME, h->watchdogTime);
   	 	ret = true;
  	}
	if (msg->id == txPDO2(h->driveParam)) {
		para = canMsgGetInt(msg, 4);
		//h->RxValidEvt.msgPXValid = false;
    		switch(((msg->data[0]) << 8) | (msg->data[1])) {
		case HARMONICA_MSG('I', 'L') :
			/* IO configuration status */
				if((msg->data[2] == 5) && ((para & IL_FCT_MASK) == IL_FCT_HARD_EN_RLS) && (para & IL_ACTIVE_MASK))
					(h->RxValidEvt.msgILValid |= 1);
				if((msg->data[2] == 6) && ((para & IL_FCT_MASK) == IL_FCT_HARD_EN_FLS) && (para & IL_ACTIVE_MASK))
					(h->RxValidEvt.msgILValid |= 2);
				break;
		case HARMONICA_MSG('I', 'P') :
			/* digital in -> limit switches */
				(para & IP_FLS_FLAG) ? (h->RxValidEvt.msgIBValid |= 1) : (h->RxValidEvt.msgIBValid &= 2);
				(para & IP_RLS_FLAG) ? (h->RxValidEvt.msgIBValid |= 2) : (h->RxValidEvt.msgIBValid &= 1);
				break;
		case HARMONICA_MSG('I', 'Q') :
			/* Read Reactive Current */
				h->motorCurr = canMsgGetFloat(msg, 4);
	     			break;
		case HARMONICA_MSG('M', 'F') :
			/* motor failure */
				failure = para;
				evalMotorFailure(h, failure);
	      			break;
		case HARMONICA_MSG('M', 'I') :
	      		/* Mask Interrupt */
				h->maskIT = !(h->RxValidEvt.msgMIValid) ? para : h->maskIT;
				h->RxValidEvt.msgMIValid = true;
	      			break;
		case HARMONICA_MSG('P', 'X') :
	      		/* current/main position */
	      			posCnt = para;
				h->posGearMeasRad = driveParamPosMotIncrToPosGearRad(p, posCnt);
	      			h->velGearMeasRadS = estimVel(h, h->posGearMeasRad);
	      			h->RxValidEvt.msgPXValid = true;
	      			break;
		case HARMONICA_MSG('S', 'R') :
	      		/* status register */
	      			h->statusCtrl = para;
	      			evalStatusRegister(h, h->statusCtrl);
				h->RxValidEvt.msgSRValid = true;
	     			break;
		case HARMONICA_MSG('U', 'M') :
	      		/* user mode */
	      			printf("-> UM: %d\n", para);
	      			break;
		case HARMONICA_MSG('A', 'C') :
			/* Acceleration */
		case HARMONICA_MSG('B', 'G') :
			/* begin motion */
		case HARMONICA_MSG('D', 'C') :
			/* Deceleration */
		case HARMONICA_MSG('H', 'M') :
			/* homing status message */
		case HARMONICA_MSG('I', 'B') :
			/* digital in -> limit switches */
		case HARMONICA_MSG('J', 'V') :
			/* jogging velocity */
		case HARMONICA_MSG('M', 'O') :
			/* Motor enable/disable */
		case HARMONICA_MSG('P', 'A') :
	      		/* position absolute */
		case HARMONICA_MSG('P', 'M') :
			/* Profiler Mode */
		case HARMONICA_MSG('R', 'M') :
			/* Reference Mode */
		case HARMONICA_MSG('S', 'P') :
	      		/* Speed for PTP mode */
		case HARMONICA_MSG('T', 'R'):
			/* Target Radius */
				break;
			default:
	      			fprintf(stderr, "harmonicaDecode: unknown msg %c%c\n", msg->data[0], msg->data[1]);
	    	}
		clock_gettime(CLOCK_REALTIME, h->watchdogTime);
#ifdef CAN_MSG_DEBUG
		printf("%c%c[%2d]: %2x,%2x,%2x,%2x, %8x(%d) - %fms\n", 
			msg->data[0], msg->data[1], msg->data[2], msg->data[4], 
			msg->data[5], msg->data[6], msg->data[7], para, para, 
			((double)(h->watchdogTime->tv_nsec))/1000000.0);
#endif 	
		ret = true;
  	}
	if (msg->id == txSDO(h->driveParam)) {
		posCnt = driveParamGetSign(p)*canMsgGetInt(msg, 4);
		h->posGearMeasRad = driveParamPosMotIncrToPosGearRad(p, posCnt);
		h->velGearMeasRadS = estimVel(h, h->posGearMeasRad);
		clock_gettime(CLOCK_REALTIME, h->watchdogTime);
		ret = true;
	}
	return ret;
}

