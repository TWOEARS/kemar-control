/*
 * Copyright (c) 2005,2011 CNRS/LAAS
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
#ifndef _HARMONICA_H
#define _HARMONICA_H

/* CAN Ids of Communication object types */
/* CANopen DS 301 Implementation Guide page 2-4 */
#define COB_NMT		0x0
#define COB_SYNC	0x080
#define COB_TIMESTAMP	0x100
#define COB_EMCY(dp)	(0x01 << 7 | ((dp).driveIdent))
#define txPDO1(dp)	(0x03 << 7 | ((dp).driveIdent))
#define rxPDO1(dp)	(0x04 << 7 | ((dp).driveIdent))
#define txPDO2(dp)	(0x05 << 7 | ((dp).driveIdent))
#define rxPDO2(dp)	(0x06 << 7 | ((dp).driveIdent))
#define txSDO(dp)	(0x0b << 7 | ((dp).driveIdent))
#define rxSDO(dp)	(0x0c << 7 | ((dp).driveIdent))
#define COB_ERROR(dp)	(0x0e << 7 | ((dp).driveIdent))

/* Bits from digital inputs */
#define HARMONICA_INPUT_FORWARD_LIMIT 0x00000400
#define HARMONICA_INPUT_REVERSE_LIMIT 0x00000800

/* structure extract */
#define driveParamGetDriveIdent(p) ((p)->driveIdent)
#define driveParamGetSign(p) ((p)->sign)
#define driveParamGetVelMax(p) ((p)->velMaxEncIncrS)

/* motor state */
typedef enum CAN_MOTOR_STATE {
	ST_PRE_INITIALIZED,
	ST_OPERATION_ENABLED,
	ST_OPERATION_DISABLED,
	ST_MOTOR_FAILURE
} CAN_MOTOR_STATE;

/* Motion type of the controller */
typedef enum CAN_MOTION_TYPE {
	MOTIONTYPE_VELCTRL,
	MOTIONTYPE_POSCTRL
} CAN_MOTION_TYPE;

/* Misc drive parameters */
typedef struct CAN_DRIVE_PARAMS {
	int8_t driveIdent;	/* drive identifier 0x00..0x7F */
	int encIncrPerRevMot;	/* encoder increments per revolution motor shaft */
	double gearRatio;
	double beltRatio;	/* if drive has belt set ratio, else 1 */
	int sign;		/* direction of motion */
	int velMaxEncIncrS;	/* max velocity */
	int accIncrS;		/* max acceleration */
	int decIncrS;		/* max decceleration */
	double posGearRadToPosMotIncr;
	int encOffsetIncr;	/* position in increments of steerwheel at homing position */
	int homingDigIn;	/* specifies which digital input is used for homing signal */
	int homingSpeed;	/* speed to use for homing (incr/s) */
	int homingDelay;	/* delay to allow for homing (s) */
	int leftIncrMax;	/* max increment up to RLS switch detected */
	int rightIncrMax;	/* max increment up to FLS switch detected */
	double leftRadMax;	/* max val in rad up to RLS switch detected */
	double rightRadMax;	/* max val in rad up to FLS switch detected */
	double currToTorque;	/* factor to convert motor active current [A] into torque [Nm] */
	double currMax;		/* max. current allowed */
} CAN_DRIVE_PARAMS;

/* Messages reception flags */
typedef struct RX_VALID_EVENTS {
	unsigned int msgIBValid : 2;
	unsigned int msgILValid : 2;
	unsigned int msgMIValid : 1;
	unsigned int msgPXValid : 1;
	unsigned int msgSRValid : 1;
} RX_VALID_EVENTS;

/* Structure describing one particular motor controller */
typedef struct CAN_HARMONICA_STR {
	int dev;		/* socket can file descriptor */
	struct CAN_DRIVE_PARAMS driveParam;
	int divForRequestStatus;
	double canTimeout;
	CAN_MOTION_TYPE motionType;
	int statusCtrl;
	int maskIT;
	bool watchdogActive;
	struct RX_VALID_EVENTS RxValidEvt;
	struct timespec *currentTime;
	struct timespec *watchdogTime;
	struct timespec *velCalcTime;
	struct timespec *failureStartTime;
	struct timespec *sendTime;
	int refVelGearIncS;	/* reference velocity */
  	double velGearMeasRadS;
	double posGearMeasRad;
	double prevPosGearMeasRad;
	u_int32_t digIn;
	bool homePos;		/* Reached home position  */
	double oldPos;
	int motorState;
	int newMotorState;
	int countRequestDiv;
	bool currentLimitOn;
	double motorCurr;	/* current (A) */
} CAN_HARMONICA_STR;

extern int intprtSetInt(CAN_HARMONICA_STR *, int, char, char, int, int);
extern CAN_HARMONICA_STR *harmonicaInit(int, int, CAN_MOTION_TYPE, const CAN_DRIVE_PARAMS *);
extern void harmonicaEnd(CAN_HARMONICA_STR *);
extern int harmonicaStart(CAN_HARMONICA_STR *);
extern void harmonicaStop(CAN_HARMONICA_STR *);
extern int harmonicaInitCtrl(CAN_HARMONICA_STR *);
extern int harmonicaReset(CAN_HARMONICA_STR *);
extern int harmonicaSetMotionType(CAN_HARMONICA_STR *);
extern int harmonicaStartWatchdog(CAN_HARMONICA_STR *, bool);
extern void harmonicaRequestDigIn(CAN_HARMONICA_STR *);
extern void harmonicaRequestCurrent(CAN_HARMONICA_STR *);
extern void harmonicaSetGearVelRadS(CAN_HARMONICA_STR *, double);
extern void harmonicaSetGearPosVelRadS(CAN_HARMONICA_STR *, double, double);
extern void harmonicaGetPosRad(CAN_HARMONICA_STR *, double *);
extern void harmonicaGetPosVelRadS(CAN_HARMONICA_STR *, double *, double *);
extern void harmonicaGetDeltaPosVelRadS(CAN_HARMONICA_STR *, double *, double *);
extern void harmonicaGetCurrent(CAN_HARMONICA_STR *, double *);
extern CAN_MOTOR_STATE harmonicaGetMotorState(CAN_HARMONICA_STR *);
extern bool harmonicaDecode(CAN_HARMONICA_STR *, CAN_MSG *);
extern void harmonicaShow(CAN_HARMONICA_STR *);

extern void driveParamInit(CAN_DRIVE_PARAMS *, int);
extern void driveParamPosVelRadToIncr(CAN_DRIVE_PARAMS *, double, double, int *, int *x);
extern int driveParamPosGearRadToPosMotIncr(CAN_DRIVE_PARAMS *, double);
extern double driveParamPosMotIncrToPosGearRad(CAN_DRIVE_PARAMS *, int);
extern int driveParamVelGearRadSToVelMotIncrPeriod(CAN_DRIVE_PARAMS *, double);
extern double driveParamVelMotIncrPeriodToVelGearRadS(CAN_DRIVE_PARAMS *, int);

#endif
