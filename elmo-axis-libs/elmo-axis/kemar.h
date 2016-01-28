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
#ifndef _KEMAR_H
#define _KEMAR_H

#define MASK_IT_ALL_DISABLE     (0xFFFF)
#define MASK_IT_ALL_ENABLE      (0x0000)

#define HM_1_DISARM_HOM_PROC    (0x00)
#define HM_1_ARM_HOM_PROC       (0x01)

#define HM_3_EVT_IMMEDIATE      (0)
#define HM_3_EVT_FLS_SW         (5)
#define HM_3_EVT_RLS_SW         (7)

#define HM_4_AFT_EVT_STOP       (0)
#define HM_4_AFT_EVT_SET_DOUT   (1)
#define HM_4_AFT_EVT_NOTHING    (2)      

#define HM_5_SET_PX_ABSOLUTE    (0)
#define HM_5_SET_PX_RELATIVE    (1)
#define HM_5_SET_PX_NOTHING     (2)

#define MO_MOTOR_DISABLE        (0)
#define MO_MOTOR_ENABLE         (1)

#define IL_ACTIVE_LOW           (0 << 0)
#define IL_ACTIVE_HIGH          (1 << 0)
#define IL_ACTIVE_MASK          0x0001

#define IL_FCT_HARD_EN_RLS      (4 << 1)
#define IL_FCT_HARD_EN_FLS      (5 << 1)
#define IL_FCT_MASK             0x001e

#define IP_FLS_FLAG             (1 << 10)
#define IP_RLS_FLAG             (1 << 11)

#define MSG_REQ_DELAY_MS        250
#define DIFF_VALID_MVT_S        10.0
#define DIFF_VALID_MSG_S        2.0
#define DIFF_VALID_POS_INCR     4.0          

#define CAN_ID                  "can1"
#define HARMONICA_ID             0x7F

/* motor state */
typedef enum HOMING_STATE {
	FLS,
	RLS,
	TARGET,
	ZERO,
} HOMING_STATE;

typedef enum MSG_HEADER {
	MSG_IB_FLS,
	MSG_IB_RLS,
	MSG_IL,
	MSG_MI,
	MSG_PULL,
	MSG_PX,
	MSG_PX_TARGET,
	MSG_SR,
} MSG_HEADER;

/* Structure describing head velocity & position */
typedef struct KEMAR_POS_VEL_STR {

//	int leftIncrMax;	/* max increment up to RLS switch detected */
//	int rightIncrMax;	/* max increment up to FLS switch detected */
//	double leftRadMax;	/* max val in rad up to RLS switch detected */
//	double rightRadMax;	/* max val in rad up to FLS switch detected */

	int posTargetEncIncr;
	double posTargetGearRad;

	int velTargetEncIncrPeriod;
	double velTargetGearRadS;

	int posEncIncr[2];
	int velEncIncrPeriod;
	double posGearRad[2];
	double velGearRadS;
/*	
	struct timespec *currentTime;
	struct timespec *velCalcTime;
	struct timespec *sendTime;
*/
} KEMAR_POS_VEL_STR;

/*
extern CAN_HARMONICA_STR* kemarInit(CAN_MOTION_TYPE);
extern int kemarHoming(CAN_HARMONICA_STR *);
extern int kemarHomingRegConfig(CAN_HARMONICA_STR *, HOMING_STATE);
extern int kemarSwitchesInit(CAN_HARMONICA_STR *);
extern int kemarWaitMsgValid(CAN_HARMONICA_STR *, MSG_HEADER, double);

extern KEMAR_POS_VEL_STR * kemarStructInit(CAN_HARMONICA_STR *);
extern void kemarSetGearPosVelRadS(CAN_HARMONICA_STR *, KEMAR_POS_VEL_STR *, double , double);
extern void kemarSetGearPosRad(CAN_HARMONICA_STR *, KEMAR_POS_VEL_STR *, double);
extern void kemarStructEnd(KEMAR_POS_VEL_STR *);
*/
extern int kemarHoming(CAN_HARMONICA_STR *);
extern int kemarHomingRegConfig(CAN_HARMONICA_STR *, HOMING_STATE);
extern CAN_HARMONICA_STR * kemarInit(CAN_MOTION_TYPE);
extern int kemarSwitchesInit(CAN_HARMONICA_STR *);
extern int kemarWaitMsgValid(CAN_HARMONICA_STR *, MSG_HEADER, double);

extern KEMAR_POS_VEL_STR * kemarStructInit(CAN_HARMONICA_STR *);
extern void kemarStructEnd(KEMAR_POS_VEL_STR *);
extern void kemarSetGearPosAbsRad(CAN_HARMONICA_STR *, KEMAR_POS_VEL_STR *, double);
extern void kemarSetGearPosRelRad(CAN_HARMONICA_STR *, KEMAR_POS_VEL_STR *, double);
extern void kemarSetGearVelRadS(CAN_HARMONICA_STR *, KEMAR_POS_VEL_STR *, double, CAN_MOTION_TYPE);
extern void kemarSetEncPosAbsIncr(CAN_HARMONICA_STR *, KEMAR_POS_VEL_STR *, int);
extern void kemarSetEncPosRelIncr(CAN_HARMONICA_STR *, KEMAR_POS_VEL_STR *, int);
extern void kemarSetEncVelIncrS(CAN_HARMONICA_STR *, KEMAR_POS_VEL_STR *, int, CAN_MOTION_TYPE);
extern void kemarSetGearPosAbsVelRadS(CAN_HARMONICA_STR *, KEMAR_POS_VEL_STR *, double, double);
extern void kemarSetGearPosRelVelRadS(CAN_HARMONICA_STR *, KEMAR_POS_VEL_STR *, double, double);
extern void kemarSetEncPosAbsVelIncrS(CAN_HARMONICA_STR *, KEMAR_POS_VEL_STR *, int, int);
extern void kemarSetEncPosRelVelIncrS(CAN_HARMONICA_STR *, KEMAR_POS_VEL_STR *, int, int);
extern CAN_MOTION_TYPE kemarSetMotionType(CAN_HARMONICA_STR *, CAN_MOTION_TYPE);
//extern int kemarGetPosRad(CAN_HARMONICA_STR *);
extern void kemarGetInfo(CAN_HARMONICA_STR *, KEMAR_POS_VEL_STR *);
extern void kemarShow(CAN_HARMONICA_STR *, KEMAR_POS_VEL_STR *);
#endif
