/*  Copyright (c) 2015, LAAS/CNRS
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, 
 *    this list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice, 
 *    this list of conditions and the following disclaimer in the documentation 
 *    and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the names of its contributors 
 *    may be used to endorse or promote products derived from this software 
 *    without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE 
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include "ackemar.h"

#include "kemar_c_types.h"

#include "includes.h"

/* --- Task motion ------------------------------------------------------ */


/** Codel motionStart of task motion.
 *
 * Triggered by kemar_start.
 * Yields to kemar_ether.
 */
genom_event
motionStart(kemar_ids *ids, const kemar_Indexes *Indexes,
            genom_context self)
{
    /* init kemar head */
    h = kemarInit(MOTIONTYPE_POSCTRL);
    p = &h->driveParam;
    firstHoming = 0;

    Indexes->data(self)->startPosition = 0;
    Indexes->data(self)->startTimeStamp.sec = 0;
    Indexes->data(self)->startTimeStamp.usec = 0;
    Indexes->data(self)->stopPosition = 0;
    Indexes->data(self)->stopTimeStamp.sec = 0;
    Indexes->data(self)->stopTimeStamp.usec = 0;
    Indexes->write(self);

    ids->headSpeed=100;

    return kemar_ether;
}


/** Codel motionStop of task motion.
 *
 * Triggered by kemar_stop.
 * Yields to kemar_ether.
 */
genom_event
motionStop(genom_context self)
{
    kemarStructEnd(k);	
    harmonicaStop(h);
    harmonicaEnd(h);
    socketcanEnd(h->dev);

    return kemar_ether;
}


/* --- Activity Homing -------------------------------------------------- */

/** Codel hStart of activity Homing.
 *
 * Triggered by kemar_start.
 * Yields to kemar_sendH.
 */
genom_event
hStart(genom_context self)
{
    flagH=0;
    stepH=0;

    if(firstHoming==1)
    {
        kemarStructEnd(k);	
        harmonicaEnd(h);
        socketcanEnd(h->dev);
        /* init kemar head */
        h = kemarInit(MOTIONTYPE_POSCTRL);
        p = &h->driveParam;
    }
    return kemar_sendH;
}

/** Codel hSend of activity Homing.
 *
 * Triggered by kemar_sendH.
 * Yields to kemar_recvH, kemar_ether.
 */
genom_event
hSend(kemar_ids *ids, genom_context self)
{
    if(flagH==0)
    {
        printf("Kemar Homing [%d]\n", stepH+1);
        return kemar_recvH;
    }
    else
    {
        flagH=0;
        stepH++;
        if(stepH<10)
        {
            //printf("Kemar Homing [%d]\n", stepH+1);
            return kemar_recvH;
        }
        else
        {
            printf("init kemar head struct using homing init\n");
        	/* init kemar head struct using homing init */
        	k = kemarStructInit(h);
            /*Sets Velocity to a 'default' value = 100deg/sec*/
            kemarSetGearVelRadS(h, k, (ids->headSpeed*(pi/180)), MOTIONTYPE_POSCTRL);
            printf("Sets velocity to default value of %2.2f deg/sec\n", ids->headSpeed);
            return kemar_ether;
        }        
    }
}

/** Codel hWaitForData of activity Homing.
 *
 * Triggered by kemar_recvH.
 * Yields to kemar_sendH, kemar_recvH, kemar_ether.
 */
genom_event
hWaitForData(genom_context self)
{
    switch(stepH)
    {
        case 0:
            printf("Kemar Homing waits [%d]: ", stepH+1);
            printf("Set DIN#5 & DIN#6 as RLS & FLS switchs (active high)\n");
	        /* Set DIN#5 & DIN#6 as RLS & FLS switchs (active high) */
	        if(kemarSwitchesInit(h))
            {
                printf("{Set DIN#5 & DIN#6 as RLS & FLS switchs (active high)} failed\n");
		        return kemar_ether;
            }
            flagH=1;
            return kemar_sendH;
        case 1:
            printf("Kemar Homing waits [%d]: ", stepH+1);
            printf("Request Mask IT & save previous mask & disable all IT\n");
	        /* Request Mask IT & save previous mask & disable all IT */
	        if(kemarWaitMsgValid(h, MSG_MI, 0.0))
            {
                printf("{Request Mask IT & save previous mask & disable all IT} failed\n");
		        return kemar_ether;
            }
            intprtSetInt(h, 8, 'M', 'I', 0, MASK_IT_ALL_DISABLE);
            flagH=1;
            return kemar_sendH;
        case 2:
            printf("Kemar Homing waits [%d]: ", stepH+1);
            printf("Configure homing for FLS event & wait FLS reached\n");
	        /* Configure homing for FLS event & wait FLS reached */
	        if(kemarHomingRegConfig(h, FLS))
            {
                printf("{Configure homing for FLS event & wait FLS reached} failed\n");
		        return kemar_ether;
            }
            flagH=1;
            return kemar_sendH;
        case 3:
            printf("Kemar Homing waits [%d]: ", stepH+1);
            printf("Configure homing for RLS event & wait RLS reached\n");
	        /* Configure homing for RLS event & wait RLS reached */
	        if(kemarHomingRegConfig(h, RLS))
            {
                printf("{Configure homing for RLS event & wait RLS reached} failed\n");
		        return kemar_ether;
            }
            flagH=1;
            return kemar_sendH;
        case 4:
            printf("Kemar Homing waits [%d]: ", stepH+1);
            printf("Request absolute position & save it\n");
	        /* Request absolute position & save it */
	        if(kemarWaitMsgValid(h, MSG_PX, 0.0))
            {
                printf("{Request absolute position & save it} failed\n");
		        return kemar_ether;
            }
            flagH=1;
            return kemar_sendH;
        case 5:
            printf("Kemar Homing waits [%d]: ", stepH+1);
            printf("Go to central position -> PX/2 + start motion & wait center reached\n");
	        /* Go to central position -> PX/2 + start motion & wait center reached */
	        if(kemarHomingRegConfig(h, TARGET))
            {
                printf("{Go to central position -> PX/2 + start motion & wait center reached} failed\n");
		        return kemar_ether;
            }
            mssleep(500);
            flagH=1;
            return kemar_sendH;
        case 6:
            printf("Kemar Homing waits [%d]: ", stepH+1);
            printf("Reset PX\n");
	        /* Reset PX */
	        if(kemarHomingRegConfig(h, ZERO))
            {
                printf("{Reset PX} failed\n");
		        return kemar_ether;
            }
            flagH=1;
            return kemar_sendH;
        case 7:
            printf("Kemar Homing waits [%d]: ", stepH+1);
            printf("Set  AUTO_IT routine to previous version\n");
	        /* Set  AUTO_IT routine to previous version */
	        intprtSetInt(h, 8, 'M', 'I', 0, h->maskIT);
            flagH=1;
            return kemar_sendH;
        case 8:
            printf("Kemar Homing waits [%d]: ", stepH+1);
            printf("Purge incoming messages\n");
        	/* Purge incoming messages */
	        kemarWaitMsgValid(h, MSG_PULL, 0.0);
            flagH=1;
            return kemar_sendH;
        case 9:
            printf("Kemar Homing waits [%d]: ", stepH+1);
            printf("homing done\n");
	        /* homing done */
	        h->homePos = true;
            if(firstHoming==0)
                firstHoming = 1;
            flagH=1;
            return kemar_sendH;
    }

    return kemar_sendH;
}




/* --- Activity SetVelocity --------------------------------------------- */

/** Codel svStart of activity SetVelocity.
 *
 * Triggered by kemar_start.
 * Yields to kemar_ether.
 */
genom_event
svStart(double velocity, kemar_ids *ids, genom_context self)
{
    ids->headSpeed = velocity;
    kemarSetGearVelRadS(h, k, (ids->headSpeed*(pi/180)), MOTIONTYPE_POSCTRL);
    /*kemarGetInfo(h, k);
    printf("[DEBUG]: statusCtrl: 0x%08x\n", h->statusCtrl);
    if(h->motionType == MOTIONTYPE_POSCTRL)
        printf("[DEBUG] MOTIONTYPE_POSCTRL\n");
    else
        printf("[DEBUG] MOTIONTYPE_VECCTRL\n");
    printf("[DEBUG] target velocity:  %6d (%3.8f = %2.2f)\n", k->velTargetEncIncrPeriod, k->velTargetGearRadS, k->velTargetGearRadS*(180/pi));*/
    return kemar_ether;
}


/* --- Activity MoveAbsolutePosition ------------------------------------ */

/** Codel mapStart of activity MoveAbsolutePosition.
 *
 * Triggered by kemar_start.
 * Yields to kemar_sendMAP.
 */
genom_event
mapStart(kemar_ids *ids, genom_context self)
{
    flagMAP=0;
    stepMAP=0;
    waitMAP=0;

/**********************************************
    //Set the speed according to the value stored in the ids.
    printf("[DEBUG] ids->headSpeed: %2.2f\n", ids->headSpeed);
    kemarSetGearVelRadS(h, k, (ids->headSpeed*(pi/180)), MOTIONTYPE_POSCTRL);

    kemarGetInfo(h, k);
    printf("[DEBUG]: statusCtrl: 0x%08x\n", h->statusCtrl);
    if(h->motionType == MOTIONTYPE_POSCTRL)
        printf("[DEBUG] MOTIONTYPE_POSCTRL\n");
    else
        printf("[DEBUG] MOTIONTYPE_VECCTRL\n");
    printf("[DEBUG] target velocity:  %6d (%3.8f = %2.2f)\n", k->velTargetEncIncrPeriod, k->velTargetGearRadS, k->velTargetGearRadS*(180/pi));
**********************************************/
    return kemar_sendMAP;
}

/** Codel mapSend of activity MoveAbsolutePosition.
 *
 * Triggered by kemar_sendMAP.
 * Yields to kemar_recvMAP, kemar_ether.
 */
genom_event
mapSend(kemar_ids *ids, double target, const kemar_Indexes *Indexes,
        genom_context self)
{
    if(h->homePos == true)
    {
        if(flagMAP==0)
        {
            /*Get the current position to post in the port*/
            printf("[DEBUG 1] Ask for the current position\n");
            return kemar_recvMAP;
        }
        else
        {
            printf("[DEBUG] stepMAP: %d\n", stepMAP);
            if(stepMAP==1)
            {
                /*Post the current (start) position in the port*/
				gettimeofday(&tv, NULL);
				Indexes->data(self)->startTimeStamp.sec = tv.tv_sec;
				Indexes->data(self)->startTimeStamp.usec = tv.tv_usec;
                printf("[DEBUG 2] Current position: %3.8f\n", k->posGearRad[0]*(180/pi));
                Indexes->data(self)->startPosition = k->posGearRad[0]*(180/pi);

                /*Move the head*/
                //printf("[DEBUG] Call kemarSetGearVelRadS\n");
                //kemarSetGearVelRadS(h, k, (velocity*(pi/180)), MOTIONTYPE_POSCTRL);
                //printf("[DEBUG] Returns kemarSetGearVelRadS\n");
                printf("[DEBUG 3] Call kemarSetGearPosAbsRad\n");
                printf("[DEBUG 3a] ids->headSpeed: %2.2f\n", ids->headSpeed);
                kemarSetGearVelRadS(h, k, (ids->headSpeed*(pi/180)), MOTIONTYPE_POSCTRL);
                kemarSetGearPosAbsRad(h, k, (target*(pi/180)));
                printf("[DEBUG 4] Return kemarSetGearPosAbsRad\n");
                Indexes->data(self)->stopPosition = target;
                printf("[DEBUG 5] Saves Indexes->data(self)->stopPosition = target\n");
                globalTarget = target;
                stepMAP++;
                printf("[DEBUG 6] Increments stepMAP: %d\n", stepMAP);
                return kemar_recvMAP;
            }
            else
            {
                flagMAP=0;
                stepMAP = 0;
                /*Post the current (stop) position in the port*/
				gettimeofday(&tv, NULL);
				Indexes->data(self)->stopTimeStamp.sec = tv.tv_sec;
				Indexes->data(self)->stopTimeStamp.usec = tv.tv_usec;
                Indexes->write(self);
                printf("[DEBUG 7]: Saves Stop Index and returns ether\n");
                return kemar_ether;
            }
        }
    }
    else
    {
        printf("Homing function not done. Please do the Homing Function\n");
        return kemar_ether;
    }
}

/** Codel mapWaitForData of activity MoveAbsolutePosition.
 *
 * Triggered by kemar_recvMAP.
 * Yields to kemar_sendMAP, kemar_recvMAP.
 */
genom_event
mapWaitForData(genom_context self)
{
    if(stepMAP==0)
    {
        kemarGetInfo(h, k);
        stepMAP++;
        flagMAP=1;
        return kemar_sendMAP;
    }
    else
    {
        //printf("[DEBUG] kemarWaitMsgValid\n");
        //kemarWaitMsgValid(h, MSG_PX, (globalTarget*(pi/180)));
        //kemarGetInfo(h, k);
        kemarGetInfo(h, k);
        if(k->posGearRad[0]*(180/pi)>=0)
        {
            if((k->posGearRad[0]*(180/pi))<globalTarget-0.5)
            {
                return kemar_recvMAP;
            }
            if((k->posGearRad[0]*(180/pi))>globalTarget+0.5)
            {
                return kemar_recvMAP;
            }
        }
        else
        {
            if((k->posGearRad[0]*(180/pi))>globalTarget+0.5)
            {
                return kemar_recvMAP;
            }
            if((k->posGearRad[0]*(180/pi))<globalTarget-0.5)
            {
                return kemar_recvMAP;
            }
        }
    }
    return kemar_sendMAP;
}


/* --- Activity MoveRelativePosition ------------------------------------ */

/** Codel mrpStart of activity MoveRelativePosition.
 *
 * Triggered by kemar_start.
 * Yields to kemar_sendMRP.
 */
genom_event
mrpStart(kemar_ids *ids, genom_context self)
{
    flagMRP=0;
    //kemarSetGearVelRadS(h, k, (ids->headSpeed*(pi/180)), MOTIONTYPE_POSCTRL);
    return kemar_sendMRP;
}

/** Codel mrpSend of activity MoveRelativePosition.
 *
 * Triggered by kemar_sendMRP.
 * Yields to kemar_recvMRP, kemar_ether.
 */
genom_event
mrpSend(double target, genom_context self)
{
    if(h->homePos == true)
    {
        if(flagMRP==0)
        {
            //kemarSetGearVelRadS(h, k, (velocity*(pi/180)), MOTIONTYPE_POSCTRL);
            globalTarget = (k->posGearRad[0]*(180/pi)) + target;
            //printf("[DEBUG] globalTarget: %2.3f\n", globalTarget);
	        kemarSetGearPosRelRad(h, k, (target*(pi/180)));
            return kemar_recvMRP;
        }
        else
        {
            flagMRP=0;
            return kemar_ether;
        }
    }
    else
    {
        printf("Homing function not done. Please do the Homing Function\n");
        return kemar_ether;
    }
}

/** Codel mrpWaitForData of activity MoveRelativePosition.
 *
 * Triggered by kemar_recvMRP.
 * Yields to kemar_sendMRP, kemar_recvMRP.
 */
genom_event
mrpWaitForData(genom_context self)
{
    //kemarWaitMsgValid(h, MSG_PX_TARGET, (globalTarget*(pi/180)));
    kemarGetInfo(h, k);
    if(k->posGearRad[0]*(180/pi)>=0)
    {
        if((k->posGearRad[0]*(180/pi))<globalTarget-0.5)
        {
            return kemar_recvMRP;
        }
        if((k->posGearRad[0]*(180/pi))>globalTarget+0.5)
        {
            return kemar_recvMRP;
        }
    }
    else
    {
        if((k->posGearRad[0]*(180/pi))>globalTarget+0.5)
        {
            return kemar_recvMRP;
        }
        if((k->posGearRad[0]*(180/pi))<globalTarget-0.5)
        {
            return kemar_recvMRP;
        }
    }
    flagMRP=1;
    return kemar_sendMRP;
}


/* --- Activity ControlInSpeed ------------------------------------------ */

/** Codel cisStart of activity ControlInSpeed.
 *
 * Triggered by kemar_start.
 * Yields to kemar_sendCIS.
 */
genom_event
cisStart(genom_context self)
{
    flagCIS=0;
    return kemar_sendCIS;
}

/** Codel cisSend of activity ControlInSpeed.
 *
 * Triggered by kemar_sendCIS.
 * Yields to kemar_recvCIS, kemar_ether.
 */
genom_event
cisSend(double velocity, genom_context self)
{
    if(h->homePos == true)
    {
        kemarSetGearVelRadS(h, k, (velocity*(pi/360)), MOTIONTYPE_VELCTRL);
        //It is called twice because the first time that this activity is called by
        //the user the head does not move.
        kemarSetGearVelRadS(h, k, (velocity*(pi/360)), MOTIONTYPE_VELCTRL);
    }
    else
    {
        printf("Homing function not done. Please do the Homing Function\n");
    }
    return kemar_ether;
}

/** Codel cisWaitForData of activity ControlInSpeed.
 *
 * Triggered by kemar_recvCIS.
 * Yields to kemar_sendCIS, kemar_recvCIS.
 */
genom_event
cisWaitForData(genom_context self)
{
  /* skeleton sample: insert your code */
  /* skeleton sample */ return kemar_sendCIS;
}


/* --- Activity TEST ---------------------------------------------------- */

/** Codel testStart of activity TEST.
 *
 * Triggered by kemar_start.
 * Yields to kemar_ether.
 */
int stepTest=0;
genom_event
testStart(genom_context self)
{
    switch(stepTest)
    {
        case 0:
            //Move to 45 (Absolute position)
            printf("Moving in Absolute Position to 45 deg\n");
            kemarSetGearPosAbsRad(h, k, (45*(pi/180)));
            break;

        case 1:
            //Set velocity to 10 deg/sec
            printf("Setting speed to 10 deg/sec\n");
            kemarSetGearVelRadS(h, k, (10*(pi/180)), MOTIONTYPE_POSCTRL);
            break;

        case 2:
            //Move to 0 (Absolute position)
            printf("Moving in Absolute Position to 0 deg\n");
            kemarSetGearPosAbsRad(h, k, (0*(pi/180)));
            break;

        case 3:
            //Move at 50 deg/sec (Control in Speed)
            printf("Moving at 50 deg/sec (control in speed)\n");
            kemarSetGearVelRadS(h, k, (50*(pi/360)), MOTIONTYPE_VELCTRL);
            //If this is called ONCE the head does not move.
            kemarSetGearVelRadS(h, k, (50*(pi/360)), MOTIONTYPE_VELCTRL);
            break;

        case 4:
            //Stops the head movement.
            printf("Stopping the head (control in speed)\n");
            kemarSetGearVelRadS(h, k, (0*(pi/360)), MOTIONTYPE_VELCTRL);
            break;

        case 5:
            //Move to 0 (Absolute position)
            printf("Moving in Absolute Position to 0 deg\n");
            kemarSetGearVelRadS(h, k, (10*(pi/180)), MOTIONTYPE_POSCTRL);
            kemarSetGearPosAbsRad(h, k, (0*(pi/180)));
            break;

        case 6:
            //Move to 45 (Absolute position)
            printf("Moving in Absolute Position to 45 deg\n");
            kemarSetGearVelRadS(h, k, (10*(pi/180)), MOTIONTYPE_POSCTRL);
            kemarSetGearPosAbsRad(h, k, (45*(pi/180)));
            break;
    }
    stepTest++;
    if(stepTest>6)
        stepTest=0;
    return kemar_ether;
}
