/*  Copyright (c) 2015-2016, LAAS/CNRS
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
motionStart(kemar_ids *ids, genom_context self)
{
    /* init kemar head */
    h = kemarInit(MOTIONTYPE_POSCTRL);
    p = &h->driveParam;
    firstHoming = 0;

    ids->headSpeed=100;
    printf("Default speed set to %.2f\n", ids->headSpeed);

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
  /* skeleton sample: insert your code */
  /* skeleton sample */ return kemar_sendMAP;
}

/** Codel mapSend of activity MoveAbsolutePosition.
 *
 * Triggered by kemar_sendMAP.
 * Yields to kemar_ether.
 */
genom_event
mapSend(kemar_ids *ids, double target, genom_context self)
{
    if(h->homePos == true)
    {
        if((target < (h->driveParam.rightRadMax*(180/pi))) || (target > (h->driveParam.leftRadMax*(180/pi))))
        {
            // This is to avoid a sudden stop when it reaches the limit sensors.
            if(target>0)
                target = h->driveParam.leftRadMax*(180/pi)-0.5;
            else
                target = h->driveParam.rightRadMax*(180/pi)+0.5;
        }
        kemarGetInfo(h, k);
        if(h->motionType !=MOTIONTYPE_POSCTRL)
            kemarSetGearVelRadS(h, k, ((ids->headSpeed)*(pi/180)), MOTIONTYPE_POSCTRL);
        kemarSetGearPosAbsRad(h, k, (target*(pi/180)));
        kemarWaitMsgValid(h, MSG_PX_TARGET, (target*(pi/180)));
    }
    else
    {
        printf("Homing function not done. Please do the Homing Function\n");
    }

    return kemar_ether;
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
  /* skeleton sample: insert your code */
  /* skeleton sample */ return kemar_sendMRP;
}

/** Codel mrpSend of activity MoveRelativePosition.
 *
 * Triggered by kemar_sendMRP.
 * Yields to kemar_ether.
 */
genom_event
mrpSend(kemar_ids *ids, double target, genom_context self)
{
    if(h->homePos == true)
    {
        kemarGetInfo(h, k);
        kemarGetInfo(h, k); // Has to be called twice to retrieve the correct current position.
        globalTarget = k->posGearRad[0]*(180/pi) + target;
        if((globalTarget < (h->driveParam.rightRadMax*(180/pi))) || (globalTarget > (h->driveParam.leftRadMax*(180/pi))))
        {
            // This is to avoid a sudden stop when it reaches the limit sensors.
            if(globalTarget>0)
                target = (h->driveParam.leftRadMax*(180/pi)) - k->posGearRad[0]*(180/pi) -0.5;
            else
                target = (h->driveParam.rightRadMax*(180/pi)) + k->posGearRad[0]*(180/pi) +0.5;
        }
        if(h->motionType !=MOTIONTYPE_POSCTRL)
            kemarSetGearVelRadS(h, k, ((ids->headSpeed)*(pi/180)), MOTIONTYPE_POSCTRL);
        kemarSetGearPosRelRad(h, k, (target*(pi/180)));
        kemarWaitMsgValid(h, MSG_PX_TARGET, (target*(pi/180)));
        kemarGetInfo(h, k);
    }
    else
    {
        printf("Homing function not done. Please do the Homing Function\n");
    }

    return kemar_ether;
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
  /* skeleton sample: insert your code */
  /* skeleton sample */ return kemar_sendCIS;
}

/** Codel cisSend of activity ControlInSpeed.
 *
 * Triggered by kemar_sendCIS.
 * Yields to kemar_ether.
 */
genom_event
cisSend(kemar_ids *ids, double velocity, genom_context self)
{
    double currentPos, target;
    if(h->homePos == true)
    {
        // If user requests the head to stop, avoid a sudden stop.
        if(velocity == 0)
        {
            // Get current position.
            kemarGetInfo(h, k);
            currentPos = k->posGearRad[0]*(180/pi);
            // Change to control in position (to have the acceleration and deaceleration curve).
            kemarSetGearVelRadS(h, k, ((ids->headSpeed)*(pi/180)), MOTIONTYPE_POSCTRL);
            // Check if the head was moving to the left (>0) or to the right (<0)
            if(k->velGearRadS>0)
                target = currentPos + overShoot;
            else
                target = currentPos - overShoot;

            // Move the head to the "overShoot" position to stop it smoothly.
            if((target < (h->driveParam.rightRadMax*(180/pi))) || (target > (h->driveParam.leftRadMax*(180/pi))))
            {
                // This is to avoid a sudden stop when it reaches the limit sensors.
                if(target>0)
                    target = h->driveParam.leftRadMax*(180/pi)-0.5;
                else
                    target = h->driveParam.rightRadMax*(180/pi)+0.5;
            }
            kemarSetGearPosAbsRad(h, k, (target*(pi/180)));
            kemarWaitMsgValid(h, MSG_PX_TARGET, (target*(pi/180)));

            // Move the head to the position when the user sent the command to stop it.
            target = currentPos;
            if((target < (h->driveParam.rightRadMax*(180/pi))) || (target > (h->driveParam.leftRadMax*(180/pi))))
            {
                // This is to avoid a sudden stop when it reaches the limit sensors.
                if(target>0)
                    target = h->driveParam.leftRadMax*(180/pi)-0.5;
                else
                    target = h->driveParam.rightRadMax*(180/pi)+0.5;
            }
            kemarSetGearPosAbsRad(h, k, (target*(pi/180)));
            kemarWaitMsgValid(h, MSG_PX_TARGET, (target*(pi/180)));
        }
        else
            kemarSetGearVelRadS(h, k, (velocity*(pi/180)), MOTIONTYPE_VELCTRL);
    }
    else
    {
        printf("Homing function not done. Please do the Homing Function\n");
    }
    return kemar_ether;
}
