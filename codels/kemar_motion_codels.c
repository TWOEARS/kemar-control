/*
 * Copyright (c) 2014, LAAS/CNRS
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
motionStart(genom_context self)
{
    /* init kemar head */
    h = kemarInit(MOTIONTYPE_POSCTRL);
    p = &h->driveParam;

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

    return kemar_sendH;
}

/** Codel hSend of activity Homing.
 *
 * Triggered by kemar_sendH.
 * Yields to kemar_recvH, kemar_ether.
 */
genom_event
hSend(genom_context self)
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
            flagH=1;
            return kemar_sendH;
    }

    return kemar_sendH;
}


/* --- Activity CurrentPosition ----------------------------------------- */

/** Codel cpStart of activity CurrentPosition.
 *
 * Triggered by kemar_start.
 * Yields to kemar_sendCP.
 */
genom_event
cpStart(genom_context self)
{
    flagCP=0;
    showCP=0;
    return kemar_sendCP;
}

/** Codel cpSend of activity CurrentPosition.
 *
 * Triggered by kemar_sendCP.
 * Yields to kemar_recvCP, kemar_sendCP, kemar_ether.
 */
genom_event
cpSend(const kemar_currentState *currentState, genom_context self)
{
    if(h->homePos == true)
    {
        while(showCP==0)
        {
            if(flagCP==0)
                return kemar_recvCP;
            else
            {
                flagCP=0;
	            printf("------------------------------------------------------\n");
                h->motionType ? printf("motionType: Control in Position\n") : printf("motionType: Control in Speed\n");
                
                printf("Max Left : %3.8f - Max Right : %3.8f\n", h->driveParam.leftRadMax*(360/pi), h->driveParam.rightRadMax*(360/pi));

	            if(h->motionType == MOTIONTYPE_POSCTRL)
                {
		            printf("Target Position:  %3.8f\n", k->posTargetGearRad*(360/pi));
		            printf("Current Position: %3.8f\n", k->posGearRad[0]*(360/pi));
                    printf("Current Velocity: %3.8f\n", k->velGearRadS*(360/pi));
	            } 
                else
                {
		            printf("Current Position: %3.8f\n", k->posGearRad[0]*(360/pi));
                    printf("Current Velocity: %3.8f\n", k->velGearRadS*(360/pi));
	            }
            	printf("------------------------------------------------------\n");
                showCP=1;   /*Only shows current State ONCE*/              
                return kemar_sendCP;
            }
        }
        return kemar_ether;
    }
    else
    {
        printf("Homing function not done. Please do the Homing Function\n");
        return kemar_ether;
    }
}

/** Codel cpWaitForData of activity CurrentPosition.
 *
 * Triggered by kemar_recvCP.
 * Yields to kemar_sendCP, kemar_recvCP.
 */
genom_event
cpWaitForData(genom_context self)
{
    kemarGetInfo(h, k);
    flagCP=1;
    return kemar_sendCP;
}


/* --- Activity StopCurrentPosition ------------------------------------- */

/** Codel scpStart of activity StopCurrentPosition.
 *
 * Triggered by kemar_start.
 * Yields to kemar_ether.
 */
genom_event
scpStart(genom_context self)
{
    showCP=1;

    return kemar_ether;
}


/* --- Activity MoveAbsolutePosition ------------------------------------ */

/** Codel mapStart of activity MoveAbsolutePosition.
 *
 * Triggered by kemar_start.
 * Yields to kemar_sendMAP.
 */
genom_event
mapStart(genom_context self)
{
    flagMAP=0;

    return kemar_sendMAP;
}

/** Codel mapSend of activity MoveAbsolutePosition.
 *
 * Triggered by kemar_sendMAP.
 * Yields to kemar_recvMAP, kemar_ether.
 */
genom_event
mapSend(double target, double velocity, genom_context self)
{
    if(h->homePos == true)
    {
        if(flagMAP==0)
        {
            kemarSetGearVelRadS(h, k, (velocity*(pi/360)), MOTIONTYPE_POSCTRL);
	        kemarSetGearPosAbsRad(h, k, (target*(pi/360)));
            globalTarget = target;
            return kemar_recvMAP;
        }
        else
        {
            flagMAP=0;
            return kemar_ether;
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
    kemarWaitMsgValid(h, MSG_PX_TARGET, (globalTarget*(pi/360)));
    flagMAP=1;
    return kemar_sendMAP;
}


/* --- Activity MoveRealtivePosition ------------------------------------ */

/** Codel mrpStart of activity MoveRealtivePosition.
 *
 * Triggered by kemar_start.
 * Yields to kemar_sendMRP.
 */
genom_event
mrpStart(genom_context self)
{
    flagMRP=0;
    return kemar_sendMRP;
}

/** Codel mrpSend of activity MoveRealtivePosition.
 *
 * Triggered by kemar_sendMRP.
 * Yields to kemar_recvMRP, kemar_ether.
 */
genom_event
mrpSend(double target, double velocity, genom_context self)
{
    if(h->homePos == true)
    {
        if(flagMRP==0)
        {
            kemarSetGearVelRadS(h, k, (velocity*(pi/360)), MOTIONTYPE_POSCTRL);
	        kemarSetGearPosRelRad(h, k, (target*(pi/360)));
            globalTarget = target;
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

/** Codel mrpWaitForData of activity MoveRealtivePosition.
 *
 * Triggered by kemar_recvMRP.
 * Yields to kemar_sendMRP, kemar_recvMRP.
 */
genom_event
mrpWaitForData(genom_context self)
{
    kemarWaitMsgValid(h, MSG_PX_TARGET, (globalTarget*(pi/360)));
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
