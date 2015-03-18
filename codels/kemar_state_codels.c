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
/* --- Task state ------------------------------------------------------- */


/** Codel stateStart of task state.
 *
 * Triggered by kemar_start.
 * Yields to kemar_sendS.
 */
genom_event
stateStart(genom_context self)
{
    flagC=0;

    return kemar_sendS;
}


/** Codel sSend of task state.
 *
 * Triggered by kemar_sendS.
 * Yields to kemar_recvS, kemar_sendS.
 */
genom_event
sSend(const kemar_currentState *currentState, kemar_ids *ids,
      genom_context self)
{
    if(h->homePos == true)
    {
        kemarGetInfo(h, k);
        //printf("[DEBUG State] velocity: %2.2f\n", k->velGearRadS);
        if(k->velGearRadS > 0)
        {
            //printf("[DEBUG State] position: %2.2f\n", k->posGearRad[0]*(180/pi));
            //printf("[DEBUG State] left max: %2.2f\n", h->driveParam.leftRadMax*(180/pi));
            if(k->posGearRad[0]*(180/pi) > (h->driveParam.leftRadMax*(180/pi)-3))
            {
                //printf("[DEBUG State] motoro stopped left\n");
                kemarSetGearVelRadS(h, k, 0, MOTIONTYPE_VELCTRL);
                //kemarSetGearVelRadS(h, k, (ids->headSpeed*(pi/180)), MOTIONTYPE_POSCTRL);
            }
        }
        else
        {
            if(k->velGearRadS < 0)
            {
                if(k->posGearRad[0]*(180/pi) < (h->driveParam.rightRadMax*(180/pi)+3))
                {
                    //printf("[DEBUG State] motoro stopped right\n");
                    kemarSetGearVelRadS(h, k, 0, MOTIONTYPE_VELCTRL);
                    //kemarSetGearVelRadS(h, k, (ids->headSpeed*(pi/180)), MOTIONTYPE_POSCTRL);
                }
            }
        }
        if(flagC==0)
            return kemar_sendS;
        else
        {
            flagC = 0;
            currentState->data(self)->position = k->posGearRad[0]*(180/pi);
            currentState->data(self)->speed = k->velGearRadS*(180/pi);
            currentState->data(self)->maxLeft = h->driveParam.leftRadMax*(180/pi);
            currentState->data(self)->maxRight = h->driveParam.rightRadMax*(180/pi);
			gettimeofday(&tv, NULL);
			currentState->data(self)->time.sec = tv.tv_sec;
			currentState->data(self)->time.usec = tv.tv_usec;
            currentState->write(self); 
            return kemar_sendS;
        }      
    }
    else
    {
        currentState->data(self)->position = 0;
        currentState->data(self)->speed = 0;
        currentState->data(self)->maxLeft = 0;
        currentState->data(self)->maxRight = 0;
		currentState->data(self)->time.sec = 0;
		currentState->data(self)->time.usec = 0;
        currentState->write(self); 
        return kemar_sendS;
    }
}


/** Codel sWaitForData of task state.
 *
 * Triggered by kemar_recvS.
 * Yields to kemar_sendS, kemar_recvS.
 */
genom_event
sWaitForData(kemar_ids *ids, genom_context self)
{
    //kemarGetInfo(h, k);
    flagC=1;

    return kemar_sendS;
}
