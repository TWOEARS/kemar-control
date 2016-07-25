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
double previousVel = 0;
/* --- Task state ------------------------------------------------------- */


/** Codel stateStart of task state.
 *
 * Triggered by kemar_start.
 * Yields to kemar_sendS.
 */
genom_event
stateStart(genom_context self)
{
  /* skeleton sample: insert your code */
  /* skeleton sample */ return kemar_sendS;
}


/** Codel sSend of task state.
 *
 * Triggered by kemar_sendS.
 * Yields to kemar_pause_sendS.
 */
genom_event
sSend(const kemar_Cmd *Cmd, const kemar_currentState *currentState,
      kemar_ids *ids, genom_context self)
{
    uint32_t sec, nsec;

    if(h->homePos == true) {
        Cmd->read(self);
        if (Cmd->data(self) != NULL) {
            if (Cmd->data(self)->speed != previousVel) {
                kemarSetGearVelRadS(h, k, Cmd->data(self)->speed, MOTIONTYPE_VELCTRL);
                previousVel = Cmd->data(self)->speed;
            }
        }

        gettimeofday(&tv, NULL);
        sec = tv.tv_sec;
        nsec = tv.tv_usec*1000;
        kemarGetInfo(h, k);
        currentState->data(self)->position = k->posGearRad[0]*(180/pi);
        currentState->data(self)->speed = k->velGearRadS*(180/pi);
        currentState->data(self)->maxLeft = h->driveParam.leftRadMax*(180/pi);
        currentState->data(self)->maxRight = h->driveParam.rightRadMax*(180/pi);
        gettimeofday(&tv, NULL);
        currentState->data(self)->time.sec = (sec + tv.tv_sec)/2;
        currentState->data(self)->time.nsec = (nsec + tv.tv_usec*1000)/2;
        currentState->write(self);
    }

    return kemar_pause_sendS;
}
