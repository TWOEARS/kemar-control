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
double previousVel = 0, lastSetVel;
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
sSend(const kemar_Cmd *Cmd, kemar_ids *ids, genom_context self)
{
    double currentPos, target, velocity;
    if(h->homePos == true)
    {
        Cmd->read(self);
        if (Cmd->data(self) != NULL)
        {
            if(previousVel != Cmd->data(self)->speed)
            {printf("New\n");
                velocity = Cmd->data(self)->speed;
                previousVel = velocity;
                // If user requests the head to stop, avoid a sudden stop.
                if(velocity == 0)
                {
                    // Get current position.
                    kemarGetInfo(h, k);
                    currentPos = k->posGearRad[0]*(180/pi);
                    // Change to control in position (to have the acceleration and deaceleration curve). Velocity is set to the current to avoid abrupt changes.
                    printf("Current velocity: %.2f\n", lastSetVel);
                    kemarSetGearVelRadS(h, k, (lastSetVel*(pi/180)), MOTIONTYPE_POSCTRL);
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
                {
                    kemarSetGearVelRadS(h, k, (velocity*(pi/180)), MOTIONTYPE_VELCTRL);
                    lastSetVel = velocity;
                }
            }
        }
    }

    return kemar_pause_sendS;
}
