#include "ackemar.h"

#include "kemar_c_types.h"

#include "includes.h"
/* --- Function GetPosition --------------------------------------------- */

/** Codel getPosition of function GetPosition.
 *
 * Returns genom_ok.
 */
genom_event
getPosition(kemar_head *Head, genom_context self)
{
    uint32_t sec, nsec;
    if(h->homePos == true)
    {
        gettimeofday(&tv, NULL);
		sec = tv.tv_sec;
		nsec = tv.tv_usec*1000;
        kemarGetInfo(h, k);
        Head->position = k->posGearRad[0]*(180/pi);
        Head->speed = k->velGearRadS*(180/pi);
        Head->maxLeft = h->driveParam.leftRadMax*(180/pi);        
        Head->maxRight = h->driveParam.rightRadMax*(180/pi);
        gettimeofday(&tv, NULL);
		Head->time.sec = (sec + tv.tv_sec)/2;
		Head->time.nsec = (nsec + tv.tv_usec*1000)/2;
    }

    return genom_ok;
}
