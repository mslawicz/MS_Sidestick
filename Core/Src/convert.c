#include "convert.h"

float scale(float imin, float imax, float ivalue, float omin, float omax)
{
    float ovalue = (ivalue - imin) * (omax - omin) / (imax - imin) + omin;
    if(ovalue < omin)
    {
        ovalue = omin;
    }
    else if(ovalue > omax)
    {
        ovalue = omax;
    }
    return ovalue;
}