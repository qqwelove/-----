#ifndef __PID_H
#define __PID_H

#include <zf_common_headfile.h>
#include "math.h"

typedef struct 
{
    float target;
    float actual;
    float actual1;
    float out;

    float kp;
    float ki;
    float kd;
    
    float error0;
    float error1;
    float errorint;

    float outmax;
    float outmin;

    float OutOffSet;
}PID_T;

void PID_Update(PID_T *p);
void PID_Init(PID_T *p);



#endif