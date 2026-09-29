#include "pid.h"



void PID_Init(PID_T *p)
{
    p->target = 0;
    p->actual = 0;
     p->actual1 = 0;
    p->error0 = 0;
    p->error1 = 0;
    p->errorint = 0;
}

void PID_Update(PID_T *p)
{
    p ->error1 = p ->error0;
    p ->error0 = p ->target - p ->actual;

    /*抗积分饱和*/
   if(p ->ki * p ->errorint > p->outmax)
   {
       p->errorint = p->outmax / p ->ki;
   }
   else if(p ->ki * p ->errorint < p->outmin)
   {
       p->errorint = p->outmin / p ->ki;

   }

    if(p->ki != 0)
    {
        p ->errorint += p ->error0;
    }

    p ->out = p ->kp * p ->error0
            + p ->ki * p ->errorint
            // + p ->kd * (p ->error0 - p ->error1);
            + p ->kd * -(p ->actual - p ->actual1);

    if(p->out > 0){p->out += p->OutOffSet;}
    if(p->out < 0){p->out -= p->OutOffSet;}

    if(p ->out > p ->outmax)
    {
        p ->out = p ->outmax;
    }
    if(p ->out < p ->outmin)
    {
        p ->out = p ->outmin;
    }

    p->actual1 = p->actual;//改
}


