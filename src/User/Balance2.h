#ifndef __BALANCE2_
#define __BALANCE2_

typedef struct
{
        float kp1;
        float ki1;
        float kd1;
        float jifen1;
        float weifen1;
        float out1;

        float kp2;
        float ki2;
        float kd2;
        float jifen2;
        float weifen2;
        float out2;

        float kps;
        float kis;
        float kds;
        float jifens;
        float weifens;
        float outs;

        float kpjd;
        float kijd;
        float kdjd;
        float jifenjd;
        float weifenjd;
        float outjd;
}FHL_PID_DJ;

void FHL_servo(void);

#endif
