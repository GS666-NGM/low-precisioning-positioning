#ifndef __ANGLE_H
#define __ANGLE_H

#define sampletime         0.01f
#define halfsampletime     0.005f

#include "main.h"
#include "MPU9520.h"
#include <math.h>

typedef struct imudata
{
    float ax, ay, az, gx, gy, gz;
}imudata;

extern imudata imu;
extern float roll, pitch, yaw;
extern uint8_t startflag;
//extern float yaw_m;
extern float yaw_k;

void Get_Angle(void);


#endif
