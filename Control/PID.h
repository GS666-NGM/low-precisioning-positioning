#ifndef __PID_H
#define __PID_H

#include "Angle.h"
#include "Encoder.h"
#include "Motor.h"
#include "math.h"
#include "Position.h"

void PWM_Get(void);
void Turn(void);
void Velocity(void);
void Distance(void);

extern float target_turn;
extern int target_velocity;
extern float target_x, target_y;
extern uint8_t distanceflag;
extern float ki_distance;
extern float distance_out;
extern float biaslast;

#endif

