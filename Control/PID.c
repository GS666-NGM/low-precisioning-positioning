#include "PID.h"

#define pwm_max  5000
#define pwm_min  -5000

//max = 50多
float velocity_out;
int target_velocity;
float kp_velocity = -154;
float ki_velocity = -15.0f;

void Velocity(void)
{
    int actual;
    int16_t bias = 0;
    static int16_t bias_integral = 0;
    
    actual = EncoderL+EncoderR;
    bias = actual - target_velocity;
    bias_integral += bias;
    
    if(bias_integral > 400){bias_integral = 400;}
    else if(bias_integral < -400){bias_integral = -400;}

    velocity_out = bias*kp_velocity + bias_integral*ki_velocity;
}

float turn_out;
float target_turn;
float kp_turn = -120;
float ki_trun = -0.2;
void Turn(void)
{
    float bias;
    static float bias_integral = 0;
        
    bias = yaw - target_turn;
    bias>180 ? bias-=360 : 0;
    bias<-180 ? bias+=360 : 0;
    bias_integral += bias;
    
    //限幅
    if(bias_integral>5000)
        bias_integral = 5000;
    else if(bias_integral < -5000)
        bias_integral = -5000;
    //积分分离
    if(bias >= 10 || bias <= -10)
        bias_integral = 0;
    
    turn_out = bias*kp_turn + bias_integral*ki_trun;
    
    //死区输入
    if(0<bias && bias<5)
    {
        turn_out-=570;
    } 
    else if(bias>-5 && bias<0)
    {
        turn_out+=570;
    }
    if(bias<0.4 && bias > -0.4)
        turn_out = 0;
    
    if(distanceflag == 1)
        if(bias<=0.4 && bias >= -0.4)
            distanceflag = 2;
}

float distance_out;
float target_x, target_y;
float kp_distance = 60;
float ki_distance = 0.2;
float biaslast = 1000;
uint8_t distanceflag;
void Distance(void)
{
    float bias_x, bias_y, bias;
    static float bias_integral = 0;
    
    if(distanceflag == 2)
    {
        bias_x = target_x - px;
        bias_y = target_y - py;
        bias = sqrt(bias_x*bias_x + bias_y*bias_y);
        
        bias_integral += bias;
        
        if(bias_integral>10000)
            bias_integral = 10000;
        else if(bias_integral < -10000)
            bias_integral = -10000;
        
        distance_out = bias*kp_distance + bias_integral*ki_distance;
        
        if(bias <= 3 || biaslast-bias<-1 )
        {
            distanceflag = 0;
            bias_integral = 0;
            distance_out = 0;
            biaslast = 1000;
            return;
        }
        biaslast = bias;
    }
}

void PWM_Get(void)
{
    int32_t  pwml;
    int32_t  pwmr;
    
    pwml = velocity_out + turn_out + distance_out;
    pwmr = velocity_out - turn_out + distance_out;
    
    //限幅
    if(pwml>pwm_max){pwml = pwm_max;}
    else if(pwml<pwm_min){pwml = pwm_min;}
    if(pwmr>pwm_max){pwmr = pwm_max;}
    else if(pwmr<pwm_min){pwmr = pwm_min;}
    
    Motor_SetSpeed(pwml, pwmr);
}

