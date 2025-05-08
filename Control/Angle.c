#include "Angle.h"

double fliterKp = 30;
double fliterKi = 0;

int16_t gx, gy, gz;
int16_t ax, ay, az;
float normalax, normalay, normalaz;
float mx, my, mz;
float normalmx, normalmy, normalmz;
float vx, vy, vz;
float hx, hy, hz;
float bx, bz;
float wx, wy, wz;

float q[4] = {1,0,0,0};

imudata imu;

float biasx, biasy, biasz;
float biasx_intigral, biasy_intigral, biasz_intigral;
float roll, pitch, yaw;
float yaw_m;
float yaw_k;

int32_t tick1, tick2;
uint8_t startflag;

//取平方根倒数函数
float invSqrt(float x) 
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f375a86 - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

//单位转化
void imu_GetData(void)
{
    MPU9520_GetCaliberateData(&ax, &ay, &az, &gx, &gy, &gz);
    imu.ax = (float)ax / 16384;
    imu.ay = (float)ay / 16384;
    imu.az = (float)az / 16384;
    imu.gx = gx * 0.001065;
    imu.gy = gy * 0.001065;
    imu.gz = -gz * 0.001065 ;
}

void Data_Correction(void)
{
    float norm;
    float tq[4];

    //刚体加速度归一化（只保留角度信息）
    imu_GetData();
    norm  = invSqrt(imu.ax * imu.ax + imu.ay * imu.ay + imu.az * imu.az); 
    normalax = imu.ax * norm;
    normalay = imu.ay * norm;
    normalaz = imu.az * norm;
   
    //把地理系理论加速度转刚体系（用角速度表示）
    vx = 2*( q[1]*q[3] - q[0]*q[2] );
    vy = 2*( q[2]*q[3] + q[0]*q[1] );
    vz = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];
    
    //刚体磁强归一化
    MPU9520_GetCaliberateMag(&mx, &my, &mz);
    norm = invSqrt(mx * mx + my * my + mz * mz);
    normalmx = mx*norm;
    normalmy = my*norm;
    normalmz = mz*norm;
    
    //刚体磁强转地理系
    hx = normalmx*(1 - 2*q[2]*q[2] - 2*q[3]*q[3]) 
         + normalmy*2*(q[1]*q[2] - q[0]*q[3]) 
         + normalmz*2*(q[1]*q[3] + q[0]*q[2]);
    hy = normalmx*2*(q[1]*q[2] + q[0]*q[3]) 
         + normalmy*(1 - 2*q[1]*q[1] - 2*q[3]*q[3])
         + normalmz*2*(q[2]*q[3] - q[0]*q[1]);
    hz = normalmx*2*(q[1]*q[3] - q[0]*q[2])
         + normalmy*2*(q[2]*q[3] + q[0]*q[1])
         + normalmz*(1 - 2*q[1]*q[1] - 2*q[2]*q[2]);
    
    //地理系理论磁强
    bx = sqrt(hx*hx + hy*hy);
    bz = hz;
    
    //地理系理论磁强转刚体系
    wx = bx*(1 - 2*q[2]*q[2] - 2*q[3]*q[3])
         + bz* 2*(q[1]*q[3] - q[0]*q[2]);
    wy = bx*2*(q[1]*q[2] - q[0]*q[3])
         + bz*2*(q[2]*q[3] + q[0]*q[1]);
    wz = bx*2*(q[1]*q[3] + q[0]*q[2])
         + bz*(1 - 2*q[1]*q[1] - 2*q[2]*q[2]);
         
    //叉积，获取欧拉角误差
    biasx = (normalay*vz - normalaz*vy);
    biasy = (normalaz*vx - normalax*vz);
    biasz = (normalax*vy - normalay*vx) + (normalmx*wy - normalmy*wx);
//        biasx = (normalay*vz - normalaz*vy) + (normalmy*wz - normalmz*wy);
//    biasy = (normalaz*vx - normalax*vz) + (normalmz*wx - normalmx*wz);

    //二阶滤波器互补滤波
    biasx_intigral += fliterKi*biasx;
    biasy_intigral += fliterKi*biasy;
    biasz_intigral += fliterKi*biasz;
    imu.gx += fliterKp*biasx + biasx_intigral;
    imu.gy += fliterKp*biasy + biasy_intigral;
    imu.gz += fliterKp*biasz + biasz_intigral;
    
    //四元数更新
    tq[0] = q[0] + (-imu.gx*q[1] - imu.gy*q[2] - imu.gz*q[3])*halfsampletime;
    tq[1] = q[1] + (imu.gx*q[0] - imu.gy*q[3] + imu.gz*q[2])*halfsampletime;
    tq[2] = q[2] + (imu.gx*q[3] + imu.gy*q[0] - imu.gz*q[1])*halfsampletime;
    tq[3] = q[3] + (-imu.gx*q[2] + imu.gy*q[1] + imu.gz*q[0])*halfsampletime;
    
    //四元数重新归一化
    norm = invSqrt(tq[0]*tq[0] + tq[1]*tq[1] + tq[2]*tq[2] + tq[3]*tq[3]);
    q[0] = tq[0]*norm;
    q[1] = tq[1]*norm;
    q[2] = tq[2]*norm;
    q[3] = tq[3]*norm;
}

void Get_Angle(void)
{
    //快速收敛
    if(startflag == 0)
    {
        tick1 = HAL_GetTick();
        startflag = 1;
    }
    else if(startflag == 1)
    {
        tick2 = HAL_GetTick();
    }
    else if(startflag == 2)
    {
        fliterKp = 2.0;
        fliterKi = 0.00002;
        startflag = 3;
    }   
    if((tick2 - tick1) >= 1000 && startflag == 1)
    {
        startflag = 2;
    }
    
    imu_GetData();
    Data_Correction();
    if(startflag == 3)
    {
        roll = atan2f(2*(q[2]*q[3] + q[0]*q[1]), 1 - 2*(q[2]*q[2] + q[1]*q[1]))* 57.296;
        pitch = asinf(2*(q[0]*q[2] - q[1]*q[3])) * 57.296;
//        yaw = atan2f(2*(q[1]*q[2] + q[0]*q[3]), 1 - 2*(q[2]*q[2] + q[3]*q[3]))* 57.296;
        yaw += imu.gz*57.296*sampletime;
        yaw>180 ? yaw-=360 : 0;
        yaw<-180 ? yaw+=360 : 0;
//        yaw_m = atan2f(-my, mx)*57.296;
        yaw_k += imu.gz*sampletime;
    }
}
