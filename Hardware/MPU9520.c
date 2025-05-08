#include "main.h"
#include "MyI2C.h"
#include "MPU9520_Reg.h"
#include "Delay.h"
#include "arm_math.h"
#include "arm_const_structs.h"
#include "stdio.h"
#include "W25Q64.h"

int8_t ASAX, ASAY, ASAZ;
int16_t data_bias[6];
float mx_offset, my_offset, mz_offset;
float mx_k, my_k, mz_k;
float magbias[6];
uint8_t exitflag;
void MPU9520_Init(void)
{
    //复位
    I2C_WriteReg(MPU9520_ADDRESS, MPU9520_PWR_MGMT_1, 0x80);
    Delay_ms(100);
    //唤醒,选择时钟源为x轴陀螺仪
    I2C_WriteReg(MPU9520_ADDRESS, MPU9520_PWR_MGMT_1, 0x01);
    //所有轴均不待机
    I2C_WriteReg(MPU9520_ADDRESS, MPU9520_PWR_MGMT_2, 0x00);
    //采样率
    I2C_WriteReg(MPU9520_ADDRESS, MPU9520_SMPLRT_DIV, 0x09);
    //配置寄存器，配置低通滤波，满足采样率>=2*带宽
    I2C_WriteReg(MPU9520_ADDRESS, MPU9520_CONFIG, 0x03);
    //acc量程
    I2C_WriteReg(MPU9520_ADDRESS, MPU9520_ACCEL_CONFIG, 0x00);
    //gyro量程
    I2C_WriteReg(MPU9520_ADDRESS, MPU9520_GYRO_CONFIG, 0x18);
    //开启旁路
    I2C_WriteReg(MPU9520_ADDRESS, MPU9520_INT_PIN_CFG, 0x02);
    //磁力计连续测量
    I2C_WriteReg(AK8963_ADDRESS, AK8963_CNTL1, 0x16);
    
    ASAX = I2C_ReadReg(AK8963_ADDRESS, AK8963_ASAX);
    ASAY = I2C_ReadReg(AK8963_ADDRESS, AK8963_ASAY);
    ASAZ = I2C_ReadReg(AK8963_ADDRESS, AK8963_ASAZ);
}

void MPU9520_GetData(int16_t* accx, int16_t* accy, int16_t* accz,
                     int16_t* gyx, int16_t* gyy, int16_t* gyz)
{
    uint8_t DataH, DataL;							           
  
    DataH = I2C_ReadReg(MPU9520_ADDRESS, MPU9520_ACCEL_XOUT_H);
    DataL = I2C_ReadReg(MPU9520_ADDRESS, MPU9520_ACCEL_XOUT_L);
    *accx = (DataH << 8) | DataL;						       

    DataH = I2C_ReadReg(MPU9520_ADDRESS, MPU9520_ACCEL_YOUT_H);
    DataL = I2C_ReadReg(MPU9520_ADDRESS, MPU9520_ACCEL_YOUT_L);
    *accy = (DataH << 8) | DataL;						       

    DataH = I2C_ReadReg(MPU9520_ADDRESS, MPU9520_ACCEL_ZOUT_H);
    DataL = I2C_ReadReg(MPU9520_ADDRESS, MPU9520_ACCEL_ZOUT_L);
    *accz = (DataH << 8) | DataL;						       

    DataH = I2C_ReadReg(MPU9520_ADDRESS, MPU9520_GYRO_XOUT_H);
    DataL = I2C_ReadReg(MPU9520_ADDRESS, MPU9520_GYRO_XOUT_L);
    *gyx = (DataH << 8) | DataL;						       

    DataH = I2C_ReadReg(MPU9520_ADDRESS, MPU9520_GYRO_YOUT_H);
    DataL = I2C_ReadReg(MPU9520_ADDRESS, MPU9520_GYRO_YOUT_L);
    *gyy = (DataH << 8) | DataL;						       

    DataH = I2C_ReadReg(MPU9520_ADDRESS, MPU9520_GYRO_ZOUT_H);
    DataL = I2C_ReadReg(MPU9520_ADDRESS, MPU9520_GYRO_ZOUT_L);
    *gyz = (DataH << 8) | DataL;						        
}

void MPU9520_Caliberation(void)
{
    int16_t data[6] = {0};
    int32_t data_sum[6] = {0};
    int16_t data_average[6] = {0};
    
    //采样一百次
    for(uint8_t j = 0; j < 200; j++)
    {
        MPU9520_GetData(&data[0], &data[1], &data[2], &data[3], &data[4], &data[5]);
        for(uint8_t i = 0; i < 6; i++)
        {
            data_sum[i] += data[i];
        }
        Delay_ms(10);
    }
    //取平均值
    for(uint8_t k = 0; k < 6; k++)
        {
            data_average[k] = data_sum[k]/200;
        }
    //计算误差
    data_bias[0] = -data_average[0];
    data_bias[1] = -data_average[1];
    data_bias[2] = (16384-data_average[2]);
    data_bias[3] = -data_average[3];
    data_bias[4] = -data_average[4];
    data_bias[5] = -data_average[5];        
}

void MPU9520_GetCaliberateData(int16_t* accx, int16_t* accy, int16_t* accz,
                     int16_t* gyx, int16_t* gyy, int16_t* gyz)
{
    int16_t data[6] = {0};
    MPU9520_GetData(&data[0], &data[1], &data[2], &data[3], &data[4], &data[5]);
    *accx = data[0] + data_bias[0];
    *accy = data[1] + data_bias[1];
    *accz = data[2] + data_bias[2];
    *gyx = data[3] + data_bias[3];
    *gyy = data[4] + data_bias[4];
    *gyz = data[5] + data_bias[5];
}




void MPU9520_GetMag(float* magx, float* magy, float* magz)
{
    uint8_t DataH, DataL;
    int16_t x, y, z;
    
    DataH = I2C_ReadReg(AK8963_ADDRESS, AK8963_XOUT_H);
    DataL = I2C_ReadReg(AK8963_ADDRESS, AK8963_XOUT_L);
    x = (DataH << 8) | DataL;						       
    

    DataH = I2C_ReadReg(AK8963_ADDRESS, AK8963_YOUT_H);
    DataL = I2C_ReadReg(AK8963_ADDRESS, AK8963_YOUT_L);
    y = (DataH << 8) | DataL;						       
    

    DataH = I2C_ReadReg(AK8963_ADDRESS, AK8963_ZOUT_H);
    DataL = I2C_ReadReg(AK8963_ADDRESS, AK8963_ZOUT_L);
    z = (DataH << 8) | DataL;						   
    
    I2C_ReadReg(AK8963_ADDRESS, AK8963_ST2);
  
    *magx = x * ((float)(ASAX-128)/1280+1)*0.15;
    *magy = y * ((float)(ASAY-128)/1280+1)*0.15;
    *magz = z * ((float)(ASAZ-128)/1280+1)*0.15;
    //单位是μT
}




void MPU9520_GetCaliberateMag(float* magx, float* magy, float* magz)
{
    MPU9520_GetMag(magx, magy, magz);
    *magx = (*magx - magbias[0])/magbias[3];
    *magy = (*magy - magbias[1])/magbias[4];
    *magz = (*magz - magbias[2])/magbias[5];
}

float   x_sum,    xx_sum,    xxx_sum,    yyyy_sum,
        y_sum,    yy_sum,    xxy_sum,    zzzz_sum,
        z_sum,    zz_sum,    xxz_sum,    xxyy_sum,
                  xy_sum,    xyy_sum,    xxzz_sum,
                  xz_sum,    xzz_sum,    yyzz_sum,
                  yz_sum,    yyy_sum,    
                             yyz_sum,    
                             yzz_sum,    
                             zzz_sum;    

float   x_avr,    xx_avr,    xxx_avr,    yyyy_avr,
        y_avr,    yy_avr,    xxy_avr,    zzzz_avr,
        z_avr,    zz_avr,    xxz_avr,    xxyy_avr,
                  xy_avr,    xyy_avr,    xxzz_avr,
                  xz_avr,    xzz_avr,    yyzz_avr,
                  yz_avr,    yyy_avr,    
                             yyz_avr,    
                             yzz_avr,    
                             zzz_avr;    
uint32_t n;
void Mag_Caliberate(void)
{
    float mx,my,mz;
    
    float A[36], A_inv[36];
    arm_matrix_instance_f32 A_matrix, A_inv_matrix;
    float B[6];
    arm_matrix_instance_f32 B_matrix;
    float Par[6];
    arm_matrix_instance_f32 Par_matrix;
    
    MPU9520_GetMag(&mx, &my, &mz);
    n++;
    
    //求和
    x_sum+=(mx);    xx_sum+=(mx*mx);    xxx_sum+=(mx*mx*mx);    yyyy_sum+=(my*my*my*my);
    y_sum+=(my);    yy_sum+=(my*my);    xxy_sum+=(mx*mx*my);    zzzz_sum+=(mz*mz*mz*mz);
    z_sum+=(mz);    zz_sum+=(mz*mz);    xxz_sum+=(mx*mx*mz);    xxyy_sum+=(mx*mx*my*my);
                    xy_sum+=(mx*my);    xyy_sum+=(mx*my*my);    xxzz_sum+=(mx*mx*mz*mz);
                    xz_sum+=(mx*mz);    xzz_sum+=(mx*mz*mz);    yyzz_sum+=(my*my*mz*mz);
                    yz_sum+=(my*mz);    yyy_sum+=(my*my*my);    
                                    yyz_sum+=(my*my*mz);    
                                    yzz_sum+=(my*mz*mz);    
                                    zzz_sum+=(mz*mz*mz); 
    
    //取平均，防止溢出
    x_avr=x_sum/n,    xx_avr=xx_sum/n,    xxx_avr=xxx_sum/n,    yyyy_avr=yyyy_sum/n,
    y_avr=y_sum/n,    yy_avr=yy_sum/n,    xxy_avr=xxy_sum/n,    zzzz_avr=zzzz_sum/n,
    z_avr=z_sum/n,    zz_avr=zz_sum/n,    xxz_avr=xxz_sum/n,    xxyy_avr=xxyy_sum/n,
                      xy_avr=xy_sum/n,    xyy_avr=xyy_sum/n,    xxzz_avr=xxzz_sum/n,
                      xz_avr=xz_sum/n,    xzz_avr=xzz_sum/n,    yyzz_avr=yyzz_sum/n,
                      yz_avr=yz_sum/n,    yyy_avr=yyy_sum/n,    
                                          yyz_avr=yyz_sum/n,    
                                          yzz_avr=yzz_sum/n,    
                                          zzz_avr=zzz_sum/n;  
    
    //系数矩阵赋值
    A[0 ] = yyyy_avr;A[1 ]=yyzz_avr;A[2 ]=xyy_avr;A[3 ]=yyy_avr;A[4 ]=yyz_avr;A[5 ]=yy_avr;
    A[6 ] = yyzz_avr;A[7 ]=zzzz_avr;A[8 ]=xzz_avr;A[9 ]=yzz_avr;A[10]=zzz_avr;A[11]=zz_avr;
    A[12] = xyy_avr ;A[13]=xzz_avr ;A[14]=xx_avr ;A[15]=xy_avr ;A[16]=xz_avr ;A[17]=x_avr ;
    A[18] = yyy_avr ;A[19]=yzz_avr ;A[20]=xy_avr ;A[21]=yy_avr ;A[22]=yz_avr ;A[23]=y_avr ;
    A[24] = yyz_avr ;A[25]=zzz_avr ;A[26]=xz_avr ;A[27]=yz_avr ;A[28]=zz_avr ;A[29]=z_avr ;
    A[30] = yy_avr  ;A[31]=zz_avr  ;A[32]=x_avr  ;A[33]=y_avr  ;A[34]=z_avr  ;A[35]=1     ;
    
    //非齐次列向量赋值
    B[0] = -xxyy_avr;
    B[1] = -xxzz_avr;
    B[2] = -xxx_avr ;
    B[3] = -xxy_avr ;
    B[4] = -xxz_avr ;
    B[5] = -xx_avr  ;
    
    //矩阵初始化
    arm_mat_init_f32(&A_matrix, 6, 6, A);   // 参数：矩阵实例指针、行数、列数、数据指针
    arm_mat_init_f32(&A_inv_matrix, 6, 6, A_inv);
    arm_mat_init_f32(&B_matrix, 6, 1, B);
    arm_mat_init_f32(&Par_matrix, 6, 1, Par);
    
    //矩阵求逆
    arm_mat_inverse_f32(&A_matrix,&A_inv_matrix);
    
    //解方程
    arm_mat_mult_f32(&A_inv_matrix, &B_matrix, &Par_matrix);
    
    //中心坐标
    mx_offset = (-Par[2]/2.0f);
    my_offset = (-Par[3]/(2.0f*Par[0]));
    mz_offset = (-Par[4]/(2.0f*Par[1]));
    
    //半径
    mx_k = sqrtf(mx_offset*mx_offset + Par[0]*my_offset*my_offset + Par[1]*mz_offset*mz_offset - Par[5]);
    my_k = mx_k/sqrt(Par[0]);
    mz_k = mx_k/sqrt(Par[1]);
}

void Mag_Bias_Store(void)
{
    char data[6][20];
    sprintf(data[0], "%.7f", mx_offset);
    sprintf(data[1], "%.7f", my_offset);
    sprintf(data[2], "%.7f", mz_offset);
    sprintf(data[3], "%.7f", mx_k);
    sprintf(data[4], "%.7f", my_k);
    sprintf(data[5], "%.7f", mz_k);
    
    W25Q64_SectorErase(0x200000);
    W25Q64_PageProgram(0x200000, (uint8_t*)data[0], strlen(data[0]));
    W25Q64_SectorErase(0x201000);
    W25Q64_PageProgram(0x201000, (uint8_t*)data[1], strlen(data[1]));
    W25Q64_SectorErase(0x202000);
    W25Q64_PageProgram(0x202000, (uint8_t*)data[2], strlen(data[2]));
    W25Q64_SectorErase(0x203000);                   
    W25Q64_PageProgram(0x203000, (uint8_t*)data[3], strlen(data[3]));
    W25Q64_SectorErase(0x204000);                   
    W25Q64_PageProgram(0x204000, (uint8_t*)data[4], strlen(data[4]));
    W25Q64_SectorErase(0x205000);                 
    W25Q64_PageProgram(0x205000, (uint8_t*)data[5], strlen(data[5]));
}                                                   

void Mag_Bias_Read(void)
{
    char data[6][10] = {0};
    W25Q64_ReadData(0x200000, (uint8_t*)data[0], 9);
    W25Q64_ReadData(0x201000, (uint8_t*)data[1], 9);
    W25Q64_ReadData(0x202000, (uint8_t*)data[2], 9);
    W25Q64_ReadData(0x203000, (uint8_t*)data[3], 9);
    W25Q64_ReadData(0x204000, (uint8_t*)data[4], 9);
    W25Q64_ReadData(0x205000, (uint8_t*)data[5], 9);
    for(uint8_t i = 0; i<6; i++)
    {
        sscanf(data[i],"%9f", &magbias[i]);
    }
    
}



