#ifndef __MPU6050_H
#define __MPU6050_H

void MPU9520_Init(void);
void MPU9520_GetData(int16_t* accx, int16_t* accy, int16_t* accz,
                     int16_t* gyx, int16_t* gyy, int16_t* gyz);
void MPU9520_Caliberation(void);
void MPU9520_GetCaliberateData(int16_t* accx, int16_t* accy, int16_t* accz,
                     int16_t* gyx, int16_t* gyy, int16_t* gyz);
void MPU9520_GetMag(float* magx, float* magy, float* magz);
void MPU9520_GetCaliberateMag(float* magx, float* magy, float* magz);
void Mag_Caliberate(void);
void Mag_Bias_Store(void);
void Mag_Bias_Read(void);

#endif
