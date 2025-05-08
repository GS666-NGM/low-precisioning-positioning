#ifndef __MPU9520_REG_H
#define __MPU9520_REG_H

#define MPU9520_ADDRESS		    0xD0  //MPU9520从机地址
#define AK8963_ADDRESS        0x18  //磁力计地址

#define	MPU9520_SMPLRT_DIV		0x19
#define	MPU9520_CONFIG		  	0x1A
#define	MPU9520_INT_PIN_CFG 	0x37
#define	MPU9520_GYRO_CONFIG		0x1B
#define	MPU9520_ACCEL_CONFIG	0x1C
#define	MPU9520_ACCEL_CONFIG2	0x1D
#define	AK8963_CNTL1	        0x0A
#define	AK8963_CNTL2	        0x0B

#define	MPU9520_ACCEL_XOUT_H	0x3B
#define	MPU9520_ACCEL_XOUT_L	0x3C
#define	MPU9520_ACCEL_YOUT_H	0x3D
#define	MPU9520_ACCEL_YOUT_L	0x3E
#define	MPU9520_ACCEL_ZOUT_H	0x3F
#define	MPU9520_ACCEL_ZOUT_L	0x40
#define	MPU9520_TEMP_OUT_H		0x41
#define	MPU9520_TEMP_OUT_L		0x42
#define	MPU9520_GYRO_XOUT_H		0x43
#define	MPU9520_GYRO_XOUT_L		0x44
#define	MPU9520_GYRO_YOUT_H		0x45
#define	MPU9520_GYRO_YOUT_L		0x46
#define	MPU9520_GYRO_ZOUT_H		0x47
#define	MPU9520_GYRO_ZOUT_L		0x48
#define	AK8963_XOUT_L		0x03
#define	AK8963_XOUT_H		0x04
#define	AK8963_YOUT_L		0x05
#define	AK8963_YOUT_H		0x06
#define	AK8963_ZOUT_L		0x07
#define	AK8963_ZOUT_H		0x08
#define AK8963_ST2      0x09
#define AK8963_ASAX     0x10
#define AK8963_ASAY     0x11
#define AK8963_ASAZ     0x12

#define	MPU9520_PWR_MGMT_1		0x6B
#define	MPU9520_PWR_MGMT_2		0x6C
#define	MPU9520_WHO_AM_I		  0x75
#define AK8963_WIA            0x00

//偏移量寄存器
#define XA_OFFS_H               0x06
#define XA_OFFS_L               0x07
#define YA_OFFS_H               0x08
#define YA_OFFS_L               0x09
#define ZA_OFFS_H               0x0A
#define ZA_OFFS_L               0x0B
#define XG_OFFS_USRH            0x13
#define XG_OFFS_USRL            0x14
#define YG_OFFS_USRH            0x15
#define YG_OFFS_USRL            0x16
#define ZG_OFFS_USRH            0x17
#define ZG_OFFS_USRL            0x18




#endif
