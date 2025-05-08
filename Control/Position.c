#include "Encoder.h"
#include "Angle.h"
#include "arm_const_structs.h"
#include "arm_math.h"

float vencoder;
void Speed_Encoder(void)
{
    float vr, vl;
    vr = EncoderR*3.1416f*3.3f/4.5;
    vl = EncoderL*3.2416f*3.3f/4.5;
    vencoder = (vl+vr)/2;
}

float px, py;

float X[4];
float Z[2];
float P[16] = {1,0,0,0,
               0,1,0,0,
               0,0,0.1,0,
               0,0,0,0.1};

float F[16] = {1,0,0,0,
               0,1,0,0,
               0,0,1,0,
               0,0,0,1};
float FT[16];
float H[8] = {0,0,1,0,
              0,0,0,1};
float HT[8];
float Q[16] = {0.01,  0   ,  0   , 0    , 
               0   ,  0.01,  0   , 0    ,
               0   ,  0   ,  0.01 , 0   ,  
               0   ,  0   ,  0   , 0.005};

float R[4] = {0.1, 0,
              0,     0.05};
float K[8];
float S1[8];
float S2[8];
float S3[4];
float S4[2];
float S5[4];
float S6[16];
              
float I[16] = {1,0,0,0,
               0,1,0,0,
               0,0,1,0,
               0,0,0,1};


               
uint8_t initflag;
void EKF_Position(void)
{
    arm_matrix_instance_f32 X_matrix;
    arm_matrix_instance_f32 Z_matrix;
    arm_matrix_instance_f32 P_matrix;
    arm_matrix_instance_f32 F_matrix;
    arm_matrix_instance_f32 FT_matrix;
    arm_matrix_instance_f32 Q_matrix;
    arm_matrix_instance_f32 H_matrix;
    arm_matrix_instance_f32 HT_matrix;
    arm_matrix_instance_f32 R_matrix;
    arm_matrix_instance_f32 K_matrix;
    arm_matrix_instance_f32 I_matrix;
    arm_matrix_instance_f32 S1_matrix;
    arm_matrix_instance_f32 S2_matrix;
    arm_matrix_instance_f32 S3_matrix;
    arm_matrix_instance_f32 S4_matrix;
    arm_matrix_instance_f32 S5_matrix;
    arm_matrix_instance_f32 S6_matrix;
        
    arm_mat_init_f32(&X_matrix, 4, 1, X);
    arm_mat_init_f32(&Z_matrix, 2, 1, Z);
    arm_mat_init_f32(&P_matrix, 4, 4, P);
    arm_mat_init_f32(&F_matrix, 4, 4, F);
    arm_mat_init_f32(&FT_matrix, 4, 4, FT);
    arm_mat_init_f32(&H_matrix, 2, 4, H);
    arm_mat_init_f32(&HT_matrix, 4, 2, HT);
    arm_mat_trans_f32(&H_matrix, &HT_matrix);
    arm_mat_init_f32(&Q_matrix, 4, 4, Q);
    arm_mat_init_f32(&R_matrix, 2, 2, R);
    arm_mat_init_f32(&K_matrix, 4, 2, K);
    arm_mat_init_f32(&S1_matrix, 4, 2, S1);
    arm_mat_init_f32(&S2_matrix, 2, 4, S2);
    arm_mat_init_f32(&S3_matrix, 2, 2, S3);
    arm_mat_init_f32(&S4_matrix, 2, 1, S4);
    arm_mat_init_f32(&S5_matrix, 4, 1, S5);
    arm_mat_init_f32(&S6_matrix, 4, 4, S6);
    arm_mat_init_f32(&I_matrix, 4, 4, I);
    
    Speed_Encoder();
    
    //F阵更新
    F[2] = cosf(X[3])*sampletime;
    F[3] = -X[2]*sinf(X[3])*sampletime;
    F[6] = sinf(X[3])*sampletime;
    F[7] = X[2]*cosf(X[3])*sampletime;
    arm_mat_trans_f32(&F_matrix, &FT_matrix);
    
    //状态外推
    X[0] += X[2]*cosf(X[3])*sampletime;
    X[1] += X[2]*sinf(X[3])*sampletime;
    
    //观测方程
    Z[0] = vencoder;
    Z[1] = yaw_k;

    //协方差外推
    arm_mat_mult_f32(&F_matrix, &P_matrix, &P_matrix);
    arm_mat_mult_f32(&P_matrix, &FT_matrix, &P_matrix);
    arm_mat_add_f32(&P_matrix, &Q_matrix, &P_matrix);
    
    //卡尔曼增益
    arm_mat_mult_f32(&P_matrix, &HT_matrix, &S1_matrix);
    arm_mat_mult_f32(&H_matrix, &P_matrix, &S2_matrix);
    arm_mat_mult_f32(&S2_matrix, &HT_matrix, &S3_matrix);
    arm_mat_add_f32(&S3_matrix, &R_matrix, &S3_matrix);
    arm_mat_inverse_f32(&S3_matrix, &S3_matrix);
    arm_mat_mult_f32(&S1_matrix, &S3_matrix, &K_matrix);
    
    //状态更新
    arm_mat_mult_f32(&H_matrix, &X_matrix, &S4_matrix);
    arm_mat_sub_f32(&Z_matrix, &S4_matrix, &S4_matrix);
    arm_mat_mult_f32(&K_matrix, &S4_matrix, &S5_matrix);
    arm_mat_add_f32(&X_matrix, &S5_matrix, &X_matrix);
    
    //协方差更新
    arm_mat_mult_f32(&K_matrix, &H_matrix, &S6_matrix);
    arm_mat_sub_f32(&I_matrix, &S6_matrix, &S6_matrix);
    arm_mat_mult_f32(&S6_matrix, &P_matrix, &P_matrix);
//    
    py = X[0];
    px = X[1];
}

void Get_Position(void)
{
    Speed_Encoder();
    py += vencoder*cosf(yaw_k)*sampletime;
    px += vencoder*sinf(yaw_k)*sampletime;
}





