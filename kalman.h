#ifndef KALMAN_H
#define KALMAN_H

#include <stdint.h>

    #include <math.h>


    #define EKF_STATE_DIM     7   // [q0,q1,q2,q3, bg_x, bg_y, bg_z]
    #define EKF_MEAS_DIM      6   // [acc_x, acc_y, acc_z, mag_x, mag_y, mag_z]
    #define EKF_DT_DEFAULT    0.08f  // 800 Hz
    #define M_PI 3.14159265358979323846f

    typedef struct {

        float x[EKF_STATE_DIM];          // [q, bg]
        

        float P[EKF_STATE_DIM][EKF_STATE_DIM];
        

        float Q_gyro[3];     // [s²_gx, s²_gy, s²_gz] ? (???/?)²
        float R_acc[3];      // [s²_ax, s²_ay, s²_az] ? g²
        float R_mag[3];      // [s²_mx, s²_my, s²_mz] ? (µT)²
        

        float F[EKF_STATE_DIM][EKF_STATE_DIM];  
        float H[3][EKF_STATE_DIM];              
        
        float dt;
        int is_initialized;
        
    } EKF_AHRS;

    // === API ===
    void EKF_Init(EKF_AHRS *ekf, 
                  float *gyro_std_rad,  // [s_x, s_y, s_z] ? ???/?
                  float *acc_std,        // [s_x, s_y, s_z] ? g
                  float *mag_std);       // [s_x, s_y, s_z] ? µT

    void EKF_Predict(EKF_AHRS *ekf, 
                     float gx_rad_s, float gy_rad_s, float gz_rad_s);

    void EKF_Update(EKF_AHRS *ekf,
                    float ax_g, float ay_g, float az_g,     
                    float mx_ut, float my_ut, float mz_ut); 

    void EKF_GetOrientation(EKF_AHRS *ekf,
                            float *roll_deg, float *pitch_deg, float *yaw_deg);
    

    
    void quatMultiply(float *qr, float *q1, float *q2);
    void normalizeQuat(float *qr, float *q);
    void rotateVectorByQuat(float *vr, float *v, float *q);

    void crossVector3(float *vr, float *va, float *vb);
    float dotVector3(float *va, float *vb);
    void normalizeVec3(float *vr, float *v);

    void log_mapQuat(float *qIn, float *omega);
    void exp_mapQuat(float *omega, float *q);
#endif 