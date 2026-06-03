#include "stm32f4xx.h"                  // Device header
#include "system_stm32f4xx.h"

#include "kalman.h"
#include <math.h>
#include <string.h>  // ??? memset

void Kalman_Init(Kalman_t *kalman)
{
    kalman->Q_angle = 0.001f;
    kalman->Q_bias = 0.003f;
    kalman->R_measure = 0.03f;

    kalman->angle = 0.0f;
    kalman->bias = 0.0f;

    kalman->P[0][0] = 0.0f;
    kalman->P[0][1] = 0.0f;
    kalman->P[1][0] = 0.0f;
    kalman->P[1][1] = 0.0f;
}

float Kalman_GetAngle(Kalman_t *kalman,
                      float newAngle,
                      float newRate,
                      float dt)
{
    // Predict

    kalman->rate = newRate - kalman->bias;
    kalman->angle += dt * kalman->rate;

    // Update estimation error covariance

    kalman->P[0][0] += dt * (dt*kalman->P[1][1]
                          - kalman->P[0][1]
                          - kalman->P[1][0]
                          + kalman->Q_angle);

    kalman->P[0][1] -= dt * kalman->P[1][1];
    kalman->P[1][0] -= dt * kalman->P[1][1];
    kalman->P[1][1] += kalman->Q_bias * dt;

    // Innovation

    float S = kalman->P[0][0] + kalman->R_measure;

    // Kalman gain

    float K[2];

    K[0] = kalman->P[0][0] / S;
    K[1] = kalman->P[1][0] / S;

    // Angle difference

    float y = newAngle - kalman->angle;

    // Update angle and bias

    kalman->angle += K[0] * y;
    kalman->bias += K[1] * y;

    // Update error covariance matrix

    float P00_temp = kalman->P[0][0];
    float P01_temp = kalman->P[0][1];

    kalman->P[0][0] -= K[0] * P00_temp;
    kalman->P[0][1] -= K[0] * P01_temp;
    kalman->P[1][0] -= K[1] * P00_temp;
    kalman->P[1][1] -= K[1] * P01_temp;

    return kalman->angle;
}