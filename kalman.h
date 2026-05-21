#ifndef KALMAN_H
#define KALMAN_H

#include <stdint.h>

    void quatMultiply(float *qr, float *q1, float *q2);
    void normalizeQuat(float *qr, float *q);
    void rotateVectorByQuat(float *vr, const float *v, const float *q);
    void quatToEuler(const float *q, float *roll, float *pitch, float *yaw);

    void crossVector3(float *vr, const float *va, const float *vb);
    float dotVector3(const float *va, const float *vb);
    void normalizeVec3(float *vr, const float *v);

    void log_mapQuat_fast(const float *q, float *omega);
    void exp_mapQuat(const float *omega, float *q);
#endif 