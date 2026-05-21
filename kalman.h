#ifndef KALMAN_H
#define KALMAN_H

#include <stdint.h>
    
    #define M_PI 3.14159265358979323846f
    
    void quatMultiply(float *qr, float *q1, float *q2);
    void normalizeQuat(float *qr, float *q);
    void rotateVectorByQuat(float *vr, float *v, float *q);

    void crossVector3(float *vr, float *va, float *vb);
    float dotVector3(float *va, float *vb);
    void normalizeVec3(float *vr, float *v);

    void log_mapQuat(float *qIn, float *omega);
    void exp_mapQuat(float *omega, float *q);
#endif 