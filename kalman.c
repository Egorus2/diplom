#include "stm32f4xx.h"                  // Device header
#include "system_stm32f4xx.h"

#include "kalman.h"

void quatMultiply(float *qr, float *q1, float *q2) {
	qr[0] = q2[0] * q1[0] - q2[1] * q1[1] - q2[2] * q1[2] - q2[3] * q1[3];
	qr[1] = q2[0] * q1[1] + q2[1] * q1[0] - q2[2] * q1[3] + q2[3] * q1[2];
	qr[2] = q2[0] * q1[2] + q2[1] * q1[3] + q2[2] * q1[0] - q2[3] * q1[1];
	qr[3] = q2[0] * q1[3] - q2[1] * q1[2] + q2[2] * q1[1] + q2[3] * q1[0];
}


void normalizeQuat(float *qr, float *q) {
	float norm;

	norm = 1.0f / sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);

	qr[0] *= norm;
	qr[1] *= norm;
	qr[2] *= norm;
	qr[3] *= norm;
}


