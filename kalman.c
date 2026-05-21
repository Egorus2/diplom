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

void rotateVectorByQuat(float *vr, float *v, float *q) {
	float w, x, y, z;

	w = q[0];
	x = q[1];
	y = q[2];
	z = q[3];

	vr[0] = w * w*v[0] + 2.0f*y*w*v[2] - 2.0f*z*w*v[1] + x * x*v[0] + 2.0f*y*x*v[1] + 2.0f*z*x*v[2] - z * z*v[0] - y * y*v[0];
	vr[1] = 2.0f*x*y*v[0] + y * y*v[1] + 2.0f*z*y*v[2] + 2.0f*w*z*v[0] - z * z*v[1] + w * w*v[1] - 2.0f*x*w*v[2] - x * x*v[1];
	vr[2] = 2.0f*x*z*v[0] + 2.0f*y*z*v[1] + z * z*v[2] - 2.0f*w*y*v[0] - y * y*v[2] + 2.0f*w*x*v[1] - x * x*v[2] + w * w*v[2];
}

void crossVector3(float *vr, float *va, float *vb) {
	vr[0] = va[1] * vb[2] - vb[1] * va[2];
	vr[1] = va[2] * vb[0] - vb[2] * va[0];
	vr[2] = va[0] * vb[1] - vb[0] * va[1];
}

float dotVector3(float *va, float *vb) {
	return va[0] * vb[0] + va[1] * vb[1] + va[2] * vb[2];
}

void normalizeVec3(float *vr, float *v) {
	float norm;

	norm = sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);

	vr[0] = v[0] / norm;
	vr[1] = v[1] / norm;
	vr[2] = v[2] / norm;
}

void log_mapQuat(float *qIn, float *omega) {

	// define these compile time constants to avoid std::abs:
	static const float twoPi = 2.0f * M_PI, NearlyOne = 1.0f - 1e-8f,
		NearlyNegativeOne = -1.0f + 1e-8f;

	const float qw = qIn[0];
	// See Quaternion-Logmap.nb in doc for Taylor expansions
	if (qw >= NearlyOne) {
		// Taylor expansion of (angle / s) at 1
		// (2 + 2 * (1-qw) / 3) * q.vec();
		float tmp = (8.0f / 3.0f - 2.0f / 3.0f * qw);
		omega[0] = tmp * qIn[1];
		omega[1] = tmp * qIn[2];
		omega[2] = tmp * qIn[3];
	}
	else if (qw <= NearlyNegativeOne) {
		// Taylor expansion of (angle / s) at -1
		// (-2 - 2 * (1 + qw) / 3) * q.vec();
		float tmp = (-8.0f / 3.0f - 2.0f / 3.0f * qw);
		omega[0] = tmp * qIn[1];
		omega[1] = tmp * qIn[2];
		omega[2] = tmp * qIn[3];
	}
	else {
		// Normal, away from zero case
		float angle = 2.0f * acosf(qw), s = sqrtf(1.0f - qw * qw);
		// Important:  convert to [-pi,pi] to keep error continuous
		if (angle > M_PI)
			angle -= twoPi;
		else if (angle < -M_PI)
			angle += twoPi;
		float tmp = (angle / s);
		omega[0] = tmp * qIn[1];
		omega[1] = tmp * qIn[2];
		omega[2] = tmp * qIn[3];
	}

}


void exp_mapQuat(float *omega, float *q) {
	float vec[3];
	float theta2 = dotVector3(omega, omega);
	if (theta2 > 5.96e-8f) {
		float theta = sqrtf(theta2);
		float ha = 0.5f * theta;
		float tmp = (sinf(ha) / theta);

		vec[0] = tmp * omega[0];
		vec[1] = tmp * omega[1];
		vec[2] = tmp * omega[2];
		q[0] = cosf(ha);
		q[1] = vec[0];
		q[2] = vec[1];
		q[3] = vec[2];
	}
	else {
		// first order approximation sin(theta/2)/theta = 0.5
		vec[0] = 0.5f * omega[0];
		vec[1] = 0.5f * omega[1];
		vec[2] = 0.5f * omega[2];
		q[0] = 1.0f;
		q[1] = vec[0];
		q[2] = vec[1];
		q[3] = vec[2];
	}
}

