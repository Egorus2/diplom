#include "stm32f4xx.h"                  // Device header
#include "system_stm32f4xx.h"

#include "kalman.h"
#include <math.h>
#include <string.h>  // ??? memset

void quatMultiply(float *qr, float *q1, float *q2) {
	qr[0] = q2[0] * q1[0] - q2[1] * q1[1] - q2[2] * q1[2] - q2[3] * q1[3];
	qr[1] = q2[0] * q1[1] + q2[1] * q1[0] - q2[2] * q1[3] + q2[3] * q1[2];
	qr[2] = q2[0] * q1[2] + q2[1] * q1[3] + q2[2] * q1[0] - q2[3] * q1[1];
	qr[3] = q2[0] * q1[3] - q2[1] * q1[2] + q2[2] * q1[1] + q2[3] * q1[0];
}


void normalizeQuat(float *qr, float *q) {
    float norm = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if (norm < 1e-6f) { 
        qr[0] = 1.0f; qr[1] = 0.0f; qr[2] = 0.0f; qr[3] = 0.0f;
        return;
    }
    norm = 1.0f / norm;
    qr[0] = q[0] * norm;  
    qr[1] = q[1] * norm;
    qr[2] = q[2] * norm;
    qr[3] = q[3] * norm;
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


#include <string.h>   // for memcpy
#include <math.h>     // for sqrtf, atan2f, asinf, etc.

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// ============================================================================
// Helper macros and inline functions
// ============================================================================

static inline float sq(float x) { 
    return x * x; 
}

// Zero out a 7x7 matrix
static void mat_zero_7x7(float m[7][7]) {
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 7; j++) {
            m[i][j] = 0.0f;
        }
    }
}

// Set a 7x7 matrix to identity
static void mat_identity_7x7(float m[7][7]) {
    mat_zero_7x7(m);
    for (int i = 0; i < 7; i++) {
        m[i][i] = 1.0f;
    }
}

// Matrix multiplication: C = A * B  (all 7x7)
static void mat_mul_7x7(float A[7][7], float B[7][7], float C[7][7]) {
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 7; j++) {
            float sum = 0.0f;
            for (int k = 0; k < 7; k++) {
                sum += A[i][k] * B[k][j];
            }
            C[i][j] = sum;
        }
    }
}

// Matrix addition: C = A + B  (all 7x7)
static void mat_add_7x7(float A[7][7], float B[7][7], float C[7][7]) {
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 7; j++) {
            C[i][j] = A[i][j] + B[i][j];
        }
    }
}

// Matrix transpose: AT = A'  (7x7)
static void mat_transpose_7x7(float A[7][7], float AT[7][7]) {
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 7; j++) {
            AT[j][i] = A[i][j];
        }
    }
}

// Analytical inversion of a symmetric 3x3 matrix
// Returns 1 on success, 0 if matrix is singular
static int invert_3x3_sym(float S[3][3], float S_inv[3][3]) {
    // Compute determinant
    float det = S[0][0] * (S[1][1]*S[2][2] - S[1][2]*S[2][1]) - 
                S[0][1] * (S[1][0]*S[2][2] - S[1][2]*S[2][0]) + 
                S[0][2] * (S[1][0]*S[2][1] - S[1][1]*S[2][0]);
    
    // Check for singularity
    if (fabsf(det) < 1e-9f) {
        return 0;
    }
    
    float inv_det = 1.0f / det;
    
    // Cofactor matrix (exploiting symmetry)
    S_inv[0][0] =  (S[1][1]*S[2][2] - S[1][2]*S[2][1]) * inv_det;
    S_inv[0][1] = -(S[0][1]*S[2][2] - S[0][2]*S[2][1]) * inv_det;
    S_inv[0][2] =  (S[0][1]*S[1][2] - S[0][2]*S[1][1]) * inv_det;
    
    S_inv[1][0] = S_inv[0][1];  // symmetry
    S_inv[1][1] =  (S[0][0]*S[2][2] - S[0][2]*S[2][0]) * inv_det;
    S_inv[1][2] = -(S[0][0]*S[1][2] - S[0][2]*S[1][0]) * inv_det;
    
    S_inv[2][0] = S_inv[0][2];
    S_inv[2][1] = S_inv[1][2];
    S_inv[2][2] =  (S[0][0]*S[1][1] - S[0][1]*S[1][0]) * inv_det;
    
    return 1;
}

// ============================================================================
// Filter initialization
// ============================================================================

void EKF_Init(EKF_AHRS *ekf, 
              float *gyro_std_rad,   // gyro noise std dev in rad/s: [x, y, z]
              float *acc_std,        // accel noise std dev in g:      [x, y, z]
              float *mag_std) {      // mag noise std dev in uT:      [x, y, z]
    
    // Initial state: identity quaternion, zero gyro biases
    ekf->x[0] = 1.0f;  // qw
    ekf->x[1] = 0.0f;  // qx
    ekf->x[2] = 0.0f;  // qy
    ekf->x[3] = 0.0f;  // qz
    ekf->x[4] = 0.0f;  // bg_x
    ekf->x[5] = 0.0f;  // bg_y
    ekf->x[6] = 0.0f;  // bg_z
    
    // Initial covariance: high uncertainty for orientation, low for biases
    mat_identity_7x7(ekf->P);
    for (int i = 0; i < 4; i++) {
        ekf->P[i][i] = 0.1f;        // quaternion uncertainty
    }
    for (int i = 4; i < 7; i++) {
        ekf->P[i][i] = sq(0.01f);   // bias uncertainty
    }
    
    // Process noise Q (diagonal, based on gyro noise)
    // Noise propagates to quaternion as: sigma_q ˜ 0.5 * dt * sigma_gyro
    for (int i = 0; i < 3; i++) {
        ekf->Q_gyro[i] = sq(0.5f * ekf->dt * gyro_std_rad[i]);
    }
    
    // Measurement noise R (diagonal)
    for (int i = 0; i < 3; i++) {
        ekf->R_acc[i] = sq(acc_std[i]);
        ekf->R_mag[i] = sq(mag_std[i]);
    }
    
    ekf->is_initialized = 1;
}

// ============================================================================
// Prediction step: integrate gyro, propagate covariance
// ============================================================================

void EKF_Predict(EKF_AHRS *ekf, float gx, float gy, float gz) {
    // Remove estimated bias from gyro measurements
    float wx = gx - ekf->x[4];
    float wy = gy - ekf->x[5];
    float wz = gz - ekf->x[6];
    
    // Quaternion derivative: q_dot = 0.5 * q ? omega
    float dq[4];
    dq[0] = 0.5f * (-ekf->x[1]*wx - ekf->x[2]*wy - ekf->x[3]*wz);
    dq[1] = 0.5f * ( ekf->x[0]*wx + ekf->x[2]*wz - ekf->x[3]*wy);
    dq[2] = 0.5f * ( ekf->x[0]*wy - ekf->x[1]*wz + ekf->x[3]*wx);
    dq[3] = 0.5f * ( ekf->x[0]*wz + ekf->x[1]*wy - ekf->x[2]*wx);
    
    // Euler integration for quaternion
    for (int i = 0; i < 4; i++) {
        ekf->x[i] += ekf->dt * dq[i];
    }
    // Biases are assumed constant over one timestep
    // (no update needed: ekf->x[4..6] stay as-is)
    
    // Normalize quaternion to prevent drift due to numerical errors
    normalizeQuat(ekf->x, ekf->x);
    
    // Compute process Jacobian F = df/dx (linearization around current state)
    mat_identity_7x7(ekf->F);
    
    // Block 4x4: partial derivatives of q_dot w.r.t. quaternion
    ekf->F[0][1] = -0.5f * ekf->dt * wx;
    ekf->F[0][2] = -0.5f * ekf->dt * wy;
    ekf->F[0][3] = -0.5f * ekf->dt * wz;
    
    ekf->F[1][0] =  0.5f * ekf->dt * wx;
    ekf->F[1][2] =  0.5f * ekf->dt * wz;
    ekf->F[1][3] = -0.5f * ekf->dt * wy;
    
    ekf->F[2][0] =  0.5f * ekf->dt * wy;
    ekf->F[2][1] = -0.5f * ekf->dt * wz;
    ekf->F[2][3] =  0.5f * ekf->dt * wx;
    
    ekf->F[3][0] =  0.5f * ekf->dt * wz;
    ekf->F[3][1] =  0.5f * ekf->dt * wy;
    ekf->F[3][2] = -0.5f * ekf->dt * wx;
    
    // Block 4x3: partial derivatives of q_dot w.r.t. gyro biases
    for (int i = 0; i < 3; i++) {
        ekf->F[i][4 + i] = 0.5f * ekf->dt;
    }
    
    // Covariance prediction: P_minus = F * P * F' + Q
    float Ft[7][7], FP[7][7], P_pred[7][7];
    
    mat_transpose_7x7(ekf->F, Ft);
    mat_mul_7x7(ekf->F, ekf->P, FP);
    mat_mul_7x7(FP, Ft, P_pred);
    
    // Add process noise Q (only diagonal terms for quaternion part)
    for (int i = 0; i < 4; i++) {
        P_pred[i][i] += ekf->Q_gyro[i % 3];
    }
    // Small random walk noise for biases
    for (int i = 4; i < 7; i++) {
        P_pred[i][i] += sq(1e-5f);
    }
    
    // Copy predicted covariance back to state
    memcpy(ekf->P, P_pred, sizeof(ekf->P));
}

// ============================================================================
// Update step: correct state using accelerometer (and optionally magnetometer)
// ============================================================================

void EKF_Update(EKF_AHRS *ekf, 
                float ax, float ay, float az, 
                float mx, float my, float mz) {
    
    // === Accelerometer update ===
    
    // Expected gravity vector in body frame: rotate [0, 0, 1] (world) by quaternion
    float g_world[3] = {0.0f, 0.0f, 1.0f};
    float g_body[3];
    rotateVectorByQuat(g_body, g_world, ekf->x);
    
    // Innovation: measurement minus prediction
    float z_acc[3] = {ax, ay, az};
    float innovation[3];
    for (int i = 0; i < 3; i++) {
        innovation[i] = z_acc[i] - g_body[i];
    }
    
    // Measurement Jacobian H = dh/dx (only w.r.t. quaternion, simplified)
    // h(q) = R(q)' * [0, 0, 1]  (projection of gravity into body frame)
    float qw = ekf->x[0], qx = ekf->x[1], qy = ekf->x[2], qz = ekf->x[3];
    
    // Zero out H first
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 7; j++) {
            ekf->H[i][j] = 0.0f;
        }
    }
    
    // Partial derivatives of projected gravity w.r.t. quaternion components
    // Simplified linearization (sufficient for small corrections)
    ekf->H[0][0] = 2.0f * ( qy - qz);   // d(gx)/d(qw)
    ekf->H[0][1] = 2.0f * ( qw + qz);   // d(gx)/d(qx)
    ekf->H[0][2] = 2.0f * (-qw + qy);   // d(gx)/d(qy)
    ekf->H[0][3] = 2.0f * (-qx - qz);   // d(gx)/d(qz)
    
    ekf->H[1][0] = 2.0f * (-qx - qz);   // d(gy)/d(qw)
    ekf->H[1][1] = 2.0f * ( qw - qy);   // d(gy)/d(qx)
    ekf->H[1][2] = 2.0f * ( qw + qx);   // d(gy)/d(qy)
    ekf->H[1][3] = 2.0f * (-qy + qz);   // d(gy)/d(qz)
    
    ekf->H[2][0] = 2.0f * ( qx - qy);   // d(gz)/d(qw)
    ekf->H[2][1] = 2.0f * ( qy + qz);   // d(gz)/d(qx)
    ekf->H[2][2] = 2.0f * (-qx + qz);   // d(gz)/d(qy)
    ekf->H[2][3] = 2.0f * ( qw + qx);   // d(gz)/d(qz)
    
    // Innovation covariance: S = H * P * H' + R
    float HP[3][7], HPHT[3][3], S[3][3];
    
    // HP = H * P
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 7; j++) {
            float sum = 0.0f;
            for (int k = 0; k < 7; k++) {
                sum += ekf->H[i][k] * ekf->P[k][j];
            }
            HP[i][j] = sum;
        }
    }
    
    // HPHT = HP * H'
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            float sum = 0.0f;
            for (int k = 0; k < 7; k++) {
                sum += HP[i][k] * ekf->H[j][k];
            }
            HPHT[i][j] = sum;
        }
    }
    
    // S = HPHT + R (R is diagonal)
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            S[i][j] = HPHT[i][j];
            if (i == j) {
                S[i][j] += ekf->R_acc[i];
            }
        }
    }
    
    // Invert S (3x3)
    float S_inv[3][3];
    if (!invert_3x3_sym(S, S_inv)) {
        return;  // Singular matrix, skip update
    }
    
    // Kalman gain: K = P * H' * S_inv
    float K[7][3];
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 3; j++) {
            float sum = 0.0f;
            for (int k = 0; k < 3; k++) {
                // Compute (P * H')[i][k] first
                float PHt = 0.0f;
                for (int m = 0; m < 7; m++) {
                    PHt += ekf->P[i][m] * ekf->H[k][m];
                }
                sum += PHt * S_inv[k][j];
            }
            K[i][j] = sum;
        }
    }
    
    // State update: x = x + K * innovation
    for (int i = 0; i < 7; i++) {
        float correction = 0.0f;
        for (int j = 0; j < 3; j++) {
            correction += K[i][j] * innovation[j];
        }
        ekf->x[i] += correction;
    }
    
    // Normalize quaternion after correction
    normalizeQuat(ekf->x, ekf->x);
    
    // Covariance update: P = (I - K * H) * P  (simplified Joseph form)
    float KH[7][7], I_KH[7][7];
    
    // KH = K * H
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 7; j++) {
            float sum = 0.0f;
            for (int k = 0; k < 3; k++) {
                sum += K[i][k] * ekf->H[k][j];
            }
            KH[i][j] = sum;
        }
    }
    
    // I_KH = I - KH
    mat_identity_7x7(I_KH);
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 7; j++) {
            I_KH[i][j] -= KH[i][j];
        }
    }
    
    // P = I_KH * P
    float P_new[7][7];
    mat_mul_7x7(I_KH, ekf->P, P_new);
    memcpy(ekf->P, P_new, sizeof(ekf->P));
    
    // === Magnetometer update (optional, same structure as accelerometer) ===
    // To add: replace g_world with local magnetic field vector,
    // recompute innovation and Jacobian, then repeat the update steps.
    // For brevity, omitted here.
}

// ============================================================================
// Output: convert quaternion to Euler angles (degrees)
// ============================================================================

void EKF_GetOrientation(EKF_AHRS *ekf,
                        float *roll_deg, float *pitch_deg, float *yaw_deg) {
    float qw = ekf->x[0], qx = ekf->x[1], qy = ekf->x[2], qz = ekf->x[3];
    
    // Roll (rotation around X axis)
    float sinr_cosp = 2.0f * (qw * qx + qy * qz);
    float cosr_cosp = 1.0f - 2.0f * (qx * qx + qy * qy);
    *roll_deg = atan2f(sinr_cosp, cosr_cosp) * 180.0f / M_PI;
    
    // Pitch (rotation around Y axis)
    float sinp = 2.0f * (qw * qy - qz * qx);
    if (fabsf(sinp) >= 1.0f) {
        // Use 90 degrees if out of range (gimbal lock)
        *pitch_deg = (sinp > 0) ? 90.0f : -90.0f;
    } else {
        *pitch_deg = asinf(sinp) * 180.0f / M_PI;
    }
    
    // Yaw (rotation around Z axis)
    float siny_cosp = 2.0f * (qw * qz + qx * qy);
    float cosy_cosp = 1.0f - 2.0f * (qy * qy + qz * qz);
    *yaw_deg = atan2f(siny_cosp, cosy_cosp) * 180.0f / M_PI;
    
    // Normalize yaw to [0, 360)
    if (*yaw_deg < 0) {
        *yaw_deg += 360.0f;
    }
}
                        