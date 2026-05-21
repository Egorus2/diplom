//=====================================================================================================
// MadgwickAHRS.h
//=====================================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=====================================================================================================
#ifndef HRS_h
#define HRS_h

//----------------------------------------------------------------------------------------------------
// Variable declaration
			// algorithm gain
extern volatile float kq0, kq1, kq2, kq3;	// quaternion of sensor frame relative to auxiliary frame

//---------------------------------------------------------------------------------------------------
// Function declarations

void AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void AHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);

#endif
//=====================================================================================================
// End of file
//=====================================================================================================
