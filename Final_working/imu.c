
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "inc/common.h"
#include "inc/tmrsys.h"
#include "inc/adxl345.h"
#include "inc/i2c_IMU.h"


float imu_InvSqrt(float number);

void imu_Init(void) {
	adxl345_Config();

}


