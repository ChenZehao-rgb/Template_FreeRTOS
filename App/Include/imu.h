#ifndef IMU_H
#define IMU_H

#include "sensors_types.h"

void imuUpdateAttitude(const sensorData_t *sensorData, attitude_t *attitude, float dt);

#endif // IMU_H