#ifndef IMU_H
#define IMU_H

void imuUpdateAttitude(const sensorData_t *sensorData, attitude_t *attitude, float dt);

#endif // IMU_H