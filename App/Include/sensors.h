#ifndef SENSORS_H
#define SENSORS_H



void imuUpdateAttitude(const sensorData_t *sensorData, attitude_t *attitude, float dt);

#endif // SENSORS_H