#ifndef SENSORS_H
#define SENSORS_H

#include <stdbool.h>
#include "sensors_types.h"

bool sensorsReadGyro(Axis3f *gyro);
bool sensorsReadAcc(Axis3f *acc);
void sensorsTask(void *parameter);
void sensorsAcquire(sensorData_t *sensors);

#endif // SENSORS_H