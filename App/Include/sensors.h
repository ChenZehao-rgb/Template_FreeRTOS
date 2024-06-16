#ifndef SENSORS_H
#define SENSORS_H

#include <stdbool.h>
#include "sensors_types.h"

void sensorsTask(void *parameter);
void sensorsAcquire(attitude_t *attitude);

#endif // SENSORS_H