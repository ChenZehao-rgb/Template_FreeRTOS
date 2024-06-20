#ifndef SENSORS_TYPES_H
#define SENSORS_TYPES_H

#include <stdint.h>

//传感器参数类型定义
typedef union sensors
{
    struct
    {
        float x;
        float y;
        float z;
    };
    float axis[3];
} Axis3f;



typedef union
{
	struct
	{
		int16_t x;
		int16_t y;
		int16_t z;
	};
	int16_t axis[3];
} Axis3i16;

typedef union
{
	struct
	{
		int32_t x;
		int32_t y;
		int32_t z;
	};
	int32_t axis[3];
} Axis3i32;

typedef union
{
	struct
	{
		int64_t x;
		int64_t y;
		int64_t z;
	};
	int64_t axis[3];
} Axis3i64;

//姿态数据结构体
typedef struct
{
    float roll;
    float pitch;
    float yaw;
	short gyro_x;
	short gyro_y;
} attitude_t;

//传感器数据结构体
typedef struct
{
    Axis3f acc; //加速度计
    Axis3f gyro; //陀螺仪
    Axis3f mag; //磁力计
} sensorData_t;


#endif // SENSORS_TYPES_H