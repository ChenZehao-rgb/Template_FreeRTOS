#ifndef SENSORS_H
#define SENSORS_H

//姿态数据结构体
typedef struct
{
    float roll;
    float pitch;
    float yaw;
} attitude_t;

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


//传感器数据结构体
typedef struct
{
    Axis3f acc; //加速度计
    Axis3f gyro; //陀螺仪
    Axis3f mag; //磁力计
} sensorData_t;

void imuUpdateAttitude(const sensorData_t *sensorData, attitude_t *attitude, float dt);

#endif // SENSORS_H