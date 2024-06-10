#include "imu.h"
#include "sensors_types.h"
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include "maths.h"
/**	
 * 姿态解算规则如下：
 *     ROLL  = 绕X轴旋转，右手定则，逆时针为正顺时针为负。
 *     PITCH = 绕Y轴旋转，右手定则，逆时针为正顺时针为负。
 *     YAW   = 绕Z轴旋转，右手定则，逆时针为正顺时针为负。
 */

#define DCM_KP_ACC			0.600f		//加速度补偿陀螺仪PI参数
#define DCM_KI_ACC			0.005f

#define SPIN_RATE_LIMIT     20			//旋转速率

static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;//四元数
static float rMat[3][3];//四元数的旋转矩阵

//四元数转换为旋转矩阵
/* R =  [1 - 2y^2 - 2z^2,     2xy - 2wz,     2xz + 2wy]
        [    2xy + 2wz, 1 - 2x^2 - 2z^2,     2yz - 2wx]
        [    2xz - 2wy,     2yz + 2wx, 1 - 2x^2 - 2y^2] */
static void imuComputeRotationMatrix(void)
{
    float q1q1 = q1 * q1;
    float q2q2 = q2 * q2;
    float q3q3 = q3 * q3;

    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q0q3 = q0 * q3;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q3 = q2 * q3;

    rMat[0][0] = 1.0f - 2.0f * (q2q2 + q3q3);
    rMat[0][1] = 2.0f * (q1q2 - q0q3);
    rMat[0][2] = 2.0f * (q0q2 + q1q3);
    rMat[1][0] = 2.0f * (q1q2 + q0q3);
    rMat[1][1] = 1.0f - 2.0f * (q1q1 + q3q3);
    rMat[1][2] = 2.0f * (q2q3 - q0q1);
    rMat[2][0] = 2.0f * (q1q3 - q0q2);
    rMat[2][1] = 2.0f * (q0q1 + q2q3);
    rMat[2][2] = 1.0f - 2.0f * (q1q1 + q2q2);
}

static float invSqrt(float x)
{
    return 1.0f / sqrtf(x);
}

//四元数归一化
static void imuMahonyAHRSupdate(float gx, float gy, float gz, 
                                float ax, float ay, float az, 
                                float mx, float my, float mz, 
                                int useMag, float dt)
{
    static float integralAccX = 0.0f, integralAccY = 0.0f, integralAccZ = 0.0f; //加速度计积分误差
    static float integralMagX = 0.0f, integralMagY = 0.0f, integralMagZ = 0.0f; //磁力计积分误差
    float ex, ey, ez;

    const float spin_rate_sq = sq(gx) + sq(gy) + sq(gz); //计算旋转速率（rad/s）

    //Step 1: Yaw correction
    if (useMag) 
	{
		
	}

    //Step 2: Roll and pitch correction
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {
        //单位化加速度计测量值
        const float accRecipNorm = invSqrt(sq(ax) + sq(ay) + sq(az));
        ax *= accRecipNorm;
        ay *= accRecipNorm;
        az *= accRecipNorm;

        //加速度计测量值与重力方向的误差，向量叉乘
        ex = ay * rMat[2][2] - az * rMat[2][1];
        ey = az * rMat[2][0] - ax * rMat[2][2];
        ez = ax * rMat[2][1] - ay * rMat[2][0];

        //累计误差补偿
        if(DCM_KI_ACC > 0.0f)
        {
            integralAccX += ex * DCM_KI_ACC * dt;
            integralAccY += ey * DCM_KI_ACC * dt;
            integralAccZ += ez * DCM_KI_ACC * dt;
            gx += integralAccX;
            gy += integralAccY;
            gz += integralAccZ;
        }

        //误差补偿
        gx += ex * DCM_KP_ACC;
        gy += ey * DCM_KP_ACC;
        gz += ez * DCM_KP_ACC;
    }

    //一阶近似算法，四元数运动学方程的离散化形式和积分
    gx*= (0.5f * dt);
    gy*= (0.5f * dt);
    gz*= (0.5f * dt);

    const float qa = q0;
    const float qb = q1;
    const float qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    //四元数归一化
    const float quatRecipNorm = invSqrt(sq(q0) + sq(q1) + sq(q2) + sq(q3));
    q0 *= quatRecipNorm;
    q1 *= quatRecipNorm;
    q2 *= quatRecipNorm;
    q3 *= quatRecipNorm;

    //四元数转换为旋转矩阵
    imuComputeRotationMatrix();
}

//更新欧拉角
static void imuUpdateEulerAngles(attitude_t *attitude)
{
    //计算欧拉角
    attitude->roll = RADIANS_TO_DEGREES(atan2_approx(rMat[2][1], rMat[2][2]));
    attitude->pitch = RADIANS_TO_DEGREES((0.5f*M_PIf) - acos_approx(-rMat[2][0]));//arcsin=0.5pi = arccos
    attitude->yaw = RADIANS_TO_DEGREES(atan2_approx(rMat[1][0], rMat[0][0]));

    if (attitude->yaw < 0.0f)
    {
        attitude->yaw +=360.0f;
    }
}

void imuUpdateAttitude(const sensorData_t *sensorData, attitude_t *attitude, float dt)
{
    Axis3f acc = sensorData->acc;
    Axis3f gyro = sensorData->gyro;
    Axis3f mag = sensorData->mag;

    //角速度单位由度每秒转换为弧度每秒
    gyro.x = gyro.x * DEG2RAD;
    gyro.y = gyro.y * DEG2RAD;
    gyro.z = gyro.z * DEG2RAD;

    //计算四元数和旋转矩阵
    imuMahonyAHRSupdate(gyro.x, gyro.y, gyro.z, acc.x, acc.y, acc.z, mag.x, mag.y, mag.z, 0, dt);

    //更新欧拉角
    imuUpdateEulerAngles(&attitude);
}