#include "sensors.h"

/**	
 * 姿态解算规则如下：
 *     ROLL  = 绕X轴旋转，右手定则，逆时针为正顺时针为负。
 *     PITCH = 绕Y轴旋转，右手定则，逆时针为正顺时针为负。
 *     YAW   = 绕Z轴旋转，右手定则，逆时针为正顺时针为负。
 */

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