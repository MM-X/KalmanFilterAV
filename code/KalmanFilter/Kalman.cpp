#include "Kalman.h"

Kalman::Kalman() {
    /* Q_velocity   速度过程噪声方差
     * Q_acc        加速度过程噪声方差
     * R_measure    测量方差
     */
    Q_velocity = 0.05f;
    Q_acc = 0.05f;
    R_measure = 0.2f;

    velocity = 0.0f;    // 速度初始值
    acc = 0.0f;         // 加速度初始值

    P[0][0] = 0.0f;     // 设置为0假设我们知道精确的初始值
    P[0][1] = 0.0f;
    P[1][0] = 0.0f;
    P[1][1] = 0.0f;
};

// 速度 m/s   加速度 m/s^2    时间 s
float Kalman::getVelocity(float VelocityMeasured, float dt) {
    //计算先验状态
    /* Step 2 */
    velocity += dt * acc;

    // 更新先验误差协方差矩阵
    /* Step 2 */
    P[0][0] += dt * (dt*P[1][1] + P[0][1] + P[1][0] + Q_velocity);
    P[0][1] += dt * P[1][1];
    P[1][0] += dt * P[1][1];
    P[1][1] += Q_acc * dt;

    // 计算innovation协方差矩阵
    /* Step 4 */
    float S = P[0][0] + R_measure;
    // 计算卡尔曼增益
    /* Step 5 */
    float K[2];             // Kalman gain - This is a 2x1 vector
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;

    // 计算innovation
    /* Step 3 */
    float y = VelocityMeasured - velocity; // velocity difference
    
    //计算后验状态
    /* Step 6 */
    velocity += K[0] * y;
    acc += K[1] * y;

    // 更新后验误差协方差矩阵
    /* Step 7 */
    float P00_temp = P[0][0];
    float P01_temp = P[0][1];

    P[0][0] -= K[0] * P00_temp;
    P[0][1] -= K[0] * P01_temp;
    P[1][0] -= K[1] * P00_temp;
    P[1][1] -= K[1] * P01_temp;

    return velocity;
};

void Kalman::setVelocity(float velocity) { this->velocity = velocity; };        //设置初始速度
void Kalman::setAcc(float acc) { this->acc = acc; };                            //设置初始加速度

/* These are used to tune the Kalman filter */
void Kalman::setQvelocity(float Q_velocity) { this->Q_velocity = Q_velocity; };
void Kalman::setQacc(float Q_acc) { this->Q_acc = Q_acc; };
void Kalman::setRmeasure(float R_measure) { this->R_measure = R_measure; };

float Kalman::getQvelocity() { return this->Q_velocity; };
float Kalman::getQacc() { return this->Q_acc; };
float Kalman::getRmeasure() { return this->R_measure; };
