#ifndef _Kalman_h_
#define _Kalman_h_

class Kalman {
public:
    Kalman();

    // 速度 m/s   加速度 m/s^2    时间 s
    float getVelocity(float VelocityMeasured, float dt);

    void setVelocity(float velocity);       //设置初始速度
    void setAcc(float acc);                 //设置初始加速度

    /* These are used to tune the Kalman filter */
    void setQvelocity(float Q_velocity);
    /**
     * setQacc(float Q_acc)
     * Default value (0.05f) is in Kalman.cpp. 
     * Raise this to follow input more closely,
     * lower this to smooth result of kalman filter.
     */
    void setQacc(float Q_acc);
    void setRmeasure(float R_measure);

    float getQvelocity();
    float getQacc();
    float getRmeasure();

private:
    /* 卡尔曼滤波变量 */
    float Q_velocity;   // 速度的过程噪声方差
    float Q_acc;        // 加速度的过程噪声方差
    float R_measure;    // 测量值的方差 - this is actually the variance of the measurement noise

    float velocity;     // 速度   - part of the 2x1 state vector
    float acc;          // 加速度 - part of the 2x1 state vector

    float P[2][2];      // 误差协方差矩阵
};

#endif
