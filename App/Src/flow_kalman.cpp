/*
 * flow_kalman.cpp
 *
 *  Created on: 2026年8月14日
 *      Author: bitstream
 */

#include "flow_kalman.h"
#include <math.h>
#include <cstring>

// 初始化卡尔曼滤波器
void FlowKalman_Init(FlowKalman *kf) {
    for (int i = 0; i < 4; i++) {
        kf->x[i] = 0.0f;
        for (int j = 0; j < 4; j++) {
            kf->P[i][j] = (i == j) ? 1.0f : 0.0f; // 初始协方差
        }
    }
    // 过程噪声 (调参点：值越大，越相信传感器；值越小，越相信预测模型)
    kf->Q[0] = 0.03f; // PosX (横向) 噪声
    kf->Q[1] = 0.08f; // VelX (横向) 噪声
    kf->Q[2] = 0.03f; // PosY (纵向) 噪声
    kf->Q[3] = 0.08f; // VelY (纵向) 噪声

    // 光流测量噪声基础值 (调参点)
    kf->R_flow = 0.1f;

    // GPS 噪声初始化
    kf->R_gps_pos = 2.0f;  // 认为 GPS 位置误差在 2 米左右
    kf->R_gps_vel = 0.3f;  // 认为 GPS 速度误差在 0.3 米/秒左右
}

// 2x2 矩阵求逆
void Matrix2x2_Inverse(Matrix2x2 *m) {
    float det = m->m[0][0] * m->m[1][1] - m->m[0][1] * m->m[1][0];
    if (fabsf(det) < 1e-6f) det = 1e-6f; // 防止除0
    float inv_det = 1.0f / det;
    float temp = m->m[0][0];
    m->m[0][0] = m->m[1][1] * inv_det;
    m->m[1][1] = temp * inv_det;
    m->m[0][1] = -m->m[0][1] * inv_det;
    m->m[1][0] = -m->m[1][0] * inv_det;
}

/**
 * @brief 预测步：使用 IMU 加速度推算 (高频调用，如 1000Hz)
 *        所有参数均为机体坐标系！
 * @param ax  机体横向加速度 (右为正)
 * @param ay  机体纵向加速度 (前为正)
 * @param yaw_rate 偏航角速度 (弧度/秒，顺时针/右转为正)
 * @param dt 距离上次预测的时间(秒)
 */
void FlowKalman_Predict(FlowKalman *kf, float ax, float ay, float yaw_rate, float dt) {
    // 1. 计算坐标系旋转补偿量 (科里奥利效应补偿)
    // 当机体顺时针旋转(yaw_rate > 0)时，原本向前的速度会变成向右的速度
    float w_dt = yaw_rate * dt;

    // 2. 速度预测 (必须加上旋转补偿)
    // vx_new = vx + ax*dt + vy * w_dt
    // vy_new = vy + ay*dt - vx * w_dt
    float vx_pred = kf->x[1] + ax * dt + kf->x[3] * w_dt;
    float vy_pred = kf->x[3] + ay * dt - kf->x[1] * w_dt;

    // 3. 位置预测 (使用更新前的速度进行积分)
    kf->x[0] += kf->x[1] * dt + 0.5f * ax * dt * dt; // 机体横向位置 px
    kf->x[2] += kf->x[3] * dt + 0.5f * ay * dt * dt; // 机体纵向位置 py

    // 4. 更新速度状态
    kf->x[1] = vx_pred;
    kf->x[3] = vy_pred;

    // 5. 协方差预测 P = A * P * A^T + Q
    // 注意：为了单片机运算效率，这里保持了原有的分块解耦近似计算
    // (忽略了 X 和 Y 之间的协方差交叉项，在大多数飞行场景下这是可接受的)
    float p00 = kf->P[0][0] + kf->P[1][0] * dt + (kf->P[0][1] + kf->P[1][1] * dt) * dt + kf->Q[0];
    float p01 = kf->P[0][1] + kf->P[1][1] * dt;
    float p10 = kf->P[1][0] + kf->P[1][1] * dt;
    float p11 = kf->P[1][1] + kf->Q[1];

    float p22 = kf->P[2][2] + kf->P[3][2] * dt + (kf->P[2][3] + kf->P[3][3] * dt) * dt + kf->Q[2];
    float p23 = kf->P[2][3] + kf->P[3][3] * dt;
    float p32 = kf->P[3][2] + kf->P[3][3] * dt;
    float p33 = kf->P[3][3] + kf->Q[3];

    kf->P[0][0] = p00; kf->P[0][1] = p01;
    kf->P[1][0] = p10; kf->P[1][1] = p11;
    kf->P[2][2] = p22; kf->P[2][3] = p23;
    kf->P[3][2] = p32; kf->P[3][3] = p33;
}

/**
 * @brief 光流更新步 (低频调用，光流中断触发时)
 *        输入参数均为机体坐标系！
 * @param dx_flow 机体横向位移增量 (米，右为正)
 * @param dy_flow 机体纵向位移增量 (米，前为正)
 * @param dt_flow 距离上次光流触发的时间 (秒)
 * @param quality 表面质量 0~255 (来自 PMW3901)
 */
void FlowKalman_UpdateFlow(FlowKalman *kf, float dx_flow, float dy_flow, float dt_flow, uint8_t quality) {
    if (dt_flow <= 0.0f || dt_flow > 1.0f) return; // 时间异常不更新

    // 1. 将位移转换为机体坐标系的测量速度
    float vx_meas = dx_flow / dt_flow;
    float vy_meas = dy_flow / dt_flow;

    // 2. 根据光流质量动态调整测量噪声 R
    float quality_factor = 1.0f;
    if (quality < 100) {
        quality_factor = 5.0f;
    } else if (quality < 200) {
        quality_factor = 1.5f;
    }
    float R = kf->R_flow * quality_factor;

    // 3. 计算卡尔曼增益 K
    Matrix2x2 S;
    S.m[0][0] = kf->P[1][1] + R;
    S.m[0][1] = kf->P[1][3];
    S.m[1][0] = kf->P[3][1];
    S.m[1][1] = kf->P[3][3] + R;

    Matrix2x2 S_inv = S;
    Matrix2x2_Inverse(&S_inv);

    float K[4][2];
    K[0][0] = kf->P[0][1] * S_inv.m[0][0] + kf->P[0][3] * S_inv.m[1][0];
    K[0][1] = kf->P[0][1] * S_inv.m[0][1] + kf->P[0][3] * S_inv.m[1][1];

    K[1][0] = kf->P[1][1] * S_inv.m[0][0] + kf->P[1][3] * S_inv.m[1][0];
    K[1][1] = kf->P[1][1] * S_inv.m[0][1] + kf->P[1][3] * S_inv.m[1][1];

    K[2][0] = kf->P[2][1] * S_inv.m[0][0] + kf->P[2][3] * S_inv.m[1][0];
    K[2][1] = kf->P[2][1] * S_inv.m[0][1] + kf->P[2][3] * S_inv.m[1][1];

    K[3][0] = kf->P[3][1] * S_inv.m[0][0] + kf->P[3][3] * S_inv.m[1][0];
    K[3][1] = kf->P[3][1] * S_inv.m[0][1] + kf->P[3][3] * S_inv.m[1][1];

    // 4. 估计值更新 X = X + K * (Z - H*X)
    float err_x = vx_meas - kf->x[1]; // 机体横向速度误差
    float err_y = vy_meas - kf->x[3]; // 机体纵向速度误差

    kf->x[0] += K[0][0] * err_x + K[0][1] * err_y; // 修正 PosX
    kf->x[1] += K[1][0] * err_x + K[1][1] * err_y; // 修正 VelX
    kf->x[2] += K[2][0] * err_x + K[2][1] * err_y; // 修正 PosY
    kf->x[3] += K[3][0] * err_x + K[3][1] * err_y; // 修正 VelY

    // 5. 协方差更新 P = (I - K*H) * P
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            if (i == 0 || i == 1) {
                kf->P[i][j] -= K[i][0] * kf->P[1][j] + K[i][1] * kf->P[3][j];
            } else {
                kf->P[i][j] -= K[i][0] * kf->P[1][j] + K[i][1] * kf->P[3][j];
            }
        }
    }
}

// 获取当前机体坐标系下的位置和速度
void FlowKalman_GetState(FlowKalman *kf, float *pos_x, float *pos_y, float *vel_x, float *vel_y) {
    *pos_x = kf->x[0]; // 横向位置 (右正)
    *pos_y = kf->x[2]; // 纵向位置 (前正)
    *vel_x = kf->x[1]; // 横向速度 (右正)
    *vel_y = kf->x[3]; // 纵向速度 (前正)
}

/**
 * @brief GPS 更新步 (低频调用，如 1Hz~10Hz)
 *        输入参数必须已经从 ENU(东-北) 旋转为 机体坐标系(横-纵)！
 * @param gps_px 机体横向位置 (米，右为正)
 * @param gps_py 机体纵向位置 (米，前为正)
 * @param gps_vx 机体横向速度
 * @param gps_vy 机体纵向速度 (米/秒)
 * @param hAcc   GPS 水平定位精度 (单位：米)
 */
void FlowKalman_UpdateGPS(FlowKalman *kf, float gps_px, float gps_py, float gps_vx, float gps_vy, float hAcc) {

    // 1. 动态计算位置测量噪声 R_pos
    if (hAcc < 0.1f) hAcc = 0.1f;
    float R_pos = kf->R_gps_pos + (hAcc * hAcc);

    // ================= X 轴更新 (状态 px, vx) =================
    Matrix2x2 S_x, K_x;

    S_x.m[0][0] = kf->P[0][0] + R_pos;
    S_x.m[0][1] = kf->P[0][1];
    S_x.m[1][0] = kf->P[1][0];
    S_x.m[1][1] = kf->P[1][1] + kf->R_gps_vel;

    Matrix2x2 S_inv_x = S_x;
    Matrix2x2_Inverse(&S_inv_x);

    K_x.m[0][0] = kf->P[0][0]*S_inv_x.m[0][0] + kf->P[0][1]*S_inv_x.m[1][0];
    K_x.m[0][1] = kf->P[0][0]*S_inv_x.m[0][1] + kf->P[0][1]*S_inv_x.m[1][1];
    K_x.m[1][0] = kf->P[1][0]*S_inv_x.m[0][0] + kf->P[1][1]*S_inv_x.m[1][0];
    K_x.m[1][1] = kf->P[1][0]*S_inv_x.m[0][1] + kf->P[1][1]*S_inv_x.m[1][1];

    float err_px = gps_px - kf->x[0];
    float err_vx = gps_vx - kf->x[1];

    kf->x[0] += K_x.m[0][0] * err_px + K_x.m[0][1] * err_vx;
    kf->x[1] += K_x.m[1][0] * err_px + K_x.m[1][1] * err_vx;

    float p00 = kf->P[0][0] - (K_x.m[0][0]*kf->P[0][0] + K_x.m[0][1]*kf->P[1][0]);
    float p01 = kf->P[0][1] - (K_x.m[0][0]*kf->P[0][1] + K_x.m[0][1]*kf->P[1][1]);
    float p10 = kf->P[1][0] - (K_x.m[1][0]*kf->P[0][0] + K_x.m[1][1]*kf->P[1][0]);
    float p11 = kf->P[1][1] - (K_x.m[1][0]*kf->P[0][1] + K_x.m[1][1]*kf->P[1][1]);
    kf->P[0][0] = p00; kf->P[0][1] = p01;
    kf->P[1][0] = p10; kf->P[1][1] = p11;


    // ================= Y 轴更新 (状态 py, vy) =================
    Matrix2x2 S_y, K_y;

    S_y.m[0][0] = kf->P[2][2] + R_pos;
    S_y.m[0][1] = kf->P[2][3];
    S_y.m[1][0] = kf->P[3][2];
    S_y.m[1][1] = kf->P[3][3] + kf->R_gps_vel;

    Matrix2x2 S_inv_y = S_y;
    Matrix2x2_Inverse(&S_inv_y);

    K_y.m[0][0] = kf->P[2][2]*S_inv_y.m[0][0] + kf->P[2][3]*S_inv_y.m[1][0];
    K_y.m[0][1] = kf->P[2][2]*S_inv_y.m[0][1] + kf->P[2][3]*S_inv_y.m[1][1];
    K_y.m[1][0] = kf->P[3][2]*S_inv_y.m[0][0] + kf->P[3][3]*S_inv_y.m[1][0];
    K_y.m[1][1] = kf->P[3][2]*S_inv_y.m[0][1] + kf->P[3][3]*S_inv_y.m[1][1];

    float err_py = gps_py - kf->x[2];
    float err_vy = gps_vy - kf->x[3];

    kf->x[2] += K_y.m[0][0] * err_py + K_y.m[0][1] * err_vy;
    kf->x[3] += K_y.m[1][0] * err_py + K_y.m[1][1] * err_vy;

    float p22 = kf->P[2][2] - (K_y.m[0][0]*kf->P[2][2] + K_y.m[0][1]*kf->P[3][2]);
    float p23 = kf->P[2][3] - (K_y.m[0][0]*kf->P[2][3] + K_y.m[0][1]*kf->P[3][3]);
    float p32 = kf->P[3][2] - (K_y.m[1][0]*kf->P[2][2] + K_y.m[1][1]*kf->P[3][2]);
    float p33 = kf->P[3][3] - (K_y.m[1][0]*kf->P[2][3] + K_y.m[1][1]*kf->P[3][3]);
    kf->P[2][2] = p22; kf->P[2][3] = p23;
    kf->P[3][2] = p32; kf->P[3][3] = p33;
}
