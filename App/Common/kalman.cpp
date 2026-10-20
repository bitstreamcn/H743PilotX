/*
 * kalman.cpp
 *
 *  Created on: May 17, 2026
 *      Author: bitstream
 */
#include "kalman.h"

// ---------- 矩阵运算 (4x4, 4x2, 2x2 等) ----------
void mat_mul_4x4(float A[4][4], float B[4][4], float C[4][4]) {
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++) {
            C[i][j] = 0.0;
            for (int k = 0; k < 4; k++)
                C[i][j] += A[i][k] * B[k][j];
        }
}

void mat_mul_4x2(float A[4][4], float B[4][2], float C[4][2]) {
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 2; j++) {
            C[i][j] = 0.0;
            for (int k = 0; k < 4; k++)
                C[i][j] += A[i][k] * B[k][j];
        }
}

void mat_mul_2x4(float A[2][4], float B[4][4], float C[2][4]) {
    for (int i = 0; i < 2; i++)
        for (int j = 0; j < 4; j++) {
            C[i][j] = 0.0;
            for (int k = 0; k < 4; k++)
                C[i][j] += A[i][k] * B[k][j];
        }
}

void mat_mul_2x2(float A[2][2], float B[2][2], float C[2][2]) {
    for (int i = 0; i < 2; i++)
        for (int j = 0; j < 2; j++) {
            C[i][j] = 0.0;
            for (int k = 0; k < 2; k++)
                C[i][j] += A[i][k] * B[k][j];
        }
}

// 2x2 矩阵加法: C = A + B
void mat_add_2x2(float A[2][2], float B[2][2], float C[2][2]) {
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
            C[i][j] = A[i][j] + B[i][j];
        }
    }
}

void mat_add_4x4(float A[4][4], float B[4][4], float C[4][4]) {
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            C[i][j] = A[i][j] + B[i][j];
}

void mat_sub_4x4(float A[4][4], float B[4][4], float C[4][4]) {
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            C[i][j] = A[i][j] - B[i][j];
}

void mat_scale_4x4(float scalar, float A[4][4], float B[4][4]) {
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            B[i][j] = scalar * A[i][j];
}

void mat_trans_4x4(float A[4][4], float B[4][4]) {
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            B[j][i] = A[i][j];
}

// 2x2 矩阵求逆 (假设可逆)
int mat_inv_2x2(float A[2][2], float inv[2][2]) {
    float det = A[0][0] * A[1][1] - A[0][1] * A[1][0];
    if (fabs(det) < 1e-12) return -1;
    inv[0][0] =  A[1][1] / det;
    inv[0][1] = -A[0][1] / det;
    inv[1][0] = -A[1][0] / det;
    inv[1][1] =  A[0][0] / det;
    return 0;
}

void mat_identity_4x4(float I[4][4]) {
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            I[i][j] = (i == j) ? 1.0 : 0.0;
}

// ---------- 卡尔曼滤波器接口 ----------
// 初始化：设置过程噪声 Q 和测量噪声 R（通常固定）
// q_pos: 位置过程噪声方差 (m²/s^? 实际是每步的随机游走强度)
// q_vel: 速度过程噪声方差 (m²/s³)
// r_pos: GPS 测量噪声方差 (m²)
void Kalman::init(float q_pos, float q_vel, float r_pos) {
	stKalmanFilter *kf = &_kf;
    memset(kf, 0, sizeof(stKalmanFilter));

    // 过程噪声 Q (对角阵)
    memset(kf->Q, 0, sizeof(kf->Q));
    kf->Q[0][0] = q_pos;   // x 位置
    kf->Q[1][1] = q_pos;   // y 位置
    kf->Q[2][2] = q_vel;   // vx
    kf->Q[3][3] = q_vel;   // vy

    // 测量噪声 R (GPS 位置)
    memset(kf->R, 0, sizeof(kf->R));
    kf->R[0][0] = r_pos;
    kf->R[1][1] = r_pos;

    // 初始协方差 P (设大值表示初始不确定)
    for (int i = 0; i < STATE_DIM; i++)
        for (int j = 0; j < STATE_DIM; j++)
            kf->P[i][j] = (i == j) ? 100.0 : 0.0;

    kf->initialized = 0;
    kf->last_timestamp = 0.0;
}

// 设置初始状态（通常在第一次 GPS 测量后调用）
void Kalman::set_state(float x, float y, float vx, float vy, float timestamp) {
	stKalmanFilter *kf = &_kf;
    kf->x[0] = x;
    kf->x[1] = y;
    kf->x[2] = vx;
    kf->x[3] = vy;
    kf->last_timestamp = timestamp;
    kf->initialized = 1;
}

// 核心更新函数
// timestamp: 当前时刻 (秒)
// gps_meas: [x, y]  GPS 测量值 (如果无效可传入 NULL)
// imu_acc:  [ax, ay] IMU 加速度 (如果无效可传入 NULL)
// 返回值: 0 成功, -1 未初始化
int Kalman::update(float timestamp, float gps_meas[2], float imu_acc[2]) {
	stKalmanFilter *kf = &_kf;
    if (!kf->initialized) return -1;

    float dt = timestamp - kf->last_timestamp;
    if (dt <= 0.0) {
        // 如果时间倒退或不变，不更新预测，但可能仍然进行观测更新
        // 这里简单处理：跳过预测，只做更新
        dt = 0.0;
    }

    // ========== 预测步骤 ==========
    if (dt > 0.0 && imu_acc != NULL) {
        // 动态构建 F 和 G 矩阵
        float F[4][4] = {
            {1, 0, dt, 0},
            {0, 1, 0, dt},
            {0, 0, 1, 0},
            {0, 0, 0, 1}
        };
        float G[4][2] = {
            {(float)(0.5 * dt * dt), 0},
            {0, (float)(0.5 * dt * dt)},
            {dt, 0},
            {0, dt}
        };

        // 控制向量 u = [ax, ay]
        float u[2] = {imu_acc[0], imu_acc[1]};

        // 1. 状态预测: x_pred = F * x + G * u
        float x_pred[STATE_DIM];
        for (int i = 0; i < STATE_DIM; i++) {
            x_pred[i] = 0.0;
            for (int j = 0; j < STATE_DIM; j++)
                x_pred[i] += F[i][j] * kf->x[j];
            for (int j = 0; j < CTRL_DIM; j++)
                x_pred[i] += G[i][j] * u[j];
        }

        // 2. 协方差预测: P_pred = F * P * F^T + Q
        float F_P[4][4];
        mat_mul_4x4(F, kf->P, F_P);           // F * P
        float F_P_Ft[4][4];
        float Ft[4][4];
        mat_trans_4x4(F, Ft);
        mat_mul_4x4(F_P, Ft, F_P_Ft);         // F*P*F^T

        float P_pred[4][4];
        mat_add_4x4(F_P_Ft, kf->Q, P_pred);

        // 更新状态和协方差
        memcpy(kf->x, x_pred, sizeof(kf->x));
        memcpy(kf->P, P_pred, sizeof(kf->P));
    }

    // ========== 更新步骤 (如果有 GPS 测量) ==========
    if (gps_meas != NULL) {
        // 观测矩阵 H (固定)
        float H[2][4] = {
            {1, 0, 0, 0},
            {0, 1, 0, 0}
        };

        // 测量余量 y = z - H*x
        float y[2];
        for (int i = 0; i < MEAS_DIM; i++) {
            float hx = 0.0;
            for (int j = 0; j < STATE_DIM; j++)
                hx += H[i][j] * kf->x[j];
            y[i] = gps_meas[i] - hx;
        }

        // 余量协方差 S = H * P * H^T + R
        float H_P[2][4];
        mat_mul_2x4(H, kf->P, H_P);           // H * P
        float H_P_Ht[2][2];
        float Ht[4][2];
        // H 转置: Ht[j][i] = H[i][j]
        for (int i = 0; i < 2; i++)
            for (int j = 0; j < 4; j++)
                Ht[j][i] = H[i][j];
        // 计算 H_P * Ht
        for (int i = 0; i < 2; i++)
            for (int j = 0; j < 2; j++) {
                H_P_Ht[i][j] = 0.0;
                for (int k = 0; k < 4; k++)
                    H_P_Ht[i][j] += H_P[i][k] * Ht[k][j];
            }
        float S[2][2];
        mat_add_2x2(H_P_Ht, kf->R, S);

        // 卡尔曼增益 K = P * H^T * inv(S)
        float P_Ht[4][2];
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 2; j++) {
                P_Ht[i][j] = 0.0;
                for (int k = 0; k < 4; k++)
                    P_Ht[i][j] += kf->P[i][k] * Ht[k][j];
            }
        float S_inv[2][2];
        if (mat_inv_2x2(S, S_inv) != 0) {
            // 奇异矩阵，跳过更新
            kf->last_timestamp = timestamp;
            return -1;
        }
        float K[4][2];
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 2; j++) {
                K[i][j] = 0.0;
                for (int k = 0; k < 2; k++)
                    K[i][j] += P_Ht[i][k] * S_inv[k][j];
            }

        // 状态更新: x = x + K * y
        float Ky[4];
        for (int i = 0; i < 4; i++) {
            Ky[i] = 0.0;
            for (int j = 0; j < 2; j++)
                Ky[i] += K[i][j] * y[j];
            kf->x[i] += Ky[i];
        }

        // 协方差更新: P = (I - K*H) * P
        float KH[4][4];
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++) {
                KH[i][j] = 0.0;
                for (int k = 0; k < 2; k++)
                    KH[i][j] += K[i][k] * H[k][j];
            }
        float I[4][4];
        mat_identity_4x4(I);
        float I_minus_KH[4][4];
        mat_sub_4x4(I, KH, I_minus_KH);
        float P_new[4][4];
        mat_mul_4x4(I_minus_KH, kf->P, P_new);
        memcpy(kf->P, P_new, sizeof(kf->P));
    }

    kf->last_timestamp = timestamp;
    return 0;
}
