/*
 * flow_kalman.h
 *
 *  Created on: 2026年8月14日
 *      Author: bitstream
 */

#ifndef INC_FLOW_KALMAN_H_
#define INC_FLOW_KALMAN_H_

#include <math.h>
#include <stdint.h>

/*
核心调参指南
代码中主要有两个调参点，决定了滤波器的表现：

Q (过程噪声)
增大 Q：表示你不相信加速度计的数据精度，滤波器会更倾向于相信光流。位置变化会更灵敏，但光流的噪声会直接体现到位置上。
减小 Q：表示极度相信加速度计。位置曲线会非常平滑，但光流的修正作用会变慢。
R_flow (测量噪声)
基础值为 0.1。如果发现位置抖动很大，把它调大（比如 0.5），滤波器会对光流数据做更强的平滑。
surface_quality 动态权重
代码里我写了一段逻辑：当质量低于 100 时，R 乘以 5 倍。这意味着：地面纹理不清晰时，光流数据几乎被忽略，完全靠加速度计维持位置。这是防飘移的杀手锏。
 */


// 定义 2x2 矩阵求逆辅助函数
typedef struct {
    float m[2][2];
} Matrix2x2;

// 4状态卡尔曼滤波器状态 (PosX, VelX, PosY, VelY)
typedef struct {
    float x[4];       // 状态向量 [px, vx, py, vy] (米, 米/秒)
    float P[4][4];    // 协方差矩阵
    float Q[4];       // 过程噪声 (加速度计噪声)
    float R_flow;     // 测量噪声基础值 (光流噪声)
    // 新增：GPS 测量噪声
    float R_gps_pos;  // GPS 位置噪声 (通常 1.0 ~ 5.0 米)
    float R_gps_vel;  // GPS 速度噪声 (通常 0.1 ~ 0.5 米/秒)
} FlowKalman;


// 初始化卡尔曼滤波器
void FlowKalman_Init(FlowKalman *kf);
// 预测步：使用 IMU 加速度推算 (高频调用，如 1000Hz)
// ax, ay: 机身水平加速度 (需扣除重力，且旋转到世界坐标系下)
// dt: 距离上次预测的时间(秒)
void FlowKalman_Predict(FlowKalman *kf, float ax, float ay, float yaw_rate, float dt);
// 更新步：使用光流数据修正 (低频调用，光流中断触发时)
// dx_flow, dy_flow: 光流测量的位移增量 (米，已做倾斜补偿)
// dt_flow: 距离上次光流触发的时间 (秒)
// quality: 表面质量 0~255 (来自 PMW3901)
void FlowKalman_UpdateFlow(FlowKalman *kf, float dx_flow, float dy_flow, float dt_flow, uint8_t quality);
// 获取当前位置和速度
void FlowKalman_GetState(FlowKalman *kf, float *pos_x, float *pos_y, float *vel_x, float *vel_y);
/**
 * @brief GPS 更新步 (低频调用，如 1Hz~10Hz)
 * @param gps_px GPS 全局 X 位置 (米)
 * @param gps_py GPS 全局 Y 位置 (米)
 * @param gps_vx GPS X 方向速度
 * @param gps_vy GPS Y 方向速度 (米/秒)
 * @param hAcc   GPS 水平定位精度 (单位：米，来自 GNSS_Handle.hAcc / 1000.0f)
 */
void FlowKalman_UpdateGPS(FlowKalman *kf, float gps_px, float gps_py, float gps_vx, float gps_vy, float hAcc);

#endif /* INC_FLOW_KALMAN_H_ */
