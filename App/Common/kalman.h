/*
 * kalman.h
 *
 *  Created on: May 17, 2026
 *      Author: bitstream
 */

#ifndef COMMON_KALMAN_H_
#define COMMON_KALMAN_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>

#define STATE_DIM 4    // 状态维度
#define MEAS_DIM 2     // 观测维度
#define CTRL_DIM  2   // 控制输入维度 (ax, ay)

class Kalman
{
public:
	// ---------- 卡尔曼滤波器接口 ----------
	// 初始化：设置过程噪声 Q 和测量噪声 R（通常固定）
	// q_pos: 位置过程噪声方差 (m²/s^? 实际是每步的随机游走强度)
	// q_vel: 速度过程噪声方差 (m²/s³)
	// r_pos: GPS 测量噪声方差 (m²)
	/*
	q_pos：位置的过程噪声方差（单位：m²/步？ 实际上它是离散时间模型中的每步方差）。
	它表示了在预测步骤中，模型对位置估计的“不信任程度”。数值越大，滤波器越倾向于相信新的测量值，轨迹会抖动；数值越小，轨迹越平滑，但对真实变化响应变慢。

	q_vel：速度的过程噪声方差（单位：m²/s² 每步）。
	它描述了速度预测的不确定性。因为速度的变化往往由未建模的加速度（风、转弯、颠簸）引起。q_vel 越大，滤波器会更快调整速度估计。

	r_pos：测量噪声方差（单位：m²）。
	它反映了位置传感器（如GPS）的精度。通常由传感器数据手册给出，或通过静态测量统计得到。r_pos 越小，滤波器越相信传感器的读数。

	卡尔曼滤波对 Q 和 R 的相对比例敏感，而不是绝对值。也就是说，如果你同时把 q_pos, q_vel, r_pos 都乘以 10，
	效果基本等价。所以可以保持 r_pos 固定（由传感器决定），然后调节 q_pos 和 q_vel 来达到平衡。

	(1) r_pos —— 测量噪声方差
	最直接，最容易获得。

	从传感器数据手册：例如GPS模块的规格书通常写“水平定位精度 < 2.5m CEP”（即圆概率误差，约67%的误差小于2.5m）。我们可以近似将标准差设为 σ ≈ 2.5m，则方差 r_pos = σ² ≈ 6.25。
	对于更差的模块，可能5m → 25；差分GPS可能0.1m → 0.01。

	静态测试：将传感器静止放置，采集大量位置数据（如一分钟），计算样本方差：

	text
	r_pos = (1/(N-1)) * Σ (xi - 均值)²
	由于静止时真实位置不变，该方差即为测量噪声方差。

	在线估计（高级）：通过残差序列的协方差自适应估算，但一般不需要。

	结论：r_pos 基本由传感器硬件决定，可以直接从指标或测试得到。

	(2) q_vel —— 速度过程噪声方差
	速度的变化是由于未建模的加速度（包括真实机动、风阻、路面倾斜等）。在匀速模型中，我们假设速度不变，但实际上速度会变化。q_vel 描述了这些随机加速度在每步产生的速度不确定性。

	理论估计：

	设最大可能的随机加速度为 a_max（单位 m/s²）。在一个时间步 dt 内，速度的变化量 Δv 可达 a_max * dt。如果认为加速度近似均匀分布，其方差 ≈ (a_max * dt)² / 12。
	但更常用的是采用连续时间随机游走模型：

	定义加速度功率谱密度 S_a（单位 m²/s³）。对于离散时间，q_vel = S_a * dt。

	S_a 的典型值（经验）：

	行人（缓慢运动，加速度 < 1 m/s²）：S_a = 0.1 ~ 0.5

	车辆（城市道路，加速度 < 5 m/s²）：S_a = 2.0 ~ 10.0

	无人机 / 赛车（加速度 < 20 m/s²）：S_a = 20 ~ 100

	如果你没有谱密度概念，可以直接根据预期速度变化率来估计：

	text
	q_vel ≈ (max_possible_speed_change_per_second * dt)² / 2
	例如，车辆最快可在 0.1s 内改变速度 0.5 m/s，则 q_vel ≈ 0.5² / 2 = 0.125（这是一个粗略估计）。

	工程试凑法（推荐新手）：

	先设一个较小值，如 0.01。

	运行滤波器，观察估计速度是否平滑且能跟上真实运动。如果速度响应太迟钝（滞后严重），增大 q_vel（比如 0.1、1.0）；如果速度抖动厉害，减小 q_vel。

	(3) q_pos —— 位置过程噪声方差
	位置的不确定性主要来源于速度预测误差的积分，以及未建模加速度对位置的直接影响。

	理论推导：

	如果随机加速度的功率谱密度为 S_a，那么位置的过程噪声方差（离散化后）为：

	text
	q_pos = (1/3) * S_a * dt³
	这个公式来自对加速度噪声的二重积分。当没有更多信息时，可以取 q_pos = q_vel * dt² / 3（因为 q_vel = S_a * dt）。

	例如：

	dt = 0.1s，q_vel = 0.1 → q_pos ≈ 0.1 * 0.01 / 3 = 0.00033。

	经验法则：q_pos 通常比 q_vel 小几个数量级，因为位置通过积分产生，随机扰动对位置的直接作用远小于速度的随机游走。

	工程调参：

	如果滤波轨迹过于平滑，跟不上急转弯，增加 q_pos（让位置也能有随机游走）。

	如果位置抖动太大，减小 q_pos。但通常调整 q_vel 效果更明显。

	常见的调参流程：
	先标定 r_pos：通过静态测试得到 GPS 噪声方差。
	设定初始 q_vel：根据运动类型，取一个经验值（如车辆用 1.0，行人用 0.1）。
	设定 q_pos = q_vel * dt² / 3 作为初始值。
	运行实验，观察估计误差：
	如果估计值抖动大（高频噪声），说明 Q 相对于 R 太大 → 减小 Q。
	如果估计值平滑但跟不上突然加速/减速（滞后），说明 Q 太小 → 增大 Q。
	微调：必要时单独调整 q_vel 或 q_pos 的比例。

	参数		依赖信息					推荐方法
	r_pos	传感器精度					手册标称值 → 静态测试 → 更精确
	q_vel	预期最大加速度、机动剧烈程度		理论：S_a * dt；调参：从 0.01 开始试，直到响应匹配
	q_pos	同上，以及 dt				理论：q_vel * dt² / 3；调参：通常保持该比例，或按需单独微调

	运动场景	S_a (m²/s³)	q_vel = S_a*dt	q_pos = (1/3)S_adt³	r_pos (GPS，典型值)
	行人慢走	0.2			0.02			0.000067			1~4（手机GPS）
	车辆城市	5.0			0.5				0.0017				4~9（普通GPS）
	车辆高速	20.0		2.0				0.0067				4~9
	无人机	50.0		5.0				0.0167				0.01~1（差分GPS）

	参数问题		直观表现
	r_pos 太小	轨迹毛刺多，噪声大
	r_pos 太大	响应慢，滞后明显
	q_vel 太小	速度跟不上变化，位置漂移
	q_vel 太大	速度抖动，位置抖动
	q_pos 太大	位置突变，速度不合理
	*/
	void init(float q_pos, float q_vel, float r_pos);

	// 设置初始状态（通常在第一次 GPS 测量后调用）
	void set_state(float x, float y, float vx, float vy, float timestamp);

	// 核心更新函数
	// timestamp: 当前时刻 (秒)
	// gps_meas: [x, y]  GPS 测量值 (如果无效可传入 NULL)
	// imu_acc:  [ax, ay] IMU 加速度 (如果无效可传入 NULL)
	// 返回值: 0 成功, -1 未初始化
	int update(float timestamp, float gps_meas[2], float imu_acc[2]);

	//获取x坐标
	float get_x(){return _kf.x[0];}
	//获取y坐标
	float get_y(){return _kf.x[1];}
	//获取x轴速度
	float get_vx(){return _kf.x[2];}
	//获取y轴速度
	float get_vy(){return _kf.x[3];}

private:
	typedef struct {
	    float x[STATE_DIM];          // 状态向量
	    float P[STATE_DIM][STATE_DIM]; // 协方差矩阵
	    float Q[STATE_DIM][STATE_DIM]; // 过程噪声协方差
	    float R[MEAS_DIM][MEAS_DIM];   // 测量噪声协方差 (GPS)
	    float last_timestamp;          // 上次更新时间戳 (秒)
	    int    initialized;             // 是否已初始化状态
	} stKalmanFilter;

	stKalmanFilter _kf;
};

#if 0
// ---------- 使用示例 ----------
int main() {
    KalmanFilter kf;

    // 初始化: 设置过程噪声和 GPS 噪声
    // q_pos = 0.01 (每步位置随机游走方差), q_vel = 0.01, r_pos = 1.0 (GPS 方差 1 m²)
    kalman_init(&kf, 0.01, 0.01, 1.0);

    // 模拟第一次 GPS 测量 (时间 0.0 秒，位置 (0,0)，速度未知设为 0)
    float first_gps[2] = {0.0, 0.0};
    kalman_set_state(&kf, first_gps[0], first_gps[1], 0.0, 0.0, 0.0);

    // 模拟后续数据: 假设 IMU 采样率 50Hz, GPS 采样率 5Hz
    float imu_data[][2] = {
        {0.1, 0.0}, {0.1, 0.0}, {0.1, 0.0}, {0.1, 0.0}, {0.1, 0.0},  // 0.1s 内 ax=0.1, ay=0
        {0.1, 0.0}, {0.1, 0.0}, {0.1, 0.0}, {0.1, 0.0}, {0.1, 0.0}
    };
    float gps_meas[][2] = {
        {0.2, 0.1},   // t=0.2s 的 GPS
        {0.8, 0.3},   // t=0.4s
        {1.5, 0.6},   // t=0.6s
        {2.3, 0.9}    // t=0.8s
    };

    float timestamp = 0.0;
    int gps_idx = 0;

    // 模拟 0 ~ 1.0 秒，步长 0.02s
    for (int step = 1; step <= 50; step++) {
        timestamp = step * 0.02;  // 20ms
        int imu_idx = (step - 1) % 10;
        float *imu = imu_data[imu_idx];

        float *gps = NULL;
        if (gps_idx < sizeof(gps_meas)/sizeof(gps_meas[0]) &&
            fabs(timestamp - (gps_idx+1)*0.2) < 0.01) {
            gps = gps_meas[gps_idx];
            gps_idx++;
        }

        kalman_update(&kf, timestamp, gps, imu);

        printf("t=%.2f: x=%.3f, y=%.3f, vx=%.3f, vy=%.3f\n",
               timestamp, kf.x[0], kf.x[1], kf.x[2], kf.x[3]);
    }

    return 0;
}
#endif

#endif /* COMMON_KALMAN_H_ */
