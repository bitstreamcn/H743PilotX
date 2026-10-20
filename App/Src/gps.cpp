/*
 * gps.cpp
 *
 *  Created on: 2026年8月14日
 *      Author: bitstream
 */
#include "main.h"
#include "gps.h"
#include <math.h>
#include <stdbool.h>
#include "spi.h"
#include "log.h"
#include <cstring>
#include "common.h"
#include <cstdio>
#include "usart.h"
#include "application.h"
#include "common.h"
#include "Config.h"


#ifdef __cplusplus
extern "C" {
#endif

#include "../Device/Imu/imu.h"


#ifdef __cplusplus
}
#endif

st_gps_data gps_data;

float hdistance = 0; //家的距离
float distance = 0;  //目标位置的距离

#define EARTH_RADIUS_M 6378137.0f           // 地球赤道半径（米）
#define DEG_TO_RAD      0.017453292519943295f // 角度转弧度 (PI / 180)
#define PI      3.141592653589793238f // 角度转弧度 (PI)

// 记录起飞点坐标
static float home_lat = 0.0f;
static float home_lon = 0.0f;
static float home_lat_rad = 0.0f;
static float home_lon_rad = 0.0f;
static bool home_is_set = false;

float target_lat = 0.0f;
float target_lon = 0.0f;

static bool uart_error = false;
static bool uart_start = false;

// 定义数据帧的完整长度：3字节帧头(AA 55 Len) + 结构体数据
#define GPS_FRAME_TOTAL_LEN (3 + sizeof(st_gps_data))

volatile bool uart2_data_recv = false;
volatile uint16_t uart2_data_size = 0;

uint8_t rx_buffer[sizeof(st_gps_data) + 50];

uint8_t pack_buffer[sizeof(st_gps_data) * 3 + 20]; // 适当扩大，防止溢出
uint8_t pack_buffer_index = 0;

uint32_t gps_update_tick = 0;

float mag_declination = 0;

void on_gps_received(void);
void on_mag_data(void);

/**
 * @brief 根据经纬度近似计算磁偏角
 * @param lat 纬度 (度)
 * @param lon 经度 (度)
 * @return 磁偏角 (度)，磁北偏东为正，偏西为负
 */
float Calculate_Magnetic_Declination(float lat, float lon)
{
    // 2025年北磁极近似坐标 (纬度, 经度)
    // 注意：磁极每年都在缓慢移动，这里取近似值
    float mag_north_lat = 86.0f;
    float mag_north_lon = -160.0f;

    // 转换为弧度
    float lat_rad = lat * DEG_TO_RAD;
    float lon_rad = lon * DEG_TO_RAD;
    float mag_lat_rad = mag_north_lat * DEG_TO_RAD;
    float mag_lon_rad = mag_north_lon * DEG_TO_RAD;

    // 计算经度差
    float d_lon = lon_rad - mag_lon_rad;

    // 使用球面三角学公式计算磁偏角
    // 公式: tan(Declination) = sin(d_lon) / (cos(lat)*tan(mag_lat) - sin(lat)*cos(d_lon))
    float numerator = sinf(d_lon);
    float denominator = cosf(lat_rad) * tanf(mag_lat_rad) - sinf(lat_rad) * cosf(d_lon);

    // 使用 atan2 防止除零和象限错误
    float declination_rad = atan2f(numerator, denominator);
#define RAD_TO_DEG 57.29577951308232f
    // 转为角度
    float declination = declination_rad * RAD_TO_DEG;

	// 针对中国中东部地区的经验补偿 (补偿东亚磁异常)
	// 这不是物理公式，而是工程上的近似修正
	if (lat > 20.0f && lat < 50.0f && lon > 100.0f && lon < 130.0f) {
		declination -= 1.5f;
	}
	return declination;
}

/**
 * @brief 设置起飞原点 (Home Point)
 *        通常在无人机解锁起飞时调用一次，记录当前 GPS 为原点 (0,0)
 */
void set_home_location(float home_lat_deg, float home_lon_deg)
{

	mag_declination = Calculate_Magnetic_Declination(gps_data.fLat, gps_data.fLon);

	home_lat = home_lat_deg;
	home_lon = home_lon_deg;
    home_lat_rad = home_lat_deg * DEG_TO_RAD;
    home_lon_rad = home_lon_deg * DEG_TO_RAD;
    home_is_set = true;
}

bool poll_read_gps()
{
	bool ret = false;
	if (!uart_start)
	{
		HAL_StatusTypeDef ret = HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_buffer, sizeof(rx_buffer));
		if (ret != HAL_OK)
		{
			uart_error = true;
		}
		uart_start = true;
	}

	if (uart_error)
	{
		HAL_StatusTypeDef ret = HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_buffer, sizeof(rx_buffer));
		if (ret == HAL_OK)
		{
			uart_error = false;
		}
	}

	if (uart2_data_recv)
	{
		uart2_data_recv = false;
        // 1. 将新接收到的数据追加到 pack_buffer 中
        if (uart2_data_size > 0 && (pack_buffer_index + uart2_data_size) <= sizeof(pack_buffer))
        {
            memcpy(&pack_buffer[pack_buffer_index], rx_buffer, uart2_data_size);
            pack_buffer_index += uart2_data_size;
        }
        else
        {
            // 缓冲区溢出或异常，清空重来
            pack_buffer_index = 0;
            memcpy(pack_buffer, rx_buffer, uart2_data_size);
            pack_buffer_index = uart2_data_size;
        }

        // 2. 在 pack_buffer 中循环解析数据包（处理粘包）
        uint16_t parse_index = 0;
        while ((pack_buffer_index - parse_index) >= GPS_FRAME_TOTAL_LEN)
        {
            // 寻找帧头 0xAA 0x55
            if (pack_buffer[parse_index] == 0xAA && pack_buffer[parse_index + 1] == 0x55)
            {
            	uint8_t expected_len = pack_buffer[parse_index + 2];
				if (expected_len != sizeof(st_gps_data)) {
					// 长度不匹配，说明主从结构体定义不一致，或者这是假帧头
					parse_index += 1;
					continue; // 跳过这个字节，继续找下一个
				}
				// 🚨 新增：防腰斩检查
				// 在这个包的有效数据区内，如果提前出现了下一个包头，说明这个包是坏的
				bool is_corrupted = false;
				for (int i = 3; i < GPS_FRAME_TOTAL_LEN - 1; i++) {
					if (pack_buffer[parse_index + i] == 0xAA && pack_buffer[parse_index + i + 1] == 0x55) {
						is_corrupted = true;
						break;
					}
				}
				if (is_corrupted) {
					// 这个包是被腰斩的假包，直接丢弃，指针+1继续找下一个
					parse_index += 1;
					continue;
				}
                st_gps_data recv_data;
                // 提取数据体
                memcpy((uint8_t *)&recv_data, &pack_buffer[parse_index + 3], sizeof(st_gps_data));

                // 校验
                uint8_t checksum = 0;
                for (int i = 0; i < (sizeof(recv_data) - 1); i++)
                {
                    checksum += ((uint8_t *)&recv_data)[i];
                }

                if (checksum == recv_data.checksum)
                {
                	//CALC_FUNC_FREQ("gps");

                    // 校验成功
                    static unsigned long prev_iTOW = 0;
                    if (recv_data.iTOW != prev_iTOW)
                    {
                        prev_iTOW = recv_data.iTOW;
                        memcpy((uint8_t*)&gps_data, (uint8_t*)&recv_data, sizeof(st_gps_data));
                        if (gps_data.fixType >= 3 && gps_data.numSV >= 8)
                        {
                        	on_gps_received();
                        }
                        else
                        {
                        	//PERIODIC_LOGI(5000, "gps fixType:%d, numSV:%d", gps_data.fixType, gps_data.numSV);
                        }
                        //10颗星设置家
                        if (!home_is_set && gps_data.fixType >= 3 && gps_data.numSV >= 10 && gps_data.hAcc < 1000)
                        {
                        	home_is_set = true;
                        	set_home_location(gps_data.fLat, gps_data.fLon);
                        	gps_hold();
                        	Log::I("Home point set!\n");
                        }
                        gps_update_tick = HAL_GetTick();
                    }
                    on_mag_data();
                    // 解析成功，指针向后移动一个完整的包长
                    parse_index += GPS_FRAME_TOTAL_LEN;
                    ret = true;
                }
                else
                {
                    // 校验失败，说明这里是假帧头或数据错乱。指针向后移1个字节，继续找下一个AA 55
                    Log::E("poll_read_gps checksum error:%x - %x", checksum, recv_data.checksum);
                    parse_index += 1;
                }
            }
            else
            {
                // 不是帧头，指针向后移1个字节
                parse_index += 1;
            }
        }

        // 3. 处理剩余的半包数据
        uint16_t remain_len = pack_buffer_index - parse_index;
        if (remain_len > 0 && parse_index > 0)
        {
            // 将剩余的数据移到缓冲区开头，等待下一次接收补全
            memmove(pack_buffer, &pack_buffer[parse_index], remain_len);
        }
        pack_buffer_index = remain_len;

		HAL_StatusTypeDef ret = HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_buffer, sizeof(rx_buffer));
		if (ret != HAL_OK)
		{
			uart_error = true;
		}
	}
	if ((HAL_GetTick() - gps_update_tick) > 2000)
	{
		PERIODIC_LOGI(2000, "gps lost");
	}
	return ret;
}

void gps_hold()
{
	target_lat = gps_data.fLat;
	target_lon = gps_data.fLon;
}

void target_to_home()
{
	if (home_is_set)
	{
		target_lat = home_lat;
		target_lon = home_lon;
	}
}


/**
 * @brief 将当前经纬度转为相对起飞点的局部坐标 (米)
 *
 * @param lat_deg  当前纬度 (度)
 * @param lon_deg  当前经度 (度)
 * @param pos_x    输出: 东向位置 (米)，向东为正
 * @param pos_y    输出: 北向位置 (米)，向北为正
 */
void convert_gps_to_local(float lat_deg, float lon_deg, float *pos_x, float *pos_y)
{
    if (!home_is_set) {
        *pos_x = 0.0f;
        *pos_y = 0.0f;
        return;
    }

    float current_lat_rad = lat_deg * DEG_TO_RAD;
    float current_lon_rad = lon_deg * DEG_TO_RAD;

    // 纬度差转换为北向位移
    // 1度纬度大约恒等于 111公里 (2 * PI * R / 360)
    *pos_y = (current_lat_rad - home_lat_rad) * EARTH_RADIUS_M;

    // 经度差转换为东向位移
    // 1度经度的长度随纬度变化，需要乘以 cos(纬度)
    float cos_lat = cosf(home_lat_rad);
    *pos_x = (current_lon_rad - home_lon_rad) * EARTH_RADIUS_M * cos_lat;
}


// 计算 GPS 运动航向 (0为北，90为东，顺时针为正)，单位米/秒
float get_gps_heading(float vel_e, float vel_n) {
    // 如果速度太低，算出来的角度全是噪声，不可信
    float speed = sqrtf(vel_e * vel_e + vel_n * vel_n);
    if (speed < 1.5f) { // 阈值 1.5 m/s
        return -1.0f; // 返回 -1 表示无效
    }

    // atan2f(东向速度, 北向速度) 得到弧度
    float heading_rad = atan2f(vel_e, vel_n);
    float heading_deg = heading_rad * 180.0f / PI;

    if (heading_deg < 0.0f) heading_deg += 360.0f;
    return heading_deg;
}

void get_target_distance(float *x, float *y)
{
	float target_x_east, target_y_north;
	convert_gps_to_local(target_lat, target_lon, &target_x_east, &target_y_north);
	// ================= 坐标系转换 ================
	// 将 ENU(东,北) 转换为 机体坐标系(X: 右为正, Y: 前为正)
	float yaw_rad = yaw * DEG_TO_RAD;
	float cos_yaw = cosf(yaw_rad);
	float sin_yaw = sinf(yaw_rad);
	// 4. 获取滤波后的纯净绝对位置 (由卡尔曼输出)
	float fused_x_body, fused_y_body, fused_vx, fused_vy;
	app.Flow_GetState(&fused_x_body, &fused_y_body, &fused_vx, &fused_vy);

	// 5. 计算目标误差 (使用滤波后的位置去算误差，喂给位置环 PID)
	float target_x_body = target_x_east * cos_yaw - target_y_north * sin_yaw;
	float target_y_body = target_y_north * cos_yaw + target_x_east * sin_yaw;

	float dx_body = target_x_body - fused_x_body; // 横向距离误差
	float dy_body = target_y_body - fused_y_body; // 纵向距离误差
	*x = -dx_body;
	*y = -dy_body;
}

void on_mag_data(void)
{
	float off_x, off_y, off_z;
	float sca_x, sca_y, sca_z;

	off_x = cfg.param.mag_cal_x;
	off_y = cfg.param.mag_cal_y;
	off_z = cfg.param.mag_cal_z;
	sca_x = cfg.param.mag_cal_sx;
	sca_y = cfg.param.mag_cal_sy;
	sca_z = cfg.param.mag_cal_sz;

	// 读取真实数据
	float mx_raw = gps_data.mx;
	float my_raw = gps_data.my;
	float mz_raw = gps_data.mz;

	// 应用椭球拟合校准
	float mx_calibrated = (mx_raw - off_x) * sca_x;
	float my_calibrated = -(my_raw - off_y) * sca_y;
	float mz_calibrated = -(mz_raw - off_z) * sca_z;


    // 3. 倾斜补偿 (使用标准的从机体坐标系到地理水平坐标系的旋转矩阵)
    float pitch_rad = pitch * DEG_TO_RAD;
    float roll_rad  = roll * DEG_TO_RAD;

    float cos_pitch = cosf(pitch_rad);
    float sin_pitch = sinf(pitch_rad);
    float cos_roll  = cosf(roll_rad);
    float sin_roll  = sinf(roll_rad);

    // 标准倾斜补偿公式 (假设 X指北，Y指东，Z指下)
    float mx_h = mx_calibrated * cos_pitch + my_calibrated * sin_roll * sin_pitch + mz_calibrated * cos_roll * sin_pitch;
    float my_h = my_calibrated * cos_roll - mz_calibrated * sin_roll;

    // 4. 用水平面上的分量计算航向角
    // atan2f(my_h, mx_h) 输出弧度，范围 -pi 到 pi
    float heading_rad = atan2f(my_h, mx_h);

    // 5. 弧度转为角度
    float heading_deg = heading_rad * 180.0f / PI;

    // 磁偏角补偿 (将 磁北 转为 真北)
    // 假设你在北京，磁偏角为 -7 度 (西偏7度)
	//#define MAG_DECLINATION  -7.0f
	//heading_deg += MAG_DECLINATION;
    heading_deg += mag_declination;

    // 6. 将角度限制在 0 ~ 360 度之间
    if (heading_deg < 0.0f)
    {
        heading_deg += 360.0f;
    }
    else if (heading_deg >= 360.0f)
    {
        heading_deg -= 360.0f;
    }
    set_mag(heading_deg);
}

void on_gps_received(void) {
	if (!home_is_set)
	{
		return;
	}

    // 当前 GPS 坐标
    float current_lat = gps_data.fLat;
    float current_lon = gps_data.fLon;

    // 1. 转换为局部米坐标 (此时 current_x 是东向 E, current_y 是北向 N)
    float current_x_east, current_y_north;

    convert_gps_to_local(current_lat, current_lon, &current_x_east, &current_y_north);

    // 获取全局坐标系(ENU)下的速度，并转为 m/s
    float gps_v_east = (float)gps_data.velE / 1000.0f;  // 东向速度
    float gps_v_north = (float)gps_data.velN / 1000.0f; // 北向速度

    // ================= 坐标系转换 ================
    // 将 ENU(东,北) 转换为 机体坐标系(X: 右为正, Y: 前为正)
    // 航向角 yaw：0度=正北，90度=正东（顺时针为正）
    // 注意：这里必须使用当前的实时 yaw！
    float yaw_rad = yaw * DEG_TO_RAD;
    float cos_yaw = cosf(yaw_rad);
    float sin_yaw = sinf(yaw_rad);

    // 旋转矩阵推导：
    // 当 yaw=0 (朝北) 时：前向(Y) = 北向, 右向(X) = 东向
    // 当 yaw=90 (朝东) 时：前向(Y) = 东向, 右向(X) = -北向
    // 所以：
    // X_body (右) = East * cos(yaw) - North * sin(yaw)
    // Y_body (前) = North * cos(yaw) + East * sin(yaw)

    // 1. 绝对位置转机体坐标 (🚨 修复2：传绝对位置给卡尔曼，不是误差！)
	float current_x_body = current_x_east * cos_yaw - current_y_north * sin_yaw;  // 当前横向绝对位置 (右为正)
	float current_y_body = current_y_north * cos_yaw + current_x_east * sin_yaw;  // 当前纵向绝对位置 (前为正)

    // GPS 速度转机体坐标
    float gps_vx_body = gps_v_east * cos_yaw - gps_v_north * sin_yaw;  // 横向速度 (右为正)
    float gps_vy_body = gps_v_north * cos_yaw + gps_v_east * sin_yaw;  // 纵向速度 (前为正)
    // ============================================

    // 3. 输入给卡尔曼滤波器进行融合
    // 传入机体坐标系下的位置和速度 (X:右, Y:前)
    app.UpdateGPS(current_x_body, current_y_body, gps_vx_body, gps_vy_body, gps_data.hAcc * 0.001f);

    float dx_body, dy_body;
    get_target_distance(&dx_body, &dy_body);

    // 4. 计算距离 (用于判断是否到达目标)
    // 距离是标量，在哪个坐标系下算都是一样的
    distance = sqrtf(dx_body * dx_body + dy_body * dy_body);
    hdistance = sqrtf(current_x_body * current_x_body + current_y_body * current_y_body);

    // 打印机体坐标系下的数据
    Log::I("[gps]fixType:%d, numSV:%d, fLon:%f, fLat:%f, hAcc:%ld, ds:%f, dx:%f, dy:%f, hds:%f, hx:%f, hy:%f",
             gps_data.fixType, gps_data.numSV, gps_data.fLon, gps_data.fLat, gps_data.hAcc,
             distance, dx_body, dy_body, hdistance, current_x_body, current_y_body);

    // 5. 航向融合 (注意：航向角计算依然使用的是全局 ENU 速度)
    float gps_heading = get_gps_heading(gps_v_east, gps_v_north);

    // 如果 GPS 速度太低，不进行修正，保持 IMU 原样
    if (gps_heading > 0.0f)
    {
        //set_mag(gps_heading);
    }
}


float heading_to_home()
{
	if ((HAL_GetTick() - gps_update_tick) > 1000)
	{
		return -1.0;
	}
	// 当前 GPS 坐标
	float current_lat = gps_data.fLat;
	float current_lon = gps_data.fLon;

	// 1. 转换为局部米坐标
	float current_x, current_y;
	float target_x, target_y;

	convert_gps_to_local(current_lat, current_lon, &current_x, &current_y);
	convert_gps_to_local(home_lat, home_lon, &target_x, &target_y);

	// 2. 计算位置误差 (目标位置 - 当前位置)
	float dx = target_x - current_x; // 东向距离差
	float dy = target_y - current_y; // 北向距离差

	// 3. 计算距离 (用于判断是否到达目标)
	float distance = sqrtf(dx * dx + dy * dy);

	// 4. 计算目标方位角 (0为北，90为东，顺时针为正)
	// 注意: 这里传入的是 (东向, 北向)，即
	float desired_heading_rad = atan2f(dx, dy);
	float desired_heading_deg = desired_heading_rad * 180.0f / PI;
	if (desired_heading_deg < 0.0f) desired_heading_deg += 360.0f;

	// 此时，desired_heading_deg 就是你需要把机头对准的方向。
	return desired_heading_deg;
}


float heading_to_target()
{
	if ((HAL_GetTick() - gps_update_tick) > 1000)
	{
		return -1.0;
	}
	// 当前 GPS 坐标
	float current_lat = gps_data.fLat;
	float current_lon = gps_data.fLon;

	// 1. 转换为局部米坐标
	float current_x, current_y;
	float target_x, target_y;

	convert_gps_to_local(current_lat, current_lon, &current_x, &current_y);
	convert_gps_to_local(target_lat, target_lon, &target_x, &target_y);

	// 2. 计算位置误差 (目标位置 - 当前位置)
	float dx = target_x - current_x; // 东向距离差
	float dy = target_y - current_y; // 北向距离差

	// 3. 计算距离 (用于判断是否到达目标)
	float distance = sqrtf(dx * dx + dy * dy);

	// 4. 计算目标方位角 (0为北，90为东，顺时针为正)
	// 注意: 这里传入的是 (东向, 北向)，即
	float desired_heading_rad = atan2f(dx, dy);
	float desired_heading_deg = desired_heading_rad * 180.0f / PI;
	if (desired_heading_deg < 0.0f) desired_heading_deg += 360.0f;

	// 此时，desired_heading_deg 就是你需要把机头对准的方向。
	return desired_heading_deg;
}


#if 0


// 全局变量，表示当前的绝对目标航向（机头需要对准的方向）
float target_bearing = 0.0f;

// GPS 修正函数（在飞行中高频调用，只要有 GPS 速度就调用）
void update_yaw_with_gps(float gps_vel_e, float gps_vel_n)
{
    float gps_heading = get_gps_heading(gps_vel_e, gps_vel_n);

    // 如果 GPS 速度太低，不进行修正，保持 IMU 原样
    if (gps_heading < 0.0f) return;

    // 计算 GPS 航向 与 当前 IMU 航向 的差值
    float diff = gps_heading - yaw;

    // 处理跨越 0/360 度边界
    if (diff > 180.0f) diff -= 360.0f;
    if (diff < -180.0f) diff += 360.0f;

    // 滑动平均滤波 (低通滤波)
    // 这里的 0.02 表示只采纳 2% 的 GPS 修正量。
    // 因为 GPS 航向在低速转弯时可能有延迟，所以不能全信，只能慢慢拉。
    yaw_offset += diff * 0.02f;
}


/*
如何把机头对准目标？
现在你的 yaw 在飞行中是准确的了。接下来就是计算目标在哪，以及怎么转过去。

计算目标方位：利用你之前的 convert_gps_to_local，算出目标相对于当前飞机的位置
(ΔX,ΔY)
。
计算所需航向：算出这个向量在地理坐标系下的角度。
计算误差并旋转：用目标航向减去当前机头航向，得到角度差，喂给偏航控制器。
*/
void navigate_to_target(float target_x, float target_y, float current_x, float current_y)
{
    // 1. 计算位置误差 (目标位置 - 当前位置)
    float dx = target_x - current_x;
    float dy = target_y - current_y;

    // 距离判断，如果已经到了目标附近
    float distance = sqrtf(dx * dx + dy * dy);
    if (distance < 1.0f) { // 小于1米
        // 停止前进，可以悬停或执行下一步
        return;
    }

    // 2. 计算目标航向 (目标相对于飞机的地理绝对方位)
    // atan2f(东向距离, 北向距离) -> 0为北，90为东
    float desired_heading_rad = atan2f(dx, dy);
    float desired_heading_deg = desired_heading_rad * 180.0f / PI;
    if (desired_heading_deg < 0.0f) desired_heading_deg += 360.0f;

    // 3. 计算航向误差 (我要转多少度才能对准目标)
    float yaw_error = desired_heading_deg - yaw;

    // 处理跨越 0/360 度边界，确保误差在 -180 ~ +180 之间
    if (yaw_error > 180.0f)  yaw_error -= 360.0f;
    if (yaw_error < -180.0f) yaw_error += 360.0f;

    // 4. 将误差送给偏航速率控制器 (PID)
    // 假设你有一个 yaw_rate_controller 函数
    // 如果机头偏左(yaw_error > 0)，则向右打舵
    float yaw_rate_cmd = yaw_error * 2.0f; // 简单的比例控制，系数2.0需调参
    yaw_rate_controller(yaw_rate_cmd);

    // 💡 避坑指南：旋转时先别往前飞！
    // 如果 abs(yaw_error) > 30 度，说明机头还没对准，此时不要给前进速度，原地位旋。
    // 等对准了（yaw_error < 10度），再给前进速度。
    if (fabsf(yaw_error) > 30.0f) {
        set_velocity(0, 0); // 前进和侧飞速度设为0
    } else {
        set_velocity(cruise_speed, 0); // 对准了，全速前进
    }
}


// 假设你的目标点 GPS 坐标已经通过地面站或其他方式写入
float target_lat = 22.123456f;
float target_lon = 113.987654f;

// 当前 GPS 坐标
float current_lat = gps.latitude;
float current_lon = gps.longitude;

// 1. 转换为局部米坐标
float current_x, current_y;
float target_x, target_y;

convert_gps_to_local(current_lat, current_lon, &current_x, &current_y);
convert_gps_to_local(target_lat, target_lon, &target_x, &target_y);

// 2. 计算位置误差 (目标位置 - 当前位置)
float dx = target_x - current_x; // 东向距离差
float dy = target_y - current_y; // 北向距离差

// 3. 计算距离 (用于判断是否到达目标)
float distance = sqrtf(dx * dx + dy * dy);

// 4. 计算目标方位角 (0为北，90为东，顺时针为正)
// 注意: 这里传入的是 (东向, 北向)，即
float desired_heading_rad = atan2f(dx, dy);
float desired_heading_deg = desired_heading_rad * 180.0f / PI;
if (desired_heading_deg < 0.0f) desired_heading_deg += 360.0f;

// 此时，desired_heading_deg 就是你需要把机头对准的方向。
// 结合之前说的，用这个值去和当前 IMU 的 yaw 做差，进行航向控制。

/*
信号极强：hAcc < 1000 (误差 < 1 米，通常在开阔地带且支持双频时能达到)
信号良好：1000 <= hAcc < 3000 (误差 1~3 米，正常飞行条件)
信号一般：3000 <= hAcc < 5000 (误差 3~5 米，可能有少量高楼遮挡)
信号弱（不建议起飞/导航）：hAcc >= 5000 (误差 > 5 米，信号较差，飘移大)
信号极差（丢弃数据）：hAcc >= 10000 (误差 > 10 米，模块可能快丢星了)
*/


#endif
