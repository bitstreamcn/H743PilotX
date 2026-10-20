/*
 * mag_calibration.cpp
 *
 *  Created on: 2026年9月18日
 *      Author: bitstream
 */
#include <stdint.h>
#include <math.h>
#include <string.h>

// 定义 4x4 矩阵和 4x1 向量
typedef struct {
    float m[4][4];
} Matrix4x4;

typedef struct {
    float v[4];
} Vector4;

// 收集数据的结构体
typedef struct {
    float sum_x, sum_y, sum_z;
    float sum_xx, sum_yy, sum_zz;
    float sum_xy, sum_xz, sum_yz;
    float sum_xxx, sum_yyy, sum_zzz;
    float sum_xxy, sum_xxz, sum_yyx, sum_yyz, sum_zzx, sum_zzy;
    float sum_xyy, sum_xzz, sum_yzz, sum_yxx, sum_zxx, sum_zyy;
    uint32_t sample_count;
} MagCalibData;

MagCalibData calib_data = {0};

// 1. 采集数据：在画 8 字校准时，高频调用此函数喂入原始数据
void MagCalib_CollectData(float x, float y, float z) {
    calib_data.sum_x += x;
    calib_data.sum_y += y;
    calib_data.sum_z += z;

    calib_data.sum_xx += x * x;
    calib_data.sum_yy += y * y;
    calib_data.sum_zz += z * z;

    calib_data.sum_xy += x * y;
    calib_data.sum_xz += x * z;
    calib_data.sum_yz += y * z;

    // 三次项
    calib_data.sum_xxx += x * x * x;
    calib_data.sum_yyy += y * y * y;
    calib_data.sum_zzz += z * z * z;

    calib_data.sum_xxy += x * x * y;
    calib_data.sum_xxz += x * x * z;
    calib_data.sum_yyx += y * y * x;
    calib_data.sum_yyz += y * y * z;
    calib_data.sum_zzx += z * z * x;
    calib_data.sum_zzy += z * z * y;

    calib_data.sample_count++;
}

// 简单的 4x4 矩阵求逆 (高斯消元法)
static int Matrix4x4_Inverse(Matrix4x4 *m, Matrix4x4 *inv) {
    float a[4][8];
    float temp;
    int i, j, k;

    // 构建 [m | I]
    for (i = 0; i < 4; i++) {
        for (j = 0; j < 4; j++) {
            a[i][j] = m->m[i][j];
            a[i][j + 4] = (i == j) ? 1.0f : 0.0f;
        }
    }

    // 高斯消元
    for (i = 0; i < 4; i++) {
        if (fabsf(a[i][i]) < 1e-7f) return -1; // 奇异矩阵
        temp = a[i][i];
        for (j = 0; j < 8; j++) a[i][j] /= temp;
        for (k = 0; k < 4; k++) {
            if (k != i) {
                temp = a[k][i];
                for (j = 0; j < 8; j++) {
                    a[k][j] -= a[i][j] * temp;
                }
            }
        }
    }

    for (i = 0; i < 4; i++)
        for (j = 0; j < 4; j++)
            inv->m[i][j] = a[i][j + 4];
    return 0;
}

// 矩阵乘法 4x4 * 4x1 = 4x1
static void Matrix4x4_Mul_Vector4(Matrix4x4 *m, Vector4 *v_in, Vector4 *v_out) {
    for (int i = 0; i < 4; i++) {
        v_out->v[i] = 0;
        for (int j = 0; j < 4; j++) {
            v_out->v[i] += m->m[i][j] * v_in->v[j];
        }
    }
}

// 2. 结束采集，计算偏移量和缩放系数
// 输出: offset[3], scale[3]
int MagCalib_Compute(float *offset_x, float *offset_y, float *offset_z,
                     float *scale_x, float *scale_y, float *scale_z) {
    if (calib_data.sample_count < 10) return -1; // 数据太少

    float n = (float)calib_data.sample_count;

    // 构建方程组 M * X = B
    // X = [2*x0, 2*y0, 2*z0, R^2 - x0^2 - y0^2 - z0^2]^T
    Matrix4x4 M = {0};
    Vector4 B = {0};

    // 填充矩阵 M (参考椭球拟合最小二乘法推导)
    M.m[0][0] = calib_data.sum_xx; M.m[0][1] = calib_data.sum_xy; M.m[0][2] = calib_data.sum_xz; M.m[0][3] = calib_data.sum_x;
    M.m[1][0] = calib_data.sum_xy; M.m[1][1] = calib_data.sum_yy; M.m[1][2] = calib_data.sum_yz; M.m[1][3] = calib_data.sum_y;
    M.m[2][0] = calib_data.sum_xz; M.m[2][1] = calib_data.sum_yz; M.m[2][2] = calib_data.sum_zz; M.m[2][3] = calib_data.sum_z;
    M.m[3][0] = calib_data.sum_x;  M.m[3][1] = calib_data.sum_y;  M.m[3][2] = calib_data.sum_z;  M.m[3][3] = n;

    // 填充向量 B
    B.v[0] = calib_data.sum_xxx + calib_data.sum_xyy + calib_data.sum_xzz;
    B.v[1] = calib_data.sum_xxy + calib_data.sum_yyy + calib_data.sum_yzz;
    B.v[2] = calib_data.sum_xxz + calib_data.sum_yyz + calib_data.sum_zzz;
    B.v[3] = calib_data.sum_xx + calib_data.sum_yy + calib_data.sum_zz;

    // 求逆并解算
    Matrix4x4 M_inv;
    if (Matrix4x4_Inverse(&M, &M_inv) != 0) return -1;

    Vector4 X;
    Matrix4x4_Mul_Vector4(&M_inv, &B, &X);

    // 提取偏移量 (中心点)
    *offset_x = X.v[0] / 2.0f;
    *offset_y = X.v[1] / 2.0f;
    *offset_z = X.v[2] / 2.0f;

    // 计算平均半径
    float avg_radius = sqrtf(X.v[3] + (*offset_x)*(*offset_x) + (*offset_y)*(*offset_y) + (*offset_z)*(*offset_z));
    if (avg_radius < 1e-3f) return -1;

    // 计算缩放比例 (将椭球拉成半径为 1 的球)
    // 这里用平均值做归一化
    *scale_x = 1.0f / (avg_radius);
    *scale_y = 1.0f / (avg_radius);
    *scale_z = 1.0f / (avg_radius);

    // 重置采集器，方便下次使用
    memset(&calib_data, 0, sizeof(calib_data));
    return 0; // 成功
}



#if 0

//第一步：采集数据
//让设备进入校准模式，用户开始画 8 字。在主循环或定时器中（比如 100Hz），读取磁力计原始数据并喂给采集器：

c

// 假设 mx, my, mz 是读取到的浮点原始数据
MagCalib_CollectData(mx, my, mz);
第二步：结束采集并计算
当用户按下按键，或者采集了约 10 秒（几千个点）后，触发计算：

c

float off_x, off_y, off_z;
float sca_x, sca_y, sca_z;

if (MagCalib_Compute(&off_x, &off_y, &off_z, &sca_x, &sca_y, &sca_z) == 0) {
    printf("校准成功！\n");
    printf("Offset: X=%.2f, Y=%.2f, Z=%.2f\n", off_x, off_y, off_z);

    // ⚠️ 重要：将算出的 off 和 sca 保存到 Flash 或 EEPROM 中！
} else {
    printf("校准失败，数据不够或矩阵奇异。\n");
}
第三步：应用校准数据（正常运行阶段）
以后每次读取磁力计，都要减去偏移量并乘以缩放系数：

c

// 读取真实数据
float mx_raw = read_mx();
float my_raw = read_my();
float mz_raw = read_mz();

// 应用椭球拟合校准
float mx_calibrated = (mx_raw - off_x) * sca_x;
float my_calibrated = (my_raw - off_y) * sca_y;
float mz_calibrated = (mz_raw - off_z) * sca_z;

// 传给姿态解算器 (Mahony / 卡尔曼)
// ...
#endif



