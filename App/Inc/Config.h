/*
 * Config.h
 *
 *  Created on: Jun 21, 2025
 *      Author: bitstream
 */

#ifndef CONFIG_H_
#define CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>

//=========================================================================================
// Parameter List
//=========================================================================================
#define MF_PARAM_LIST \
/*系统设置*/\
MF_PARAM( sys_fly_mode, 0, int32_t, 'i') /*飞行模式 0=手动 1=油门居中悬停*/ \
\
MF_PARAM( imu_docal, 1, int32_t, 'i') /*每次启动时进行校准*/ \
MF_PARAM( imu_cal_ax, 0, float, 'f') /*accel x zero offset calibration 默认0*/ \
MF_PARAM( imu_cal_ay, 0, float, 'f') /*accel y zero offset calibration 默认0*/ \
MF_PARAM( imu_cal_az, 0, float, 'f') /*accel z zero offset calibration 默认0*/ \
MF_PARAM( imu_cal_gx, 0, float, 'f') /*gyro x zero offset calibration 默认0*/ \
MF_PARAM( imu_cal_gy, 0, float, 'f') /*gyro y zero offset calibration 默认0*/ \
MF_PARAM( imu_cal_gz, 0, float, 'f') /*gyro z zero offset calibration 默认0*/ \
\
MF_PARAM( mag_docal, 0, int32_t, 'i') /*启动时进行校准*/ \
MF_PARAM( mag_cal_x, 0.0, float, 'f') /*magnetometer x zero offset calibration 默认0*/ \
MF_PARAM( mag_cal_y, 0.0, float, 'f') /*magnetometer y zero offset calibration 默认0*/ \
MF_PARAM( mag_cal_z, 0.0, float, 'f') /*magnetometer z zero offset calibration 默认0*/ \
MF_PARAM( mag_cal_sx, 1.0, float, 'f') /*magnetometer x scale calibration 默认1*/ \
MF_PARAM( mag_cal_sy, 1.0, float, 'f') /*magnetometer y scale calibration 默认1*/ \
MF_PARAM( mag_cal_sz, 1.0, float, 'f') /*magnetometer z scale calibration 默认1*/ \
\
MF_PARAM( bat_cal_v, 1, float, 'f') /*battery adc voltage scale calibration, value is actual_voltage_in_v / adc_reading*/ \
MF_PARAM( bat_cal_i, 1, float, 'f') /*battery adc current scale calibration, value is actual_current_in_a / adc_reading, ina226: rshunt value in ohm*/ \
\
MF_PARAM( rcl_thr_ch, 1, int32_t, 'i') /*1-based channel number*/ \
MF_PARAM( rcl_thr_pull, 1000, int32_t, 'i') \
MF_PARAM( rcl_thr_mid, 1500, int32_t, 'i') \
MF_PARAM( rcl_thr_push, 2000, int32_t, 'i') \
MF_PARAM( rcl_thr_dead, 10, int32_t, 'i') \
\
MF_PARAM( rcl_rol_ch, 2, int32_t, 'i') \
MF_PARAM( rcl_rol_left, 1000, int32_t, 'i') \
MF_PARAM( rcl_rol_mid, 1500, int32_t, 'i') \
MF_PARAM( rcl_rol_right, 2000, int32_t, 'i') \
MF_PARAM( rcl_rol_dead, 10, int32_t, 'i') \
\
MF_PARAM( rcl_pit_ch, 3, int32_t, 'i') \
MF_PARAM( rcl_pit_pull, 1000, int32_t, 'i') \
MF_PARAM( rcl_pit_mid, 1500, int32_t, 'i') \
MF_PARAM( rcl_pit_push, 2000, int32_t, 'i') \
MF_PARAM( rcl_pit_dead, 10, int32_t, 'i') \
\
MF_PARAM( rcl_yaw_ch, 4, int32_t, 'i') \
MF_PARAM( rcl_yaw_left, 1000, int32_t, 'i') \
MF_PARAM( rcl_yaw_mid, 1500, int32_t, 'i') \
MF_PARAM( rcl_yaw_right, 2000, int32_t, 'i') \
MF_PARAM( rcl_yaw_dead, 10, int32_t, 'i') \
\
MF_PARAM( rcl_arm_ch, 5, int32_t, 'i') \
MF_PARAM( rcl_arm_min, 1600, int32_t, 'i') /*armed pwm range min*/ \
MF_PARAM( rcl_arm_max, 2500, int32_t, 'i') /*armed pwm range max*/ \
\
MF_PARAM( rcl_flt_ch, 6, int32_t, 'i') \
MF_PARAM( rcl_flt_min, 1165, int32_t, 'i') /*6-pos switch lowest pwm (flight mode 0)*/ \
MF_PARAM( rcl_flt_max, 1815, int32_t, 'i') /*6-pos switch lowest pwm (flight mode 5)*/ \
\
/*Serial pins*/ \
MF_PARAM( pin_ser0_rx, -1, int32_t, 'p') \
MF_PARAM( pin_ser1_rx, -1, int32_t, 'p') \
MF_PARAM( pin_ser2_rx, -1, int32_t, 'p') \
MF_PARAM( pin_ser3_rx, -1, int32_t, 'p') \
MF_PARAM( pin_ser4_rx, -1, int32_t, 'p') \
MF_PARAM( pin_ser5_rx, -1, int32_t, 'p') \
MF_PARAM( pin_ser6_rx, -1, int32_t, 'p') \
MF_PARAM( pin_ser7_rx, -1, int32_t, 'p') \
\
MF_PARAM( pin_ser0_tx, -1, int32_t, 'p') \
MF_PARAM( pin_ser1_tx, -1, int32_t, 'p') \
MF_PARAM( pin_ser2_tx, -1, int32_t, 'p') \
MF_PARAM( pin_ser3_tx, -1, int32_t, 'p') \
MF_PARAM( pin_ser4_tx, -1, int32_t, 'p') \
MF_PARAM( pin_ser5_tx, -1, int32_t, 'p') \
MF_PARAM( pin_ser6_tx, -1, int32_t, 'p') \
MF_PARAM( pin_ser7_tx, -1, int32_t, 'p') \
\
MF_PARAM( pin_ser0_inv, -1, int32_t, 'p') \
MF_PARAM( pin_ser1_inv, -1, int32_t, 'p') \
MF_PARAM( pin_ser2_inv, -1, int32_t, 'p') \
MF_PARAM( pin_ser3_inv, -1, int32_t, 'p') \
MF_PARAM( pin_ser4_inv, -1, int32_t, 'p') \
MF_PARAM( pin_ser5_inv, -1, int32_t, 'p') \
MF_PARAM( pin_ser6_inv, -1, int32_t, 'p') \
MF_PARAM( pin_ser7_inv, -1, int32_t, 'p') \
\
/*I2C pins*/ \
MF_PARAM( pin_i2c0_sda, -1, int32_t, 'p') \
MF_PARAM( pin_i2c1_sda, -1, int32_t, 'p') \
MF_PARAM( pin_i2c2_sda, -1, int32_t, 'p') \
MF_PARAM( pin_i2c3_sda, -1, int32_t, 'p') \
\
MF_PARAM( pin_i2c0_scl, -1, int32_t, 'p') \
MF_PARAM( pin_i2c1_scl, -1, int32_t, 'p') \
MF_PARAM( pin_i2c2_scl, -1, int32_t, 'p') \
MF_PARAM( pin_i2c3_scl, -1, int32_t, 'p') \
\
/*SPI pins*/ \
MF_PARAM( pin_spi0_miso, -1, int32_t, 'p') \
MF_PARAM( pin_spi1_miso, -1, int32_t, 'p') \
MF_PARAM( pin_spi2_miso, -1, int32_t, 'p') \
MF_PARAM( pin_spi3_miso, -1, int32_t, 'p') \
\
MF_PARAM( pin_spi0_mosi, -1, int32_t, 'p') \
MF_PARAM( pin_spi1_mosi, -1, int32_t, 'p') \
MF_PARAM( pin_spi2_mosi, -1, int32_t, 'p') \
MF_PARAM( pin_spi3_mosi, -1, int32_t, 'p') \
\
MF_PARAM( pin_spi0_sclk, -1, int32_t, 'p') \
MF_PARAM( pin_spi1_sclk, -1, int32_t, 'p') \
MF_PARAM( pin_spi2_sclk, -1, int32_t, 'p') \
MF_PARAM( pin_spi3_sclk, -1, int32_t, 'p') \
\
/*OUT pins*/ \
MF_PARAM( pin_out0, -1, int32_t, 'p') \
MF_PARAM( pin_out1, -1, int32_t, 'p') \
MF_PARAM( pin_out2, -1, int32_t, 'p') \
MF_PARAM( pin_out3, -1, int32_t, 'p') \
MF_PARAM( pin_out4, -1, int32_t, 'p') \
MF_PARAM( pin_out5, -1, int32_t, 'p') \
MF_PARAM( pin_out6, -1, int32_t, 'p') \
MF_PARAM( pin_out7, -1, int32_t, 'p') \
MF_PARAM( pin_out8, -1, int32_t, 'p') \
MF_PARAM( pin_out9, -1, int32_t, 'p') \
MF_PARAM( pin_out10, -1, int32_t, 'p') \
MF_PARAM( pin_out11, -1, int32_t, 'p') \
MF_PARAM( pin_out12, -1, int32_t, 'p') \
MF_PARAM( pin_out13, -1, int32_t, 'p') \
MF_PARAM( pin_out14, -1, int32_t, 'p') \
MF_PARAM( pin_out15, -1, int32_t, 'p') \
\
/*Other pins*/ \
MF_PARAM( pin_bbx_cs, -1, int32_t, 'p') \
MF_PARAM( pin_bat_i, -1, int32_t, 'p') \
MF_PARAM( pin_bat_v, -1, int32_t, 'p') \
MF_PARAM( pin_imu_cs, -1, int32_t, 'p') \
MF_PARAM( pin_imu_int, -1, int32_t, 'p') \
MF_PARAM( pin_led, -1, int32_t, 'p') \
MF_PARAM( pin_mmc_dat, -1, int32_t, 'p') \
MF_PARAM( pin_mmc_clk, -1, int32_t, 'p') \
MF_PARAM( pin_mmc_cmd, -1, int32_t, 'p') \
MF_PARAM( pin_rcl_ppm, -1, int32_t, 'p') \
\
/*AHR - AHRS*/ \
MF_PARAM( ahr_gizmo, 0, int32_t, 'e', mf_MAHONY,mf_MAHONY_BF,mf_MADGWICK,mf_VQF) \
\
/*BAR - Barometer*/ \
MF_PARAM( bar_gizmo, 0, int32_t, 'e', mf_NONE,mf_BMP280,mf_BMP388,mf_BMP390,mf_MS5611,mf_HP203B) \
MF_PARAM( bar_i2c_bus, -1, int32_t, 'i') \
MF_PARAM( bar_i2c_adr, 0, int32_t, 'i') \
MF_PARAM( bar_rate, 100, float, 'f') /*Barometer sample rate in Hz (default 100)*/ \
\
/*BAT - Battery Monitor*/ \
MF_PARAM( bat_gizmo, 0, int32_t, 'e', mf_NONE,mf_ADC,mf_INA226,mf_INA228) \
MF_PARAM( bat_i2c_bus, -1, int32_t, 'i') \
MF_PARAM( bat_i2c_adr, 0, int32_t, 'i') \
\
/*BBX - Black Box Data Logger*/ \
MF_PARAM( bbx_gizmo, 0, int32_t, 'e', mf_NONE,mf_SDSPI,mf_SDMMC) \
MF_PARAM( bbx_spi_bus, -1, int32_t, 'i') \
\
/*GPS*/ \
MF_PARAM( gps_gizmo, 0, int32_t, 'e', mf_NONE,mf_UBLOX) \
MF_PARAM( gps_ser_bus, -1, int32_t, 'i') \
MF_PARAM( gps_baud, 0, int32_t, 'i') \
\
/*IMU - Inertial Measurement Unit (acc/gyro)*/ \
MF_PARAM( imu_gizmo, 0, int32_t, 'e', mf_NONE,mf_BMI270,mf_MPU6000,mf_MPU6050,mf_MPU6500,mf_MPU9150,mf_MPU9250,mf_ICM45686,mf_ICM42688) \
MF_PARAM( imu_spi_bus, -1, int32_t, 'i') \
MF_PARAM( imu_i2c_bus, -1, int32_t, 'i') \
MF_PARAM( imu_i2c_adr, 0, int32_t, 'i') \
MF_PARAM( imu_align, 0, int32_t, 'e', mf_CW0,mf_CW90,mf_CW180,mf_CW270,mf_CW0FLIP,mf_CW90FLIP,mf_CW180FLIP,mf_CW270FLIP) \
MF_PARAM( imu_rate, 1000, float, 'f') /*IMU sample rate in Hz (default 1000) NOTE: not all IMU drivers support a different rate*/ \
MF_PARAM( imu_acc_lp, 70, float, 'f') /*Accelerometer Low Pass Filter cutoff frequency in Hz */ \
MF_PARAM( imu_gyr_lp, 60, float, 'f') /*Gyro Low Pass Filter cutoff frequency in Hz */ \
\
/*LED*/ \
MF_PARAM( led_on, 0, int32_t, 'e', mf_LOW_IS_ON,mf_HIGH_IS_ON) \
\
/*MAG - Magnetometer*/ \
MF_PARAM( mag_gizmo, 0, int32_t, 'e', mf_NONE,mf_QMC5883,mf_QMC6309,mf_RM3100) \
MF_PARAM( mag_i2c_bus, -1, int32_t, 'i') \
MF_PARAM( mag_i2c_adr, 0, int32_t, 'i') \
MF_PARAM( mag_lp, 1e10, float, 'f') /*Magnetometer Gyro Low Pass Filter cutoff frequency in Hz (default 1e10Hz, i.e. no filtering) */ \
\
/*RCL - Remote Control Link*/ \
MF_PARAM( rcl_gizmo, 0, int32_t, 'e', mf_NONE,mf_MAVLINK,mf_CRSF,mf_SBUS,mf_SBUS_NOT_INV,mf_DSM,mf_PPM) \
MF_PARAM( rcl_ser_bus, -1, int32_t, 'i') \
MF_PARAM( rcl_baud, 0, int32_t, 'i') \
MF_PARAM( rcl_num_ch, 8, int32_t, 'i') /*max 20*/ \
MF_PARAM( rcl_deadband, 0, int32_t, 'i') \
\
/*RDR - Radar*/ \
MF_PARAM( rdr_gizmo, 0, int32_t, 'e', mf_NONE,mf_LD2411S,mf_LD2413,mf_USD1,mf_SR04) \
MF_PARAM( rdr_ser_bus, -1, int32_t, 'i') \
MF_PARAM( rdr_baud, 0, int32_t, 'i') \
\
/*v2.0.1 additions */ \
MF_PARAM( pin_rdr_trig, -1, int32_t, 'p') \
MF_PARAM( pin_rdr_echo, -1, int32_t, 'p') \
MF_PARAM( imu_bus_type, 0, int32_t, 'e', mf_SPI,mf_I2C) \
\
/*校准*/ \
MF_PARAM( cal_roll, 16.0f, float, 'f') \
MF_PARAM( cal_pitch, -10.0f, float, 'f') \
MF_PARAM( cal_yaw, 0.0f, float, 'f') \
\
/*悬停油门大小*/\
MF_PARAM( alt_hover_thr, 0.20f, float, 'f') \
/*油门一阶低通滤波系数*/\
MF_PARAM( alt_thr_lp, 0.10f, float, 'f') \
/*加速度一阶低通滤波系数*/ \
MF_PARAM( acc_x_alpha, 0.5f, float, 'f') \
MF_PARAM( acc_y_alpha, 0.5f, float, 'f') \
MF_PARAM( acc_z_alpha, 0.5f, float, 'f') \
/*PID*/ \
MF_PARAM( pid_in_roll_p, 0.06f, float, 'f') \
MF_PARAM( pid_in_roll_i, 0.0001f, float, 'f') \
MF_PARAM( pid_in_roll_il, 5.0f, float, 'f')/*积分限幅*/ \
MF_PARAM( pid_in_roll_d, 0.00001f, float, 'f') \
MF_PARAM( pid_ou_roll_p, 1.4f, float, 'f') \
MF_PARAM( pid_ou_roll_i, 0.001f, float, 'f') \
MF_PARAM( pid_ou_roll_il, 15.0f, float, 'f') \
MF_PARAM( pid_ou_roll_d, 0.0f, float, 'f') \
MF_PARAM( pid_in_pitch_p, 0.07f, float, 'f') \
MF_PARAM( pid_in_pitch_i, 0.0001f, float, 'f') \
MF_PARAM( pid_in_pitch_il, 5.0f, float, 'f') \
MF_PARAM( pid_in_pitch_d, 0.00001f, float, 'f') \
MF_PARAM( pid_ou_pitch_p, 1.4f, float, 'f') \
MF_PARAM( pid_ou_pitch_i, 0.001f, float, 'f') \
MF_PARAM( pid_ou_pitch_il, 15.0f, float, 'f') \
MF_PARAM( pid_ou_pitch_d, 0.0f, float, 'f') \
MF_PARAM( pid_yaw_p, 0.5f, float, 'f') \
MF_PARAM( pid_yaw_i, 0.0f, float, 'f') \
MF_PARAM( pid_yaw_il, 5.0f, float, 'f') \
MF_PARAM( pid_yaw_d, 0.0f, float, 'f') \
MF_PARAM( pid_yaw_rate_p, 0.5f, float, 'f') \
MF_PARAM( pid_yaw_rate_i, 0.0f, float, 'f') \
MF_PARAM( pid_yaw_rate_il, 5.0f, float, 'f') \
MF_PARAM( pid_yaw_rate_d, 0.0f, float, 'f') \
MF_PARAM( pid_pmw_p, 0.0f, float, 'f') \
MF_PARAM( pid_pmw_i, 0.0f, float, 'f') \
MF_PARAM( pid_pmw_il, 5.0f, float, 'f') \
MF_PARAM( pid_pmw_d, 0.0f, float, 'f') \
MF_PARAM( pid_acc_p, 0.0f, float, 'f') \
MF_PARAM( pid_acc_i, 0.0f, float, 'f') \
MF_PARAM( pid_acc_il, 5.0f, float, 'f') \
MF_PARAM( pid_acc_d, 0.0f, float, 'f') \
MF_PARAM( pid_acc_alt_p, 1.0f, float, 'f') \
MF_PARAM( pid_acc_alt_i, 0.0f, float, 'f') \
MF_PARAM( pid_acc_alt_il, 5.0f, float, 'f') \
MF_PARAM( pid_acc_alt_d, 0.0f, float, 'f') \
MF_PARAM( pid_gps_p, 3.0f, float, 'f') \
MF_PARAM( pid_gps_i, 0.0f, float, 'f') \
MF_PARAM( pid_gps_il, 0.0f, float, 'f') \
MF_PARAM( pid_gps_d, 0.0f, float, 'f') \
MF_PARAM( pid_in_thr_p, 0.1f, float, 'f') \
MF_PARAM( pid_in_thr_i, 0.0f, float, 'f') \
MF_PARAM( pid_in_thr_il, 0.1f, float, 'f') \
MF_PARAM( pid_in_thr_d, 0.0f, float, 'f') \
MF_PARAM( pid_ou_thr_p, 0.1f, float, 'f') \
MF_PARAM( pid_ou_thr_i, 0.0f, float, 'f') \
MF_PARAM( pid_ou_thr_il, 0.2f, float, 'f') \
MF_PARAM( pid_ou_thr_d, 0.0f, float, 'f')


// Parameter count
#define MF_PARAM(name, defval, datatype, type, ...) +1
enum { CFG_PARAM_COUNT = 0 MF_PARAM_LIST };
#undef MF_PARAM

// Parameter list structure
typedef struct {
    const char* name;
    float defval;
    char type;
    const char* options;
} cfg_param_list_t;

// Parameter union
typedef union {
    struct {
        #define MF_PARAM(name, defval, datatype, type, ...) datatype name;
        MF_PARAM_LIST
        #undef MF_PARAM
    };
    float param_float[CFG_PARAM_COUNT];
    int32_t param_int32_t[CFG_PARAM_COUNT];
} cfg_param_t;

// Configuration header
typedef struct __attribute__((packed)) {
    uint8_t header0;
    uint8_t header1;
    uint8_t header2;
    uint8_t header3;
    uint16_t len;
    uint16_t _reserved0;
    uint32_t madflight_param_crc;
    uint8_t _reserved1[28];
} cfg_header_t;

// Configuration structure
typedef struct {
    cfg_header_t hdr;
    cfg_param_t param;
} cfg_t;

// Global configuration instance
extern cfg_t cfg;

// Function prototypes
void cfg_init(void);
uint16_t cfg_param_count(void);
bool cfg_get_name_and_value(uint16_t index, const char** name, float* value);
bool cfg_set_param(const char* namestr, const char* val);
bool cfg_set_param_mavlink(const char* namestr, float val);
int cfg_get_index(const char* namestr);
void cfg_clear(void);
bool cfg_load_from_eeprom(void);
void cfg_load_from_string(const char *batch);
void cfg_load_madflight(const char *board, const char *config);
bool cfg_write_to_eeprom(void);
float cfg_get_value_str(const char* namestr, float default_value);
float cfg_get_value_idx(int i);
void cfg_print_param_option(const int32_t* param);
bool cfg_get_option_string(uint16_t param_idx, int32_t param_val, char out_option[20]);
void cfg_print_module(const char* module_name);
void cfg_print_name_and_value(uint16_t i, const char* comment);
void cfg_print_value(uint16_t i);
void cfg_cli_dump(const char* filter);
void cfg_cli_diff(const char* filter);
void cfg_print_pins(void);
bool cfg_load_cmdline(const char *cmdline);
// Helper functions
int cfg_get_enum_index(const char* key, const char* options);
void cfg_print_options(const char *str);

bool cfg_get_str(uint16_t index, char* buffer);

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_H_ */
