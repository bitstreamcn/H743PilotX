/*
 * file.cpp
 *
 *  Created on: 2026年8月8日
 *      Author: bitstream
 */
#include "main.h"
#include "file.h"
#include "log.h"
#include "ff.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "uartx.h"
#include "application.h"
#include "ring_buffer.h"
#include "sdmmc.h"

extern RingBuffer log_buffer;

// --- 全局变量定义 ---
FATFS fs;          // FatFS 文件系统对象 (相当于 FileX 的 sdio_disk)
FIL log_file;      // FatFS 文件对象 (相当于 FileX 的 FX_FILE)
char log_filename[20];

// 假设文件系统已经通过 f_mount 挂载成功
// 如果没有，请在系统初始化时调用: f_mount(&fs, "0:", 1);


// ==========================================
// 1. 获取下一个日志编号 & 清理空文件
// ==========================================
uint8_t get_next_log_number(void)
{
    DIR dir;
    FILINFO fno;
    uint16_t max_num = 0;
    FRESULT res;

    // 打开根目录
    res = f_opendir(&dir, "0:/");
    if (res != FR_OK) {
        return 1; // 打开目录失败，默认返回1
    }

    // 循环读取目录项
    while (1) {
        res = f_readdir(&dir, &fno);
        if (res != FR_OK || fno.fname[0] == '\0') {
            break; // 读取完毕或出错
        }

        // 跳过文件夹
        if (fno.fattrib & AM_DIR) {
            continue;
        }

        // 提取数字部分（假设文件名如 "123.log"）
        size_t len = strlen(fno.fname);
        if (len > 4 && strcasecmp(fno.fname + len - 4, ".log") == 0)
        {
            // 检查文件大小 (FatFS 中 f_size 获取大小)
            if (fno.fsize == 0)
            {
                // 删除0字节文件
                f_unlink(fno.fname);
            }
            else
            {
                char *p = fno.fname;
                UINT num = 0;
                while (*p >= '0' && *p <= '9')
                {
                    num = num * 10 + (*p - '0');
                    p++;
                }
                if (*p == '.' && strcasecmp(p, ".log") == 0)
                {
                    if (num > max_num)
                    {
                        max_num = num;
                    }
                }
            }
        }
    }

    f_closedir(&dir);

    if (max_num == 0) {
        return 1;
    } else {
        return max_num + 1;
    }
}


// ==========================================
// 2. 创建并打开日志文件
// ==========================================
FRESULT create_log_file(void)
{
	uint16_t next_num = get_next_log_number();
    // FatFS 路径需要带上盘符 "0:/"
    sprintf(log_filename, "0:/%d.log", next_num);

    // 创建并打开文件 (FA_OPEN_ALWAYS: 如果不存在则创建，存在则打开)
    // FA_WRITE: 写权限
    FRESULT res = f_open(&log_file, log_filename, FA_OPEN_ALWAYS | FA_WRITE);
    if (res != FR_OK) {
        return res;
    }

    // 将文件指针移动到文件末尾 (追加写入)
    f_lseek(&log_file, f_size(&log_file));

    return FR_OK; // FR_OK = 0
}


// ==========================================
// 3. 清空日志 (删除所有文件)
// ==========================================
void clearlog()
{
    DIR dir;
    FILINFO fno;
    FRESULT res;

    // 重新挂载文件系统以确保状态干净 (相当于 fx_media_close/open)
    f_mount(NULL, "0:", 0);
    res = f_mount(&fs, "0:", 1);
    if (res != FR_OK) return;

    res = f_opendir(&dir, "0:/");
    if (res != FR_OK) return;

    while (1) {
        res = f_readdir(&dir, &fno);
        if (res != FR_OK || fno.fname[0] == '\0') break;

        if (!(fno.fattrib & AM_DIR)) { // 只删除文件，不删文件夹
            f_unlink(fno.fname);
            LogI("delete %s", fno.fname);
            HAL_Delay(10);
        }
    }
    f_closedir(&dir);
}


// ==========================================
// 4. 下载日志 (通过串口发送)
// ==========================================
void downlog()
{
    DIR dir;
    FILINFO fno;
    FRESULT res;
    LogPause();

    // 重新挂载
    f_mount(NULL, "0:", 0);
    res = f_mount(&fs, "0:", 1);
    if (res != FR_OK) {
        LogResume();
        return;
    }

    res = f_opendir(&dir, "0:/");
    if (res != FR_OK) {
        LogResume();
        return;
    }

    while (1) {
        res = f_readdir(&dir, &fno);
        if (res != FR_OK || fno.fname[0] == '\0') break;

        char buffer[512];
        char line[128];
        int iline = 0;
        do {
            FIL file;
            size_t name_len = strlen(fno.fname);
            if (name_len < 5) break;

            if (strcasecmp(fno.fname + name_len - 4, ".log") != 0) break;

            // 打开文件只读
            res = f_open(&file, fno.fname, FA_READ);
            if (res != FR_OK) {
                snprintf(buffer, sizeof(buffer), "Failed to open file '%s', res: %d\r\n", fno.fname, res);
                UART1_DMA_Send((uint8_t*)buffer, strlen(buffer));
                break;
            }

            // 获取文件大小
            uint32_t file_size = f_size(&file);
            snprintf(buffer, sizeof(buffer), "\r\n-*-*- File: %s (size: %lu bytes) -*-*-\r\n", fno.fname, file_size);
            UART1_DMA_Send((uint8_t*)buffer, strlen(buffer));

            uint32_t total_read = 0;
            UINT bytes_read = 0;

            // 循环读取
            while (1) {
                res = f_read(&file, buffer, sizeof(buffer), &bytes_read);
                if (res != FR_OK || bytes_read == 0) break;

                bool issent = false;
                for (int i = 0; i < bytes_read; i++) {
                    if (iline >= sizeof(line)) {
                        issent = true;
                    } else {
                        if (buffer[i] != '\n') {
                            line[iline] = buffer[i];
                            iline++;
                        } else {
                            issent = true;
                        }
                    }

                    if (issent) {
                        if (iline > 0) {
                            if (0 != UART1_DMA_Send((uint8_t*)line, iline)) {
                                // 串口错误处理
                            }
                            HAL_Delay(10);
                        }
                        issent = false;
                        iline = 0;
                    }
                }
                total_read += bytes_read;
            }

            snprintf(buffer, sizeof(buffer), "\r\n-*-*- End of %s (total %lu bytes) -*-*-\r\n\r\n", fno.fname, total_read);
            UART1_DMA_Send((uint8_t*)buffer, strlen(buffer));

            f_close(&file);
        } while(false);
    }

    f_closedir(&dir);
    LogResume();
}


// ==========================================
// 5. 日志写入线程
// ==========================================
#define CACHE_SIZE  4096  // 4KB 缓存
static char write_cache[CACHE_SIZE];
static uint16_t cache_len = 0;
static bool is_init = false;
static bool file_max = false;

// 最多写入10M日志
#define MAX_LOG_FILE_SIZE (10 * 1024 * 1024) // 10MB

void file_log_write()
{
    char log_string[256];
    static uint32_t real_written_size = 0;
    static bool is_start_log = false;
    static uint32_t write_time = 0; // 移出 do-while，保持状态

    // 1. 处理解除武装逻辑
    if (!app.IsArmed()) {
        if (is_init) { // 只有在初始化过的情况下才执行关闭
            //f_lseek(&log_file, real_written_size);
            //f_truncate(&log_file);
            f_close(&log_file);
            is_init = false;
            is_start_log = false;

            // 🚨 修复A：必须清零状态，防止下次记录时数据错乱
            cache_len = 0;
            real_written_size = 0;
            write_time = 0;
        }
        return; // 解除武装后直接退出，不执行后续写入逻辑
    }

    // 2. 处理武装逻辑 (初始化 SD 卡和文件)
    if (app.IsArmed() && !is_init) {
        if (!is_sdmmc_init) return;
        f_mount(NULL, "0:", 0);
        if (FR_OK != f_mount(&fs, "0:", 1)) return;

        if (create_log_file() != FR_OK) return;
        /*
        if (f_expand(&log_file, MAX_LOG_FILE_SIZE, 1) != FR_OK) {
            f_close(&log_file);
            return;
        }
        */
        file_max = false;
        is_init = true;
        is_start_log = true;
        write_time = HAL_GetTick(); // 初始化时间戳
    }

    if (!is_start_log) return;

    // 3. 检查文件是否已满
    if (real_written_size >= (MAX_LOG_FILE_SIZE - 512)) { // 留 512B 余量
        //file_max = true;
    }
    if (file_max) return;

    // 4. 核心数据搬运与写入循环
    // 🚨 修复C：改为 while 循环，尽量排空环形缓冲区，防止溢出丢数据
    while (rb_avail(&log_buffer) > 0) {
        uint32_t len = sizeof(log_string); // 256
        uint32_t readsize = rb_get(&log_buffer, (uint8_t *)log_string, len);
        if (readsize == 0) break;

        // 检查缓存是否会溢出，如果快满了，强制触发写入
        if (cache_len + readsize > CACHE_SIZE) {
            write_time = HAL_GetTick(); // 重置计时
            if (cache_len > 0) {
                UINT bw = 0;
                // 🚨 修复B：按 512 整数倍写入，消除读-改-写卡顿
                uint16_t writelen = (cache_len / 512) * 512;
                if (writelen > 0) {
                    f_write(&log_file, write_cache, writelen, &bw);
                    real_written_size += bw;
                    cache_len -= bw;
                    memmove(write_cache, write_cache + bw, cache_len);
                }
            }
        }

        // 将新数据拷贝到缓存
        memcpy(write_cache + cache_len, log_string, readsize);
        cache_len += readsize;

        // 检查是否满足写入条件：攒够 1KB(或2KB) 或 超过3秒未写入
        uint32_t remain = HAL_GetTick() - write_time;
        if (cache_len >= 1024 || remain > 3000) {
            write_time = HAL_GetTick();
            UINT bw = 0;

            // 如果是因为超时(3秒)触发，但不足 512 字节，也必须写，否则日志会延迟太久
            if (remain > 3000 && cache_len < 512) {
                f_write(&log_file, write_cache, cache_len, &bw);
                real_written_size += bw;
                cache_len -= bw;
                memmove(write_cache, write_cache + bw, cache_len);
            }
            else {
                // 正常按 512 整数倍写入
                uint16_t writelen = (cache_len / 512) * 512;
                if (writelen > 0) {
                    f_write(&log_file, write_cache, writelen, &bw);
                    real_written_size += bw;
                    cache_len -= bw;
                    memmove(write_cache, write_cache + bw, cache_len);
                }
            }
        }
    }
}

void file_clearlog()
{
    LogI("clearlog");
    clearlog();
}

void file_downlog()
{
    LogI("downlog");
    downlog();
}

