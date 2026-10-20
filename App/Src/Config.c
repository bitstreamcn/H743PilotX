/*
 * Config.c
 *
 *  Created on: Jun 21, 2025
 *      Author: bitstream
 */

#include "Config.h"

#include <stdio.h>
#include <ctype.h>
#include "../Device/Flash/BL24C256A.h"
#include "log.h"

// Global configuration instance
cfg_t cfg;

// Parameter list
static const cfg_param_list_t param_list[] = {
    #define MF_PARAM(name, defval, datatype, type, ...) {#name, defval, type, #__VA_ARGS__},
    MF_PARAM_LIST
    #undef MF_PARAM
};

// Constants
#define CFG_HDR0 'm'
#define CFG_HDR1 'a'
#define CFG_HDR2 'd'
#define CFG_HDR3 '2'

// Initialize configuration
void cfg_init(void) {
    //hal_eeprom_begin();
	eeprom_init();
}

// Get number of parameters
uint16_t cfg_param_count(void) {
    return CFG_PARAM_COUNT;
}

// 16-bit CCITT CRC calculation
uint16_t tbx_crc16(const uint8_t *buf, uint32_t len, uint16_t crc) { //initial value = 0xFFFF
  for (uint32_t i = 0; i < len; i++) {
    crc ^= buf[i] << 8;
    for (uint8_t bit = 0; bit < 8; bit++) {
      if ( crc & (1 << 15) ) {
          crc = (crc << 1) ^ 0x1021;
      }else{
          crc <<= 1;
      }
    }
  }
  return crc;
}

uint32_t tbx_crc32(const uint8_t *buf, uint32_t len, uint32_t crc) { //initial value = 0xFFFFFFFF
   for(uint32_t i=0; i<len; i++) {
      uint32_t byte = buf[i];
      crc = crc ^ byte;
      for (int j = 7; j >= 0; j--) {
         uint32_t mask = -(crc & 1);
         crc = (crc >> 1) ^ (0xEDB88320 & mask);
      }
      i = i + 1;
   }
   return ~crc;
}


// Get parameter name and value for index
bool cfg_get_name_and_value(uint16_t index, const char** name, float* value) {
    if(index >= cfg_param_count()) return false;
    *name = param_list[index].name;
    if(param_list[index].type == 'f') {
        *value = cfg.param.param_float[index];
    } else {
        int32_t v = cfg.param.param_int32_t[index];
        *value = v;
    }
    return true;
}

// Get parameter value as float
float cfg_get_value_str(const char* namestr, float default_value) {
    int i = cfg_get_index(namestr);
    if(i < 0) return default_value;
    return cfg_get_value_idx(i);
}

// Get parameter value as float
float cfg_get_value_idx(int i) {
    if(i < 0 || i > cfg_param_count()) return 0;
    if(param_list[i].type == 'f') {
        return cfg.param.param_float[i];
    } else {
        return cfg.param.param_int32_t[i];
    }
}

// Print enum option name for parameter pointer
void cfg_print_param_option(const int32_t* param) {
    uint32_t i = ((uint32_t)param - (uint32_t)(cfg.param.param_int32_t))  / 4;
    if(i < cfg_param_count()) {
        char option[20];
        if(cfg_get_option_string(i, *param, option)) {
            //printf("%s", option);
            return;
        }
    }
    //printf("***INVALID***");
}

// Get enum option name for param_idx and param_val
bool cfg_get_option_string(uint16_t param_idx, int32_t param_val, char out_option[20]) {
    out_option[0] = 0;
    if(param_idx >= cfg_param_count()) return false;
    if(param_val < 0) return false;
    const char *option = param_list[param_idx].options;
    int opt_i = 0;
    int pos = 0;
    // Skip i-1 commas
    while(option[pos] && opt_i < (int)param_val) {
        if(option[pos] == ',') opt_i++;
        pos++;
    }
    if(!option[pos]) return false;
    int out_i = 0;
    while(option[pos] && option[pos] != ',' && out_i < 20-1) {
        out_option[out_i] = option[pos];
        pos++;
        out_i++;
    }
    out_option[out_i] = 0;
    // Remove "mf_" prefix
    if(strncmp(out_option, "mf_", 3) == 0) strcpy(out_option, out_option + 3);
    return true;
}

// Print all parameters for module_name on single line
void cfg_print_module(const char* module_name) {
    char n[32];
    strncpy(n, module_name, sizeof(n));
    for(int i = 0; n[i]; i++) n[i] = toupper(n[i]);
    //printf("%s: ", n);

    for(int i = 0; n[i]; i++) n[i] = tolower(n[i]);
    strcat(n, "_");

    char type_name[40];
    snprintf(type_name, sizeof(type_name), "%sgizmo", n);
    int type_i = cfg_get_index(type_name);
    if(type_i >= 0) {
        cfg_print_value(type_i);
        //printf(" - ");
    }

    for(int i = 0; i < cfg_param_count(); i++) {
        if(strncmp(param_list[i].name, n, strlen(n)) == 0 && i != type_i) {
            //printf("%s:", param_list[i].name + strlen(n));
            cfg_print_value(i);
            //printf(" ");
        }
    }
    //printf("\n");
}

// Print "<name> <value> # options: <options>" for given param_index
void cfg_print_name_and_value(uint16_t i, const char* comment) {
    if(i > cfg_param_count()) return;
    //printf("%-16s ", param_list[i].name);
    cfg_print_value(i);
    //if(comment) printf(" # %s", comment);
    const char *options = param_list[i].options;
    if(options && options[0] != 0) {
        //printf(" # options: ");
        cfg_print_options(options);
    }
    //printf("\n");
}

// Print param value
void cfg_print_value(uint16_t i) {
    if(i >= cfg_param_count()) return;
    switch(param_list[i].type) {
        case 'e': { //enum
            char option[20];
            if(cfg_get_option_string(i, cfg.param.param_int32_t[i],  option)) {
                //printf("%s", option);
            } else {
                //printf("%d", (int)cfg.param.param_int32_t[i]);
            }
            break;
        }
        case 'f': //float
            //printf("%f", cfg.param.param_float[i]);
            break;
        case 'i': //integer
            //printf("%d", (int)cfg.param.param_int32_t[i]);
            break;
        case 'p': //pinnumber/pinname
            //hal_print_pin_name(cfg.param.param_int32_t[i]);
            break;
        default:
            //printf("ERROR invalid type '%c'", param_list[i].type);
    }
}

// Comparison function for sorting
static int cfg_param_list_name_compare(const void *a, const void *b) {
    uint16_t i = *(uint16_t*)a;
    uint16_t j = *(uint16_t*)b;
    return strcmp(param_list[i].name, param_list[j].name);
}

// CLI dump: print all config values, sorted by name
void cfg_cli_dump(const char* filter) {
    uint16_t arr[CFG_PARAM_COUNT];
    for(int i = 0; i < CFG_PARAM_COUNT; i++) arr[i] = i;
    qsort(arr, CFG_PARAM_COUNT, sizeof(uint16_t), cfg_param_list_name_compare);
    for(int j = 0; j < CFG_PARAM_COUNT; j++) {
        uint16_t i = arr[j];
        if(strstr(param_list[i].name, filter)) {
            cfg_print_name_and_value(i, NULL);
        }
    }
}

// CLI diff: print all modified config values, sorted by name
void cfg_cli_diff(const char* filter) {
    //cfg_t cfgdefault;
    uint16_t arr[CFG_PARAM_COUNT];
    for(int i = 0; i < CFG_PARAM_COUNT; i++) arr[i] = i;
    qsort(arr, CFG_PARAM_COUNT, sizeof(uint16_t), cfg_param_list_name_compare);
    for(int j = 0; j < CFG_PARAM_COUNT; j++) {
        uint16_t i = arr[j];
        if(strstr(param_list[i].name, filter) && cfg_get_value_idx(i) != cfg_get_value_idx(i)) {
            cfg_print_name_and_value(i, NULL);
        }
    }
}

// Print pins sorted by pin number
void cfg_print_pins() {
    for(int pinno = 0; pinno < 128; pinno++) {
        int cnt = 0;
        for(int i = 0; i < cfg_param_count(); i++) {
            if(strncmp(param_list[i].name, "pin_", 4) == 0 && cfg.param.param_int32_t[i]  == pinno) {
                //printf("PIN: ");
                if(cnt == 0) {
                    cfg_print_name_and_value(i, NULL);
                } else {
                    cfg_print_name_and_value(i, "WARNING: Duplicate pin assignment");
                }
                cnt++;
            }
        }
    }
}

// Set a parameter value, returns true on success
bool cfg_set_param(const char* namestr, const char* val) {
    char name[32], value[32];
    strncpy(name, namestr, sizeof(name));
    strncpy(value, val, sizeof(value));

    // Trim whitespace
    char *p = name;
    while(isspace((unsigned char)*p)) p++;
    char *end = p + strlen(p) - 1;
    while(end > p && isspace((unsigned char)*end)) end--;
    *(end + 1) = 0;

    p = value;
    while(isspace((unsigned char)*p)) p++;
    end = p + strlen(p) - 1;
    while(end > p && isspace((unsigned char)*end)) end--;
    *(end + 1) = 0;

    if(name[0] == 0) return false;
    if(value[0] == 0) {
        //printf("CFG: WARNING - No value for param '%s'\n", name);
        return false;
    }

    int i = cfg_get_index(name);
    if(i < 0) {
        //printf("CFG: WARNING - Param '%s' not found\n", name);
        return false;
    }

    switch(param_list[i].type) {
        case 'e': { //enum
            int enum_idx = cfg_get_enum_index(value, param_list[i].options);
            if(enum_idx >= 0) {
                cfg.param.param_int32_t[i]  = enum_idx;
                return true;
            } else {
                //printf("CFG: WARNING - Param '%s' has no '%s' option. Available options: ", name, value);
                cfg_print_options(param_list[i].options);
                //printf("\n");
                return false;
            }
            break;
        }
        case 'f': //float
            cfg.param.param_float[i]  = atof(value);
            break;
        case 'i': //integer
            cfg.param.param_int32_t[i]  = atoi(value);
            break;
        case 'p': //pinnumber/pinname
            //cfg.param.param_int32_t[i]  = hal_get_pin_number(value);
            break;
    }
    return true;
}

// Set parameter value for MAVLink
bool cfg_set_param_mavlink(const char* namestr, float val) {
    char name[32];
    strncpy(name, namestr, sizeof(name));

    // Trim whitespace
    char *p = name;
    while(isspace((unsigned char)*p)) p++;
    char *end = p + strlen(p) - 1;
    while(end > p && isspace((unsigned char)*end)) end--;
    *(end + 1) = 0;

    if(name[0] == 0) return false;

    int i = cfg_get_index(name);
    if(i < 0) return false;

    if(param_list[i].type == 'f') {
        cfg.param.param_float[i]  = val;
    } else {
        cfg.param.param_int32_t[i]  = val;
    }
    return true;
}

// Get parameter index for a parameter name
int cfg_get_index(const char* namestr) {
    char name[32];
    strncpy(name, namestr, sizeof(name));

    // Convert to lowercase
    for(char *p = name; *p; p++) *p = tolower(*p);

    for(uint16_t i = 0; i < cfg_param_count(); i++) {
        if(strcmp(param_list[i].name, name) == 0) {
            return i;
        }
    }
    return -1;
}

// Load defaults
void cfg_clear(void) {
    memset(&cfg, 0, sizeof(cfg_t));
    cfg.hdr.header0  = CFG_HDR0;
    cfg.hdr.header1  = CFG_HDR1;
    cfg.hdr.header2  = CFG_HDR2;
    cfg.hdr.header3  = CFG_HDR3;
    cfg.hdr.len  = 0;
    cfg.hdr._reserved0  = 0;
    cfg.hdr.madflight_param_crc  = 0;
    memset(cfg.hdr._reserved1,  0, sizeof(cfg.hdr._reserved1));

    // Set default values
    for(int i = 0; i < cfg_param_count(); i++) {
        if(param_list[i].type == 'f') {
            cfg.param.param_float[i]  = param_list[i].defval;
        } else {
            cfg.param.param_int32_t[i]  = param_list[i].defval;
        }
    }
}

// Read parameters from EEPROM/flash
bool cfg_load_from_eeprom(void) {
    //printf("CFG: Loading EEPROM - ");

    cfg_clear();

    // Load header into buffer
    cfg_header_t hdr_new;
    uint8_t *buf = (uint8_t*)&hdr_new;
    /*
    for(uint32_t i = 0; i < sizeof(cfg_header_t); i++) {
        //buf[i] = hal_eeprom_read(i);
    	buf[i] = eeprom_read_byte(i);
    }
    */
    if (!eeprom_read(0, buf, sizeof(cfg_header_t)))
    {
    	LogE("cfg_load_from_eeprom false");
    	return false;
    }

    // Check header
    if(hdr_new.header0  != CFG_HDR0
    || hdr_new.header1  != CFG_HDR1
    || hdr_new.header2  != CFG_HDR2
    || hdr_new.header3  != CFG_HDR3
    || hdr_new.len  < sizeof(cfg_header_t) + 8
    || hdr_new.len  > 4096) {
        //printf("Header invalid, using defaults\n");
    	LogE("cfg_load_from_eeprom Header invalid, using defaults");
        return false;
    }

    uint32_t datalen = hdr_new.len  - 4; // Length of header+param (4=crc)
    uint32_t paramlen = datalen - sizeof(cfg_header_t); // Length of param

    // Check CRC
    uint32_t crc = 0xFFFFFFFF;


    cfg_param_t param;

    if (!eeprom_read(sizeof(cfg_header_t), (uint8_t*)&param, paramlen))
    {
    	LogE("cfg_load_from_eeprom eeprom_read false");
    	return false;
    }

    for(uint32_t i = 0; i < sizeof(cfg_header_t); i++) {
        //uint8_t byte = hal_eeprom_read(i);
    	uint8_t byte = ((uint8_t*)&hdr_new)[i];
        crc = tbx_crc32(&byte, 1, crc);
    }
    for(uint32_t i = 0; i < sizeof(cfg_param_t); i++) {
        //uint8_t byte = hal_eeprom_read(i);
    	uint8_t byte = ((uint8_t*)&param)[i];
        crc = tbx_crc32(&byte, 1, crc);
    }

    uint32_t crc_eeprom;
    uint8_t *crc_eeprom_buf = (uint8_t*)&crc_eeprom;

    for(uint32_t i = 0; i < 4; i++) { // 4=crc
        //crc_eeprom_buf[i] = hal_eeprom_read(datalen + i);
    	crc_eeprom_buf[i] = eeprom_read_byte(datalen + i);
    }

    if(crc != crc_eeprom) {
        //printf("CRC invalid, using defaults\n");
    	LogE("cfg_load_from_eeprom CRC invalid, using defaults");
        return false;
    }

    memcpy(&cfg.hdr,  &hdr_new, sizeof(cfg_header_t));
    memcpy(&cfg.param,  &param, sizeof(cfg_param_t));

    // Load header from EEPROM
    //memcpy(&cfg.hdr,  &hdr_new, sizeof(cfg_header_t));

    // Load param from EEPROM
    //uint8_t *param_buf = (uint8_t*)&cfg.param;
    //uint32_t num_bytes = sizeof(cfg_param_t);
    //if(num_bytes > paramlen) num_bytes = paramlen; // Minimum of CfgParam and EEPROM bytes
    /*
    for(uint32_t i = 0; i < num_bytes; i++) {
        //param_buf[i] = hal_eeprom_read(sizeof(cfg_header_t) + i);
    	param_buf[i] = eeprom_read_byte(sizeof(cfg_header_t) + i);
    }
	*/

    LogI("cfg_load_from_eeprom OK");
    //printf("OK\n");
    return true;
}

bool cfg_write_to_eeprom(void) {
    uint32_t pos = 0;
    uint32_t crc = 0xFFFFFFFF;

    // Setup header
    cfg.hdr.header0  = CFG_HDR0;
    cfg.hdr.header1  = CFG_HDR1;
    cfg.hdr.header2  = CFG_HDR2;
    cfg.hdr.header3  = CFG_HDR3;
    cfg.hdr.len  = sizeof(cfg_header_t) + sizeof(cfg_param_t) + 4; // header + params + crc
    cfg.hdr._reserved0  = 0;

    // Write header
    if (!eeprom_write(pos, (uint8_t*)&cfg.hdr, sizeof(cfg_header_t)))
    {
    	LogE("cfg_write_to_eeprom Write header false");
    	return false;
    }

    for(uint32_t i = 0; i < sizeof(cfg_header_t); i++) {
        uint8_t byte = ((uint8_t*)&cfg.hdr)[i];
        //hal_eeprom_write(pos, byte);
        //eeprom_write_byte(pos, byte);
        crc = tbx_crc32(&byte, 1, crc);
        pos++;
    }

    // Write parameters - only write the actual used portion
    uint32_t params_to_write = cfg_param_count();
    if(params_to_write > CFG_PARAM_COUNT) {
        params_to_write = CFG_PARAM_COUNT;
    }

    // Calculate actual size to write
    size_t param_bytes_to_write = params_to_write * (
        sizeof(float) > sizeof(int32_t) ? sizeof(float) : sizeof(int32_t)
    );

    // Write parameter data
    uint8_t* param_ptr = (uint8_t*)&cfg.param;
    if (!eeprom_write(pos, param_ptr, param_bytes_to_write))
    {
    	LogE("cfg_write_to_eeprom Write parameter data false");
    	return false;
    }

    for(uint32_t i = 0; i < param_bytes_to_write; i++) {
        uint8_t byte = param_ptr[i];
        //hal_eeprom_write(pos, byte);
        //eeprom_write_byte(pos, byte);
        crc = tbx_crc32(&byte, 1, crc);
        pos++;
    }

    // Write CRC
    for(uint32_t i = 0; i < sizeof(crc); i++) {
        //hal_eeprom_write(pos, ((uint8_t*)&crc)[i]);
    	if (!eeprom_write_byte(pos, ((uint8_t*)&crc)[i]))
    	{
    		LogE("cfg_write_to_eeprom Write CRC false");
    		return false;
    	}
        pos++;
    }

    //hal_eeprom_commit();
    //printf("CFG: EEPROM written (%u params, %u bytes)\n", params_to_write, cfg.hdr.len);
    LogI("CFG: EEPROM written (%u params, %u bytes)\n", params_to_write, (uint32_t)cfg.hdr.len);
    return true;
}

// 加载字符串形式的配置
void cfg_load_from_string(const char *batch) {
    int pos = 0;
    int lineno = 0;
    char cmdline[256] = "";
    int cmdline_pos = 0;

    while(1) {
        char c = batch[pos];
        if(c == '\r' || c == '\n' || c == 0) { // 行结束或字符串结束
            lineno++;
            cmdline[cmdline_pos] = 0;
            if(!cfg_load_cmdline(cmdline))
            {
            	(void)(lineno);
                //printf("Error processing line %d: %s\n", lineno, cmdline);
            }
            cmdline_pos = 0;
            cmdline[0] = 0;
            if(c == 0) return;
        } else {
            if(cmdline_pos < sizeof(cmdline)-1) {
                cmdline[cmdline_pos++] = c;
            }
        }
        pos++;
    }
}

// 加载板级和配置参数
void cfg_load_madflight(const char *board, const char *config) {
    //printf("Loading configuration - ");

    // 计算CRC校验值
    uint32_t crc = 0xFFFFFFFF;
    if(board && board[0]) crc = tbx_crc32((const uint8_t*)board, strlen(board), crc);
    if(config && config[0]) crc = tbx_crc32((const uint8_t*)config, strlen(config), crc);

    // 检查CRC是否匹配
    if(cfg.hdr.madflight_param_crc  == crc) {
        //printf("Skipping (EEPROM is newer)\n");
        return;
    }

    // 加载板级配置
    if(board && board[0]) {
        cfg_load_from_string(board);
        //printf("Board config OK, ");
    } else {
        //printf("No board config, ");
    }

    // 加载应用配置
    if(config && config[0]) {
        cfg_load_from_string(config);
        //printf("App config OK\n");
    } else {
        //printf("No app config\n");
    }

    // 更新CRC
    cfg.hdr.madflight_param_crc  = crc;
}

// 处理单条配置命令
bool cfg_load_cmdline(const char *cmdline) {
    char line[256];
    strncpy(line, cmdline, sizeof(line));

    // 移除注释
    char *comment_pos = strchr(line, '#');
    if(comment_pos) *comment_pos = 0;
    comment_pos = strchr(line, '/');
    if(comment_pos) *comment_pos = 0;

    // 分割参数名和值
    char *space_pos = strchr(line, ' ');
    if(!space_pos) {
        if(strlen(line) > 0) {
            //printf("Missing value for: %s\n", line);
            return false;
        }
        return true;
    }

    *space_pos = 0;
    char *name = line;
    char *value = space_pos + 1;

    // 去除首尾空格
    while(isspace((unsigned char)*name)) name++;
    char *end = name + strlen(name) - 1;
    while(end > name && isspace((unsigned char)*end)) end--;
    *(end + 1) = 0;

    while(isspace((unsigned char)*value)) value++;
    end = value + strlen(value) - 1;
    while(end > value && isspace((unsigned char)*end)) end--;
    *(end + 1) = 0;

    // 设置参数
    if(name[0] != 0) {
        return cfg_set_param(name, value);
    }
    return true;
}

// 获取枚举值的索引
int cfg_get_enum_index(const char* key, const char* options) {
    char skey[64];
    snprintf(skey, sizeof(skey), "mf_%s", key);

    // 转换为大写
    for(char *p = skey; *p; p++) *p = toupper(*p);

    const char *k = skey;
    int klen = strlen(k);
    int len = strlen(options);
    int pos = 0;
    int i = 0;

    // 在选项中查找匹配项
    while(pos < len) {
        if(strncmp(options + pos, k, klen) == 0 &&
           (pos + klen >= len || options[pos + klen] == ',')) {
            return i;
        }
        i++;
        while(pos < len && options[pos] != ',') pos++;
        pos++; // 跳过逗号
    }
    return -1;
}

// 打印选项（去除mf_前缀）
void cfg_print_options(const char *str) {
    const char *p = str;
    const char *p2;

    while(*p) {
        p2 = strstr(p, "mf_");
        if(!p2) {
            //printf("%s", p);
            return;
        }

        // 打印"mf_"之前的内容
        for(const char *c = p; c < p2; c++) putchar(*c);

        p = p2 + 3; // 跳过"mf_"
    }
}

