#include "io_interface.hpp"
#include <yaml-cpp/yaml.h>
#include <stdio.h>
#include <errno.h>
#include <modbus/modbus.h>
#include <cstring>
#include <time.h>
#include <mutex>

// ===== 运行时配置变量定义（由 io_config.yaml 加载）=====
bool g_di_module_enabled = true;
bool g_do_module_enabled = true;
bool g_io_integrated_mode = true;
int  g_modbus_port = 502;
int  g_modbus_slave_id = 1;
int  g_di_base_address = 500;
int  g_do_base_address = 800;
std::string g_di_device_ip = "192.168.1.84";
std::string g_do_device_ip = "192.168.1.84";

// DI/DO 信号地址数组（按结构体字段顺序，由YAML加载填充）
int g_di_signal_addresses[DI_SIGNAL_COUNT];
int g_do_signal_addresses[DO_SIGNAL_COUNT];

// Modbus 连接（所有模式变量始终存在，运行时按需使用）
static modbus_t *ctx_io = NULL;  // 集成模式连接
static modbus_t *ctx_di = NULL;  // 分立模式 DI 连接
static modbus_t *ctx_do = NULL;  // 分立模式 DO 连接

// Modbus 访问互斥锁
static std::mutex modbus_io_mutex;
static std::mutex modbus_di_mutex;
static std::mutex modbus_do_mutex;

static DO_Interface current_do_state = {0};

// ===== 内部辅助函数 =====

// 获取当前 DI 对应的 Modbus 连接
static inline modbus_t* get_di_ctx() {
    return g_io_integrated_mode ? ctx_io : ctx_di;
}

// 获取当前 DO 对应的 Modbus 连接
static inline modbus_t* get_do_ctx() {
    return g_io_integrated_mode ? ctx_io : ctx_do;
}

// 获取 DI 对应的互斥锁
static inline std::mutex& get_di_mutex() {
    return g_io_integrated_mode ? modbus_io_mutex : modbus_di_mutex;
}

// 获取 DO 对应的互斥锁
static inline std::mutex& get_do_mutex() {
    return g_io_integrated_mode ? modbus_io_mutex : modbus_do_mutex;
}

// ===== IO 配置加载 =====
bool load_io_config(const std::string &yaml_path) {
    YAML::Node config;
    try {
        config = YAML::LoadFile(yaml_path);
    } catch (const YAML::Exception &e) {
        fprintf(stderr, "无法加载IO配置文件: %s, 错误: %s\n", yaml_path.c_str(), e.what());
        return false;
    }

    // 解析 Modbus 配置
    if (config["modbus"]) {
        auto m = config["modbus"];
        if (m["integrated_mode"])      g_io_integrated_mode = m["integrated_mode"].as<bool>();
        if (m["port"])                 g_modbus_port = m["port"].as<int>();
        if (m["slave_id"])             g_modbus_slave_id = m["slave_id"].as<int>();
        if (m["di_device_ip"])         g_di_device_ip = m["di_device_ip"].as<std::string>();
        if (m["do_device_ip"])         g_do_device_ip = m["do_device_ip"].as<std::string>();
        if (m["integrated_device_ip"]) g_di_device_ip = g_do_device_ip = m["integrated_device_ip"].as<std::string>();
    }

    // 解析 DI 模块配置
    if (config["di_module"]) {
        auto dm = config["di_module"];
        if (dm["enabled"])      g_di_module_enabled = dm["enabled"].as<bool>();
        if (dm["base_address"]) g_di_base_address = dm["base_address"].as<int>();
    }

    // 解析 DO 模块配置
    if (config["do_module"]) {
        auto dm = config["do_module"];
        if (dm["enabled"])      g_do_module_enabled = dm["enabled"].as<bool>();
        if (dm["base_address"]) g_do_base_address = dm["base_address"].as<int>();
    }

    // 解析 DI 信号地址表
    if (config["di_signals"]) {
        auto signals = config["di_signals"];
        if (signals.size() < (size_t)DI_SIGNAL_COUNT) {
            fprintf(stderr, "DI信号数量不足: yaml有%zu个, 需要%d个\n", signals.size(), DI_SIGNAL_COUNT);
            return false;
        }
        for (size_t i = 0; i < (size_t)DI_SIGNAL_COUNT; i++) {
            g_di_signal_addresses[i] = signals[i]["address"].as<int>();
        }
    } else {
        fprintf(stderr, "YAML中未找到 'di_signals' 节点\n");
        return false;
    }

    // 解析 DO 信号地址表
    if (config["do_signals"]) {
        auto signals = config["do_signals"];
        if (signals.size() < (size_t)DO_SIGNAL_COUNT) {
            fprintf(stderr, "DO信号数量不足: yaml有%zu个, 需要%d个\n", signals.size(), DO_SIGNAL_COUNT);
            return false;
        }
        for (size_t i = 0; i < (size_t)DO_SIGNAL_COUNT; i++) {
            g_do_signal_addresses[i] = signals[i]["address"].as<int>();
        }
    } else {
        fprintf(stderr, "YAML中未找到 'do_signals' 节点\n");
        return false;
    }

    printf("IO配置加载成功:\n");
    printf("  集成模式: %s\n", g_io_integrated_mode ? "是" : "否");
    printf("  端口: %d, 从站ID: %d\n", g_modbus_port, g_modbus_slave_id);
    printf("  DI: %s (base=M%d, %d信号, IP=%s)\n",
           g_di_module_enabled ? "启用" : "禁用", g_di_base_address, DI_SIGNAL_COUNT, g_di_device_ip.c_str());
    printf("  DO: %s (base=M%d, %d信号, IP=%s)\n",
           g_do_module_enabled ? "启用" : "禁用", g_do_base_address, DO_SIGNAL_COUNT, g_do_device_ip.c_str());

    return true;
}

// ===== 从设备刷新 DO 状态（内部函数）=====
static void refresh_do_state_from_device() {
    std::lock_guard<std::mutex> lock(get_do_mutex());
    modbus_t *ctx = get_do_ctx();
    if (!ctx) return;

    uint8_t do_values[16];
    if (modbus_read_bits(ctx, 0, 16, do_values) == 16) {
        current_do_state.start_button_light  = do_values[0];   // M800 语义名
        current_do_state.reset_button_light  = do_values[1];   // M801
        current_do_state.pause_button_light  = do_values[2];   // M802
        current_do_state.buzzer              = do_values[3];   // M803
        current_do_state.red_light           = do_values[4];   // M804
        current_do_state.yellow_light        = do_values[5];   // M805
        current_do_state.green_light         = do_values[6];   // M806
        current_do_state.do_M807             = do_values[7];   // M807 通用
        current_do_state.do_M808             = do_values[8];   // M808
        current_do_state.do_M809             = do_values[9];   // M809
        current_do_state.do_M810             = do_values[10];  // M810
        current_do_state.do_M811             = do_values[11];  // M811
        current_do_state.do_M812             = do_values[12];  // M812
        current_do_state.do_M813             = do_values[13];  // M813
        current_do_state.do_M814             = do_values[14];  // M814
        current_do_state.do_M815             = do_values[15];  // M815
    } else {
        fprintf(stderr, "刷新DO状态失败: %s\n", modbus_strerror(errno));
    }
}

// ===== Modbus 初始化 =====
int init_modbus_interface(const char* di_ip, int di_port, int di_slave_id,
                         const char* do_ip, int do_port, int do_slave_id) {
    int result = 0;

    if (g_io_integrated_mode) {
        // ===== 集成模式：单设备单连接 =====
        (void)do_ip; (void)do_port; (void)do_slave_id;

        if (di_ip != NULL) {
            ctx_io = modbus_new_tcp(di_ip, di_port);
            if (ctx_io == NULL) {
                fprintf(stderr, "无法创建 IO Modbus 上下文\n");
                result = -1;
            } else {
                modbus_set_response_timeout(ctx_io, 1, 0);
                modbus_set_slave(ctx_io, di_slave_id);

                if (modbus_connect(ctx_io) == -1) {
                    fprintf(stderr, "IO设备连接失败: %s\n", modbus_strerror(errno));
                    modbus_free(ctx_io);
                    ctx_io = NULL;
                    result = -1;
                } else {
                    printf("IO设备连接成功: %s:%d (从站ID: %d)\n", di_ip, di_port, di_slave_id);
                    if (g_do_module_enabled) {
                        refresh_do_state_from_device();
                    }
                }
            }
        }

        if (result == 0) {
            printf("Modbus连接初始化完成 (集成模式)\n");
        }
    } else {
        // ===== 分立模式：双设备双连接 =====

        // 初始化 DI 连接
        if (g_di_module_enabled && di_ip != NULL) {
            ctx_di = modbus_new_tcp(di_ip, di_port);
            if (ctx_di == NULL) {
                fprintf(stderr, "无法创建 DI Modbus 上下文\n");
                result = -1;
            } else {
                modbus_set_response_timeout(ctx_di, 1, 0);
                modbus_set_slave(ctx_di, di_slave_id);

                if (modbus_connect(ctx_di) == -1) {
                    fprintf(stderr, "DI连接失败: %s\n", modbus_strerror(errno));
                    modbus_free(ctx_di);
                    ctx_di = NULL;
                    result = -1;
                } else {
                    printf("DI设备连接成功: %s:%d (从站ID: %d)\n", di_ip, di_port, di_slave_id);
                }
            }
        } else {
            printf("DI模块已禁用，跳过DI设备连接\n");
        }

        // 初始化 DO 连接
        if (g_do_module_enabled && do_ip != NULL && result == 0) {
            ctx_do = modbus_new_tcp(do_ip, do_port);
            if (ctx_do == NULL) {
                fprintf(stderr, "无法创建 DO Modbus 上下文\n");
                result = -1;
            } else {
                modbus_set_response_timeout(ctx_do, 1, 0);
                modbus_set_slave(ctx_do, do_slave_id);

                if (modbus_connect(ctx_do) == -1) {
                    fprintf(stderr, "DO连接失败: %s\n", modbus_strerror(errno));
                    modbus_free(ctx_do);
                    ctx_do = NULL;
                    result = -1;
                } else {
                    printf("DO设备连接成功: %s:%d (从站ID: %d)\n", do_ip, do_port, do_slave_id);
                    refresh_do_state_from_device();
                }
            }
        } else {
            printf("DO模块已禁用，跳过DO设备连接\n");
        }

        if (result == 0) {
            printf("Modbus连接初始化完成\n");
        }
    }

    return result;
}

// ===== Modbus 清理 =====
void cleanup_modbus_interface() {
    if (ctx_io) {
        modbus_close(ctx_io);
        modbus_free(ctx_io);
        ctx_io = NULL;
        printf("IO连接已关闭\n");
    }
    if (ctx_di) {
        modbus_close(ctx_di);
        modbus_free(ctx_di);
        ctx_di = NULL;
        printf("DI连接已关闭\n");
    }
    if (ctx_do) {
        modbus_close(ctx_do);
        modbus_free(ctx_do);
        ctx_do = NULL;
        printf("DO连接已关闭\n");
    }
}

// ===== DI 信号读取 =====
DI_Interface read_all_di_signals() {
    DI_Interface di = {0};

    if (!g_di_module_enabled) return di;

    std::lock_guard<std::mutex> lock(get_di_mutex());
    modbus_t *ctx = get_di_ctx();
    if (!ctx) {
        fprintf(stderr, "DI Modbus 未初始化\n");
        return di;
    }

    uint8_t di_values[DI_SIGNAL_COUNT];
    int rc = modbus_read_input_bits(ctx, 0, DI_SIGNAL_COUNT, di_values);

    if (rc == -1) {
        static int error_count = 0;
        if (error_count++ % 100 == 0) {
            fprintf(stderr, "读取DI失败: %s (已抑制%d次重复错误)\n",
                    modbus_strerror(errno), error_count);
        }
        return di;
    } else {
        static int error_count = 0;
        error_count = 0;
    }

    // 映射到结构体（按字段顺序，索引即偏移量，地址由YAML配置决定）
    di.start_button            = di_values[0];   // M500 语义名
    di.reset_button            = di_values[1];   // M501
    di.pause_button            = di_values[2];   // M502
    di.manual_auto_button      = di_values[3];   // M503
    di.emergency_stop          = di_values[4];   // M504
    di.air_supply              = di_values[5];   // M505
    di.di_M506                 = di_values[6];   // M506 通用
    di.di_M507                 = di_values[7];   // M507
    di.di_M508                 = di_values[8];   // M508
    di.di_M509                 = di_values[9];   // M509
    di.di_M510                 = di_values[10];  // M510
    di.di_M511                 = di_values[11];  // M511
    di.di_M512                 = di_values[12];  // M512
    di.di_M513                 = di_values[13];  // M513
    di.di_M514                 = di_values[14];  // M514
    di.di_M515                 = di_values[15];  // M515

    return di;
}

bool read_single_di_signal(int di_address) {
    if (!g_di_module_enabled) return false;

    std::lock_guard<std::mutex> lock(get_di_mutex());
    modbus_t *ctx = get_di_ctx();
    if (!ctx) return false;

    // 验证地址是否在配置的信号地址表中
    bool valid = false;
    for (int i = 0; i < DI_SIGNAL_COUNT; i++) {
        if (g_di_signal_addresses[i] == di_address) { valid = true; break; }
    }
    if (!valid) {
        fprintf(stderr, "DI地址不在配置范围内: M%d\n", di_address);
        return false;
    }

    uint8_t value;
    int rc = modbus_read_input_bits(ctx, di_address - g_di_base_address, 1, &value);
    return (rc == 1) ? value : false;
}

// ===== DO 信号写入 =====
int write_do_signals(DO_Interface do_signals) {
    if (!g_do_module_enabled) {
        fprintf(stderr, "警告: DO模块已禁用，无法写入信号\n");
        return -1;
    }

    std::lock_guard<std::mutex> lock(get_do_mutex());
    modbus_t *ctx = get_do_ctx();
    if (!ctx) return -1;

    // 先读取当前所有DO状态，避免清除其他位
    uint8_t do_values[16];
    if (modbus_read_bits(ctx, 0, 16, do_values) != 16) {
        fprintf(stderr, "读取当前DO状态失败\n");
        return -1;
    }

    // 设置DO值（偏移量 = 地址 - g_do_base_address）
    do_values[0]  = do_signals.start_button_light;   // M800
    do_values[1]  = do_signals.reset_button_light;   // M801
    do_values[2]  = do_signals.pause_button_light;   // M802
    do_values[3]  = do_signals.buzzer;               // M803
    do_values[4]  = do_signals.red_light;            // M804
    do_values[5]  = do_signals.yellow_light;         // M805
    do_values[6]  = do_signals.green_light;          // M806
    do_values[7]  = do_signals.do_M807;              // M807
    do_values[8]  = do_signals.do_M808;              // M808
    do_values[9]  = do_signals.do_M809;              // M809
    do_values[10] = do_signals.do_M810;              // M810
    do_values[11] = do_signals.do_M811;              // M811
    do_values[12] = do_signals.do_M812;              // M812
    do_values[13] = do_signals.do_M813;              // M813
    do_values[14] = do_signals.do_M814;              // M814
    do_values[15] = do_signals.do_M815;              // M815

    // 写入设备
    int result = modbus_write_bits(ctx, 0, 16, do_values);

    // 写入后刷新状态缓存
    if (result == 16) {
        refresh_do_state_from_device();
    }

    return result;
}

int write_single_do_signal(int do_address, bool state) {
    if (!g_do_module_enabled) {
        fprintf(stderr, "警告: DO模块已禁用，无法写入信号\n");
        return -1;
    }

    std::lock_guard<std::mutex> lock(get_do_mutex());
    modbus_t *ctx = get_do_ctx();
    if (!ctx) return -1;

    // 验证地址是否在配置的信号地址表中
    bool valid = false;
    for (int i = 0; i < DO_SIGNAL_COUNT; i++) {
        if (g_do_signal_addresses[i] == do_address) { valid = true; break; }
    }
    if (!valid) {
        fprintf(stderr, "DO地址不在配置范围内: M%d\n", do_address);
        return -1;
    }

    // 写入单个信号到设备
    int result = modbus_write_bit(ctx, do_address - g_do_base_address, state);

    // 写入成功后刷新DO状态缓存
    if (result == 1) {
        uint8_t do_values[16];
        if (modbus_read_bits(ctx, 0, 16, do_values) == 16) {
            current_do_state.start_button_light  = do_values[0];
            current_do_state.reset_button_light  = do_values[1];
            current_do_state.pause_button_light  = do_values[2];
            current_do_state.buzzer              = do_values[3];
            current_do_state.red_light           = do_values[4];
            current_do_state.yellow_light        = do_values[5];
            current_do_state.green_light         = do_values[6];
            current_do_state.do_M807             = do_values[7];
            current_do_state.do_M808             = do_values[8];
            current_do_state.do_M809             = do_values[9];
            current_do_state.do_M810             = do_values[10];
            current_do_state.do_M811             = do_values[11];
            current_do_state.do_M812             = do_values[12];
            current_do_state.do_M813             = do_values[13];
            current_do_state.do_M814             = do_values[14];
            current_do_state.do_M815             = do_values[15];
        }
    }

    return result;
}

// ===== DO 状态获取 =====
DO_Interface get_current_do_state() {
    return current_do_state;
}

void refresh_do_state() {
    if (g_do_module_enabled) {
        refresh_do_state_from_device();
    } else {
        printf("DO模块已禁用，无法刷新状态\n");
    }
}

// ===== 模块状态检查 =====
bool is_di_module_enabled() {
    return g_di_module_enabled;
}

bool is_do_module_enabled() {
    return g_do_module_enabled;
}

// ===== 非阻塞延时检查 =====
int should_execute_sequence(time_t *last_time, int interval_seconds) {
    time_t current_time = time(NULL);
    if (current_time - *last_time >= interval_seconds) {
        *last_time = current_time;
        return 1;
    }
    return 0;
}

// ===== DO 控制命令解析 =====
bool parse_do_control_command(const std::string& command, DOControlCommand& do_cmd) {
    // 格式: "801:1" 或 "801:0"
    size_t colon_pos = command.find(':');
    if (colon_pos == std::string::npos) {
        fprintf(stderr, "无效的DO控制命令格式，应为 '地址:状态'\n");
        return false;
    }

    std::string addr_str = command.substr(0, colon_pos);
    std::string state_str = command.substr(colon_pos + 1);

    // 去除空格
    addr_str.erase(0, addr_str.find_first_not_of(" \t"));
    addr_str.erase(addr_str.find_last_not_of(" \t") + 1);
    state_str.erase(0, state_str.find_first_not_of(" \t"));
    state_str.erase(state_str.find_last_not_of(" \t") + 1);

    // 验证地址是否在DO信号地址表中
    try {
        int address = std::stoi(addr_str);
        bool valid = false;
        for (int i = 0; i < DO_SIGNAL_COUNT; i++) {
            if (g_do_signal_addresses[i] == address) { valid = true; break; }
        }
        if (!valid) {
            fprintf(stderr, "DO地址不在配置范围内: M%d (范围M%d~M%d)\n",
                    address, g_do_base_address, g_do_base_address + DO_SIGNAL_COUNT - 1);
            return false;
        }
        do_cmd.do_address = addr_str;
    } catch (const std::exception& e) {
        fprintf(stderr, "DO地址解析失败: %s\n", e.what());
        return false;
    }

    // 验证状态有效性
    if (state_str == "1" || state_str == "true" || state_str == "True") {
        do_cmd.state = true;
    } else if (state_str == "0" || state_str == "false" || state_str == "False") {
        do_cmd.state = false;
    } else {
        fprintf(stderr, "无效的状态值: %s (应为 0/1 或 true/false)\n", state_str.c_str());
        return false;
    }

    return true;
}
