#include "io_interface.hpp"
#include <yaml-cpp/yaml.h>
#include <stdio.h>
#include <errno.h>
#include <modbus/modbus.h>
#include <cstring>
#include <exception>
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

// 有效点数（= 各自 yaml 的信号条数）
int g_di_count = 0;
int g_do_count = 0;

// 角色 → 位偏移（IO_ROLE_NONE 表示未配置）
int g_di_role_offset[DI_ROLE_COUNT];
int g_do_role_offset[DO_ROLE_COUNT];

// 被保护（不允许外部写）的 DO 位
bool g_do_protected[IO_MAX_SIGNALS];

bool g_emergency_stop_active_low = true;
int  g_reset_confirm_ms = 3000;

// IO 控模式下手自动按钮角色未配置时的默认模式（缺省手动，见头文件说明）
bool g_default_auto_mode = false;

int g_pause_reset_do[IO_MAX_SIGNALS];       int g_pause_reset_do_count = 0;
int g_shutdown_reset_do[IO_MAX_SIGNALS];    int g_shutdown_reset_do_count = 0;

// DI/DO 信号地址数组（下标 = 位偏移，由YAML加载填充）
int g_di_signal_addresses[IO_MAX_SIGNALS];
int g_do_signal_addresses[IO_MAX_SIGNALS];

// 配置是否已成功加载
static bool g_io_config_valid = false;

// ===== 角色名表与必需性表 =====
static const char *kDiRoleNames[DI_ROLE_COUNT] = {
    "emergency_stop", "start_button", "reset_button",
    "pause_button", "manual_auto_button"
};
static const bool kDiRoleRequired[DI_ROLE_COUNT] = {
    true, true, true, true, false
};
static const char *kDoRoleNames[DO_ROLE_COUNT] = {
    "buzzer", "red_light", "yellow_light", "green_light",
    "start_button_light", "reset_button_light", "pause_button_light"
};
static const bool kDoRoleRequired[DO_ROLE_COUNT] = {
    true, true, true, true, true, true, true
};

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

// ===== 角色名/取值辅助 =====
const char* di_role_name(DiRole role) {
    if (role < 0 || role >= DI_ROLE_COUNT) return "?";
    return kDiRoleNames[role];
}

const char* do_role_name(DoRole role) {
    if (role < 0 || role >= DO_ROLE_COUNT) return "?";
    return kDoRoleNames[role];
}

bool io_config_ok() {
    return g_io_config_valid;
}

// 从 current_do_state.raw[] 按角色表回填语义字段
static void apply_do_semantics_from_raw() {
    for (int r = 0; r < DO_ROLE_COUNT; r++) {
        int off = g_do_role_offset[r];
        bool v = (off >= 0 && off < current_do_state.count) ? current_do_state.raw[off] : false;
        switch (r) {
            case DO_ROLE_BUZZER:             current_do_state.buzzer             = v; break;
            case DO_ROLE_RED_LIGHT:          current_do_state.red_light          = v; break;
            case DO_ROLE_YELLOW_LIGHT:       current_do_state.yellow_light       = v; break;
            case DO_ROLE_GREEN_LIGHT:        current_do_state.green_light        = v; break;
            case DO_ROLE_START_BUTTON_LIGHT: current_do_state.start_button_light = v; break;
            case DO_ROLE_RESET_BUTTON_LIGHT: current_do_state.reset_button_light = v; break;
            case DO_ROLE_PAUSE_BUTTON_LIGHT: current_do_state.pause_button_light = v; break;
            default: break;
        }
    }
}

// 读取某个 DI 角色的电平（语义统一为：急停 true = 正常）
bool di_role_value(const DI_Interface &di, DiRole role) {
    if (role < 0 || role >= DI_ROLE_COUNT) return false;
    const int off = g_di_role_offset[role];
    if (off < 0 || off >= di.count || off >= IO_MAX_SIGNALS) {
        // 角色未配置（或超出本次读取范围）：急停按"正常"处理，其余按"未触发"处理
        return (role == DI_ROLE_EMERGENCY_STOP);
    }
    const bool raw = di.raw[off];
    if (role == DI_ROLE_EMERGENCY_STOP) {
        // 语义：true = 正常（未触发）
        // active_low 时物理高电平 = 正常；否则物理低电平 = 正常
        return g_emergency_stop_active_low ? raw : !raw;
    }
    return raw;
}

// ===== 配置解析辅助 =====

// 解析 roles 段：值必须是 [base, base+sig_count) 内的整数地址，且互不重复；
// 未知角色名 或 必需角色缺失 → 返回 false（拒绝启动）
static bool parse_roles(const YAML::Node &node, const char *const *names, int role_count,
                        const bool *required, int base, int sig_count,
                        int *out_offset, const char *tag) {
    for (int r = 0; r < role_count; r++) out_offset[r] = IO_ROLE_NONE;

    if (node) {
        if (!node.IsMap()) {
            fprintf(stderr, "[IO] %s 必须是映射(key: address)\n", tag);
            return false;
        }
        bool used[IO_MAX_SIGNALS] = {false};
        for (auto it = node.begin(); it != node.end(); ++it) {
            const std::string key = it->first.as<std::string>();
            int idx = -1;
            for (int r = 0; r < role_count; r++) {
                if (key == names[r]) { idx = r; break; }
            }
            if (idx < 0) {
                fprintf(stderr, "[IO] %s 出现未知角色名: '%s'\n", tag, key.c_str());
                return false;
            }
            int addr = 0;
            try {
                addr = it->second.as<int>();
            } catch (const std::exception &) {
                fprintf(stderr, "[IO] %s.%s 必须是整数地址\n", tag, key.c_str());
                return false;
            }
            const int off = addr - base;
            if (off < 0 || off >= sig_count) {
                fprintf(stderr, "[IO] %s.%s 地址 M%d 越界，应在 M%d ~ M%d\n",
                        tag, key.c_str(), addr, base, base + sig_count - 1);
                return false;
            }
            if (used[off]) {
                fprintf(stderr, "[IO] %s 地址 M%d 被重复绑定\n", tag, addr);
                return false;
            }
            used[off] = true;
            out_offset[idx] = off;
        }
    }

    for (int r = 0; r < role_count; r++) {
        if (required[r] && out_offset[r] == IO_ROLE_NONE) {
            fprintf(stderr, "[IO] %s 缺少必需角色 '%s'（该角色缺失会使对应安全/灯效功能失效，拒绝启动）\n",
                    tag, names[r]);
            return false;
        }
    }
    return true;
}

// 解析地址清单（缺省 not node → 合法，*out_count = 0，由调用方决定是否补默认）
static bool parse_addr_list(const YAML::Node &node, int base, int sig_count,
                            int *out, int *out_count, const char *tag) {
    *out_count = 0;
    if (!node) return true;
    if (!node.IsSequence()) {
        fprintf(stderr, "[IO] %s 必须是地址数组\n", tag);
        return false;
    }
    for (size_t i = 0; i < node.size(); i++) {
        int addr = 0;
        try {
            addr = node[i].as<int>();
        } catch (const std::exception &) {
            fprintf(stderr, "[IO] %s[%zu] 必须是整数地址\n", tag, i);
            return false;
        }
        if (addr < base || addr >= base + sig_count) {
            fprintf(stderr, "[IO] %s[%zu] 地址 M%d 越界，应在 M%d ~ M%d\n",
                    tag, i, addr, base, base + sig_count - 1);
            return false;
        }
        if (*out_count >= IO_MAX_SIGNALS) {
            fprintf(stderr, "[IO] %s 条目过多（上限 %d）\n", tag, IO_MAX_SIGNALS);
            return false;
        }
        out[(*out_count)++] = addr;
    }
    return true;
}

// ===== IO 配置加载 =====
bool load_io_config(const std::string &yaml_path) {
    // 每次加载前重置运行期配置，避免上一套点表（机型切换）的残留影响
    g_io_config_valid = false;
    g_di_count = 0;
    g_do_count = 0;
    for (int r = 0; r < DI_ROLE_COUNT; r++) g_di_role_offset[r] = IO_ROLE_NONE;
    for (int r = 0; r < DO_ROLE_COUNT; r++) g_do_role_offset[r] = IO_ROLE_NONE;
    for (int i = 0; i < IO_MAX_SIGNALS; i++) g_do_protected[i] = false;
    g_pause_reset_do_count = 0;
    g_shutdown_reset_do_count = 0;
    g_default_auto_mode = false;   // 机型切换时复位为缺省(手动)，由 di_module 段重新解析

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
        // integrated_device_ip 是"集成模式(单设备DI/DO共用IP)"专用字段，只在集成模式下生效。
        // 分立模式下若仍然赋值，会把上面已解析好的 DI/DO 两个 IP 全部覆盖成同一个地址，
        // 导致 di_device_ip / do_device_ip 形同虚设（历史问题：日志里 DI/DO 打印同一个 IP）。
        // 注：g_io_integrated_mode 已在本代码块首行解析完成，此处判断是安全的。
        if (m["integrated_device_ip"] && g_io_integrated_mode) {
            g_di_device_ip = g_do_device_ip = m["integrated_device_ip"].as<std::string>();
        }
    }

    // 解析 DI 模块配置
    if (config["di_module"]) {
        auto dm = config["di_module"];
        if (dm["enabled"])      g_di_module_enabled = dm["enabled"].as<bool>();
        if (dm["base_address"]) g_di_base_address = dm["base_address"].as<int>();
        if (dm["emergency_stop_active_low"]) {
            g_emergency_stop_active_low = dm["emergency_stop_active_low"].as<bool>();
        }
        // IO 控模式(CONTROL_SOURCE_IO=1)下、手自动按钮角色未配置时的默认模式。
        // 只允许 auto / manual，非法值按契约风格 fail-fast（避免静默用错默认值）。
        if (dm["default_operation_mode"]) {
            std::string mode;
            try {
                mode = dm["default_operation_mode"].as<std::string>();
            } catch (const std::exception &) {
                fprintf(stderr, "[IO] di_module.default_operation_mode 必须是字符串(auto|manual)\n");
                return false;
            }
            if (mode == "auto") {
                g_default_auto_mode = true;
            } else if (mode == "manual") {
                g_default_auto_mode = false;
            } else {
                fprintf(stderr, "[IO] di_module.default_operation_mode 非法: '%s'（只允许 auto / manual）\n",
                        mode.c_str());
                return false;
            }
        }
    }

    // 解析 DO 模块配置
    if (config["do_module"]) {
        auto dm = config["do_module"];
        if (dm["enabled"])      g_do_module_enabled = dm["enabled"].as<bool>();
        if (dm["base_address"]) g_do_base_address = dm["base_address"].as<int>();
        if (dm["reset_confirm_ms"]) g_reset_confirm_ms = dm["reset_confirm_ms"].as<int>();
    }

    // 解析 DI 信号地址表（契约 C1/C2：条数 1~IO_MAX_SIGNALS，且 address - base == 下标）
    if (config["di_signals"]) {
        auto signals = config["di_signals"];
        if (signals.size() == 0 || signals.size() > (size_t)IO_MAX_SIGNALS) {
            fprintf(stderr, "[IO] di_signals 条数非法: %zu（允许 1~%d）\n",
                    signals.size(), IO_MAX_SIGNALS);
            return false;
        }
        for (size_t i = 0; i < signals.size(); i++) {
            int addr = 0;
            try {
                addr = signals[i]["address"].as<int>();
            } catch (const std::exception &) {
                fprintf(stderr, "[IO] di_signals[%zu].address 缺失或非整数\n", i);
                return false;
            }
            if (addr - g_di_base_address != (int)i) {
                fprintf(stderr, "[IO] 契约C2 违反: di_signals[%zu].address=M%d，期望 M%d "
                                "（数组必须按 address 升序，使「下标 == address - base_address」）\n",
                        i, addr, g_di_base_address + (int)i);
                return false;
            }
            g_di_signal_addresses[i] = addr;
        }
        g_di_count = (int)signals.size();
    } else {
        fprintf(stderr, "YAML中未找到 'di_signals' 节点\n");
        return false;
    }

    // 解析 DO 信号地址表（同 C1/C2）
    if (config["do_signals"]) {
        auto signals = config["do_signals"];
        if (signals.size() == 0 || signals.size() > (size_t)IO_MAX_SIGNALS) {
            fprintf(stderr, "[IO] do_signals 条数非法: %zu（允许 1~%d）\n",
                    signals.size(), IO_MAX_SIGNALS);
            return false;
        }
        for (size_t i = 0; i < signals.size(); i++) {
            int addr = 0;
            try {
                addr = signals[i]["address"].as<int>();
            } catch (const std::exception &) {
                fprintf(stderr, "[IO] do_signals[%zu].address 缺失或非整数\n", i);
                return false;
            }
            if (addr - g_do_base_address != (int)i) {
                fprintf(stderr, "[IO] 契约C2 违反: do_signals[%zu].address=M%d，期望 M%d "
                                "（数组必须按 address 升序，使「下标 == address - base_address」）\n",
                        i, addr, g_do_base_address + (int)i);
                return false;
            }
            g_do_signal_addresses[i] = addr;
        }
        g_do_count = (int)signals.size();
    } else {
        fprintf(stderr, "YAML中未找到 'do_signals' 节点\n");
        return false;
    }

    // 解析角色表（契约 C3/C4）
    if (!parse_roles(config["di_roles"], kDiRoleNames, DI_ROLE_COUNT, kDiRoleRequired,
                     g_di_base_address, g_di_count, g_di_role_offset, "di_roles")) {
        return false;
    }
    if (!parse_roles(config["do_roles"], kDoRoleNames, DO_ROLE_COUNT, kDoRoleRequired,
                     g_do_base_address, g_do_count, g_do_role_offset, "do_roles")) {
        return false;
    }

    // 解析 safety 段
    {
        YAML::Node safety = config["safety"];
        if (safety) {
            if (!parse_addr_list(safety["pause_reset_do"], g_do_base_address, g_do_count,
                                 g_pause_reset_do, &g_pause_reset_do_count,
                                 "safety.pause_reset_do")) {
                return false;
            }
            if (!parse_addr_list(safety["shutdown_reset_do"], g_do_base_address, g_do_count,
                                 g_shutdown_reset_do, &g_shutdown_reset_do_count,
                                 "safety.shutdown_reset_do")) {
                return false;
            }
            int protect[IO_MAX_SIGNALS];
            int protect_count = 0;
            if (!parse_addr_list(safety["protected_do"], g_do_base_address, g_do_count,
                                 protect, &protect_count, "safety.protected_do")) {
                return false;
            }
            for (int i = 0; i < protect_count; i++) {
                g_do_protected[protect[i] - g_do_base_address] = true;
            }
        }
    }

    // shutdown_reset_do 缺省 = 复位全部（[base, base+count)）
    if (g_shutdown_reset_do_count == 0) {
        for (int i = 0; i < g_do_count; i++) {
            g_shutdown_reset_do[i] = g_do_base_address + i;
        }
        g_shutdown_reset_do_count = g_do_count;
    }

    // protected_do 缺省 = 三个按钮灯角色所在的位
    {
        bool any = false;
        for (int i = 0; i < IO_MAX_SIGNALS; i++) {
            if (g_do_protected[i]) { any = true; break; }
        }
        if (!any) {
            const DoRole lights[3] = {
                DO_ROLE_START_BUTTON_LIGHT, DO_ROLE_RESET_BUTTON_LIGHT, DO_ROLE_PAUSE_BUTTON_LIGHT
            };
            for (int k = 0; k < 3; k++) {
                const int off = g_do_role_offset[lights[k]];
                if (off >= 0 && off < IO_MAX_SIGNALS) {
                    g_do_protected[off] = true;
                }
            }
        }
    }

    // 缺省与摘要打印
    printf("IO配置加载成功 (文件: %s):\n", yaml_path.c_str());
    printf("  集成模式: %s | 端口: %d | 从站ID: %d\n",
           g_io_integrated_mode ? "是" : "否", g_modbus_port, g_modbus_slave_id);
    printf("  DI: %s (base=M%d, %d点, IP=%s)\n",
           g_di_module_enabled ? "启用" : "禁用", g_di_base_address,
           g_di_count, g_di_device_ip.c_str());
    printf("      ");
    for (int r = 0; r < DI_ROLE_COUNT; r++) {
        const int off = g_di_role_offset[r];
        if (off >= 0) {
            printf("%s=位%d ", kDiRoleNames[r], off);
        }
    }
    printf("\n");
    printf("  DO: %s (base=M%d, %d点, IP=%s)\n",
           g_do_module_enabled ? "启用" : "禁用", g_do_base_address,
           g_do_count, g_do_device_ip.c_str());
    printf("      ");
    for (int r = 0; r < DO_ROLE_COUNT; r++) {
        const int off = g_do_role_offset[r];
        if (off >= 0) {
            printf("%s=位%d ", kDoRoleNames[r], off);
        }
    }
    printf("\n");
    printf("  急停极性: %s | 复位确认: %dms\n",
           g_emergency_stop_active_low ? "低电平有效(正常=高)" : "高电平有效(正常=低)",
           g_reset_confirm_ms);
    // IO 控模式(CONTROL_SOURCE_IO=1)下手自动来源：角色已配置→按 DI 电平；未配置→按默认模式
    printf("  手自动按钮角色: %s", g_di_role_offset[DI_ROLE_MANUAL_AUTO_BUTTON] >= 0
                                       ? "已配置(高=自动/低=手动)"
                                       : "未配置");
    if (g_di_role_offset[DI_ROLE_MANUAL_AUTO_BUTTON] < 0) {
        printf(" | IO控默认模式: %s", g_default_auto_mode ? "自动" : "手动");
    }
    printf("\n");

    printf("  暂停复位DO(%d): ", g_pause_reset_do_count);
    for (int i = 0; i < g_pause_reset_do_count; i++) printf("M%d ", g_pause_reset_do[i]);
    if (g_pause_reset_do_count == 0) {
        printf("<未配置，暂停时不复位任何DO>");
        fprintf(stderr, "[IO] WARN: safety.pause_reset_do 未配置，暂停时不会复位任何 DO\n");
    }
    printf("\n");
    printf("  关闭复位DO(%d): M%d ~ M%d\n", g_shutdown_reset_do_count,
           g_shutdown_reset_do_count > 0 ? g_shutdown_reset_do[0] : 0,
           g_shutdown_reset_do_count > 0 ? g_shutdown_reset_do[g_shutdown_reset_do_count - 1] : 0);

    printf("  保护DO(禁止外部写): ");
    for (int i = 0; i < IO_MAX_SIGNALS; i++) {
        if (g_do_protected[i]) printf("M%d ", g_do_base_address + i);
    }
    printf("\n");

    g_io_config_valid = true;
    return true;
}

// ===== 从设备刷新 DO 状态（内部函数）=====
static void refresh_do_state_from_device() {
    std::lock_guard<std::mutex> lock(get_do_mutex());
    modbus_t *ctx = get_do_ctx();
    if (!ctx) return;

    // 按实际点数读取（下标 = 位偏移），再按角色表回填语义字段
    if (g_do_count <= 0) return;

    uint8_t do_values[IO_MAX_SIGNALS] = {0};
    if (modbus_read_bits(ctx, 0, g_do_count, do_values) == g_do_count) {
        for (int i = 0; i < g_do_count; i++) {
            current_do_state.raw[i] = do_values[i];
        }
        current_do_state.count = g_do_count;
        apply_do_semantics_from_raw();
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
            printf("DI模块已禁用(或未提供IP)，跳过DI设备连接\n");
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
        } else if (result != 0) {
            // 注意：这里的第二条件 result==0 会让"DI 连接失败"也走到本分支，
            // 历史日志因此误报为"DO模块已禁用"，排查时极易被带偏，故分开提示。
            printf("DO设备连接跳过：DI连接失败（见上方 DI 连接错误）\n");
        } else {
            printf("DO模块已禁用(或未提供IP)，跳过DO设备连接\n");
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
    di.count = g_di_count;

    if (!g_di_module_enabled) {
        // 无硬件联调模式（DI模块禁用）：返回"安全默认电平"而非全0。
        // 急停必须模拟为"正常"，否则 di.emergency_stop 恒为 false，
        // 系统会被永久判定为急停激活，导致 update_button_lights 拦截启动按钮
        // （含虚拟启动）且红灯常亮。其余信号保持"未触发"。
        di.emergency_stop = true;
        return di;
    }

    std::lock_guard<std::mutex> lock(get_di_mutex());
    modbus_t *ctx = get_di_ctx();
    if (!ctx) {
        fprintf(stderr, "DI Modbus 未初始化\n");
        return di;
    }

    // 保留上次有效DI状态：读取失败时返回旧值而非全0，
    // 避免瞬态Modbus错误被误判为"按钮全部释放/急停未触发"
    static DI_Interface last_valid_di = {0};

    if (g_di_count <= 0) {
        return di;
    }

    uint8_t di_values[IO_MAX_SIGNALS] = {0};
    int rc = modbus_read_input_bits(ctx, 0, g_di_count, di_values);

    if (rc == -1) {
        static int error_count = 0;
        if (error_count++ % 100 == 0) {
            fprintf(stderr, "读取DI失败: %s (已抑制%d次重复错误，保留上次有效状态)\n",
                    modbus_strerror(errno), error_count);
        }
        return last_valid_di;
    } else {
        static int error_count = 0;
        error_count = 0;
    }

    // 映射：raw[] 直接承载"设备位"（下标 = 位偏移），
    // 语义字段由角色表（di_roles）给出，与位置彻底解耦
    for (int i = 0; i < g_di_count; i++) {
        di.raw[i] = di_values[i];
    }
    di.count              = g_di_count;
    di.emergency_stop     = di_role_value(di, DI_ROLE_EMERGENCY_STOP);
    di.start_button       = di_role_value(di, DI_ROLE_START_BUTTON);
    di.reset_button       = di_role_value(di, DI_ROLE_RESET_BUTTON);
    di.pause_button       = di_role_value(di, DI_ROLE_PAUSE_BUTTON);
    di.manual_auto_button = di_role_value(di, DI_ROLE_MANUAL_AUTO_BUTTON);

    last_valid_di = di;
    return di;
}

// ===== DO 信号写入 =====
int write_single_do_signal(int do_address, bool state) {
    if (!g_do_module_enabled) {
        fprintf(stderr, "警告: DO模块已禁用，无法写入信号\n");
        return -1;
    }

    std::lock_guard<std::mutex> lock(get_do_mutex());
    modbus_t *ctx = get_do_ctx();
    if (!ctx) return -1;

    // 验证地址是否在配置的信号地址表中（契约 C2 保证 address == base + 位偏移）
    bool valid = false;
    for (int i = 0; i < g_do_count; i++) {
        if (g_do_signal_addresses[i] == do_address) { valid = true; break; }
    }
    if (!valid) {
        fprintf(stderr, "DO地址不在配置范围内: M%d（范围 M%d ~ M%d）\n",
                do_address, g_do_base_address, g_do_base_address + g_do_count - 1);
        return -1;
    }

    // 写入单个信号到设备
    int result = modbus_write_bit(ctx, do_address - g_do_base_address, state);

    // 写入成功后刷新DO状态缓存（按实际点数回读，再按角色表回填语义字段）
    if (result == 1 && g_do_count > 0) {
        uint8_t do_values[IO_MAX_SIGNALS] = {0};
        if (modbus_read_bits(ctx, 0, g_do_count, do_values) == g_do_count) {
            for (int i = 0; i < g_do_count; i++) {
                current_do_state.raw[i] = do_values[i];
            }
            current_do_state.count = g_do_count;
            apply_do_semantics_from_raw();
        }
    }

    return result;
}

// 按角色写 DO：角色未配置（IO_ROLE_NONE）时返回 -1，不做任何写入
int write_do_role(DoRole role, bool state) {
    if (role < 0 || role >= DO_ROLE_COUNT) return -1;
    const int off = g_do_role_offset[role];
    if (off < 0) {
        return -1;   // 该角色在本机型的 do_roles 中未配置
    }
    return write_single_do_signal(g_do_base_address + off, state);
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
        for (int i = 0; i < g_do_count; i++) {
            if (g_do_signal_addresses[i] == address) { valid = true; break; }
        }
        if (!valid) {
            fprintf(stderr, "DO地址不在配置范围内: M%d (范围M%d~M%d)\n",
                    address, g_do_base_address, g_do_base_address + g_do_count - 1);
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
