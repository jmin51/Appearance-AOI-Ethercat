#ifndef IO_INTERFACE_HPP
#define IO_INTERFACE_HPP

#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include <string>

// ============================================================================
// 点位映射：配置驱动（io_config.yaml）
// ============================================================================
// 设计目标：语义（哪个位是急停/启动/按钮灯）由 YAML 的角色表决定，
//           代码只认"位偏移"，从而同一份代码可适配不同机型的点位顺序。
//
// 契约（违反任一条 → 拒绝启动）：
//   C1  di_signals.size() <= IO_MAX_SIGNALS 且 do_signals.size() <= IO_MAX_SIGNALS
//   C2  di_signals[i].address - di_module.base_address == i（do_signals 同理）
//       —— 保证「数组下标 == 位偏移」，控制器发布与 HMI 解析都依赖此约定
//   C3  di_roles / do_roles 的取值是整数 address，落在 [base, base+count) 且互不重复
//   C4  di_roles / do_roles 出现未知 key、或必需角色缺失
//   C5  DI{n} / DO{n} 的 n 即「设备位偏移」（不是结构体字段序号、不是 Modbus 绝对地址）
// ============================================================================

// DI/DO 通道数上限（当前 DI、DO 模块最大都是 48 通道）
#define IO_MAX_SIGNALS 48

// 角色未配置时的哨兵值
#define IO_ROLE_NONE   (-1)

// ===== DI 语义角色 =====
// 必需：急停/启动/复位/暂停；可选：手自动（未配置时恒 false）
enum DiRole {
    DI_ROLE_EMERGENCY_STOP = 0,   // 必需
    DI_ROLE_START_BUTTON,         // 必需
    DI_ROLE_RESET_BUTTON,         // 必需
    DI_ROLE_PAUSE_BUTTON,         // 必需
    DI_ROLE_MANUAL_AUTO_BUTTON,   // 可选
    DI_ROLE_COUNT
};

// ===== DO 语义角色（7 个全部必需）=====
enum DoRole {
    DO_ROLE_BUZZER = 0,
    DO_ROLE_RED_LIGHT,
    DO_ROLE_YELLOW_LIGHT,
    DO_ROLE_GREEN_LIGHT,
    DO_ROLE_START_BUTTON_LIGHT,
    DO_ROLE_RESET_BUTTON_LIGHT,
    DO_ROLE_PAUSE_BUTTON_LIGHT,
    DO_ROLE_COUNT
};

// ===== 运行时配置变量（由 io_config.yaml 加载）=====
extern bool g_di_module_enabled;      // DI模块启用标志
extern bool g_do_module_enabled;      // DO模块启用标志
extern bool g_io_integrated_mode;     // 集成模式标志（true=单设备, false=双设备）
extern int  g_modbus_port;            // Modbus TCP 端口
extern int  g_modbus_slave_id;        // Modbus 从站ID
extern int  g_di_base_address;        // DI起始Modbus地址
extern int  g_do_base_address;        // DO起始Modbus地址
extern std::string g_di_device_ip;    // DI设备IP
extern std::string g_do_device_ip;    // DO设备IP

// 有效点数（= 各自 yaml 的信号条数）
extern int g_di_count;
extern int g_do_count;

// 角色 → 位偏移（IO_ROLE_NONE 表示该角色未配置）
extern int g_di_role_offset[DI_ROLE_COUNT];
extern int g_do_role_offset[DO_ROLE_COUNT];

// 被保护（不允许外部 /do_control 写）的位；下标 = 位偏移
extern bool g_do_protected[IO_MAX_SIGNALS];

// 急停极性：true=低电平有效（正常时为高电平）
extern bool g_emergency_stop_active_low;
// 复位按钮长按确认时长（ms）
extern int  g_reset_confirm_ms;

// IO 控模式（CONTROL_SOURCE_IO=1）下、手自动按钮角色(DI_ROLE_MANUAL_AUTO_BUTTON)未配置
// 时的默认模式：true=自动, false=手动。
// 仅在该角色未配置时生效；角色已配置则始终以 DI 电平为准（高=自动、低=手动，无极性反转）。
// 缺省 false（手动），保持历史行为不变（历史实现对该情形恒判手动）。
extern bool g_default_auto_mode;

// 暂停时需复位的 DO 地址清单（缺省为空）
extern int g_pause_reset_do[IO_MAX_SIGNALS];
extern int g_pause_reset_do_count;
// 关闭时需复位的 DO 地址清单（缺省 = 全部，即 [base, base+count)）
extern int g_shutdown_reset_do[IO_MAX_SIGNALS];
extern int g_shutdown_reset_do_count;

// DI/DO 信号地址表（由YAML加载填充；下标 = 位偏移）
extern int g_di_signal_addresses[IO_MAX_SIGNALS];
extern int g_do_signal_addresses[IO_MAX_SIGNALS];

// DI 输入信号接口定义
// raw[] 承载"设备位"，下标 = 位偏移（= address - base_address），与 yaml 的 di_signals 下标一致；
// 语义字段由 di_roles 角色表填充 —— 调用方按字段名取用，不再依赖位置。
typedef struct {
    bool raw[IO_MAX_SIGNALS];    // raw[i] = 设备第 i 位
    int  count;                  // 本次有效位数（= yaml di_signals 条数）
    // ---- 语义字段（角色表驱动，不再与位置绑定）----
    bool emergency_stop;         // 急停（true = 正常；active_low 时物理高电平为正常）
    bool start_button;           // 启动
    bool reset_button;           // 复位
    bool pause_button;           // 暂停
    bool manual_auto_button;     // 手自动（可选角色，未配置恒 false）
} DI_Interface;

// DO 输出信号接口定义
// raw[] 承载"设备位"，下标 = 位偏移（= address - base_address），与 yaml 的 do_signals 下标一致；
// 语义字段由 do_roles 角色表填充。工艺执行器（气缸/皮带等）无角色，按位偏移直接读写。
typedef struct {
    bool raw[IO_MAX_SIGNALS];    // raw[i] = 设备第 i 位
    int  count;                  // 本次有效位数（= yaml do_signals 条数）
    // ---- 语义字段（角色表驱动）----
    bool buzzer;                 // 蜂鸣器
    bool red_light;              // 三色红灯
    bool yellow_light;           // 三色黄灯
    bool green_light;            // 三色绿灯
    bool start_button_light;     // 启动按钮灯
    bool reset_button_light;     // 复位按钮灯
    bool pause_button_light;     // 暂停按钮灯
} DO_Interface;

// ===== IO 配置加载（从 io_config.yaml 读取）=====
// 返回 false 表示配置不合法（契约 C1~C4 任一被违反），调用方应据此 fail-fast
bool load_io_config(const std::string &yaml_path);

// 配置是否已成功加载（供启动流程判断是否 fail-fast）
bool io_config_ok();

// 初始化函数
int init_modbus_interface(const char* di_ip, int di_port, int di_slave_id, 
                         const char* do_ip, int do_port, int do_slave_id);
void cleanup_modbus_interface();

// ===== 角色访问接口（语义与位置解耦）=====
// 读取某个 DI 角色的电平；急停返回"true=正常"（已按 g_emergency_stop_active_low 换算）；
// 未配置的可选角色返回 false；未配置的急停角色返回 true（正常）。
bool di_role_value(const DI_Interface &di, DiRole role);
// 按角色写 DO；返回 1 成功、-1 失败（模块禁用 / 角色未配置 / Modbus 错误）
int  write_do_role(DoRole role, bool state);
const char* di_role_name(DiRole role);
const char* do_role_name(DoRole role);

// DI 读取接口
DI_Interface read_all_di_signals();

// DO 写入接口
int write_single_do_signal(int do_address, bool state);

// 获取当前DO状态
DO_Interface get_current_do_state();
void refresh_do_state();  // 强制刷新DO状态

// 模块状态检查
bool is_di_module_enabled();
bool is_do_module_enabled();

// DO控制命令消息格式
struct DOControlCommand {
    std::string do_address;  // DO地址，如 "801"
    bool state;              // 状态：true(1)/false(0)
};

// DO控制命令解析函数
bool parse_do_control_command(const std::string& command, DOControlCommand& do_cmd);
#endif