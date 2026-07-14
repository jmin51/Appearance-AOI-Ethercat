#ifndef IO_INTERFACE_HPP
#define IO_INTERFACE_HPP

#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include <string>

// ===== DI/DO 信号数量常量（与结构体字段数对应）=====
#define DI_SIGNAL_COUNT  16   // DI信号数量
#define DO_SIGNAL_COUNT  16   // DO信号数量

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

// DI/DO 信号地址数组（与结构体字段顺序严格一致，由YAML加载填充）
extern int g_di_signal_addresses[DI_SIGNAL_COUNT];
extern int g_do_signal_addresses[DO_SIGNAL_COUNT];

// 向后兼容宏（引用已有代码中的 ENABLE_DI_MODULE / ENABLE_DO_MODULE / IO_INTEGRATED_MODE）
// 这些宏已不再用于条件编译，仅保留供外部引用，实际逻辑由运行时变量控制
#define ENABLE_DI_MODULE      (g_di_module_enabled ? 1 : 0)
#define ENABLE_DO_MODULE      (g_do_module_enabled ? 1 : 0)
#define IO_INTEGRATED_MODE    (g_io_integrated_mode ? 1 : 0)
#define DI_DEVICE_IP          g_di_device_ip.c_str()
#define DO_DEVICE_IP          g_do_device_ip.c_str()
#define MODBUS_PORT           g_modbus_port
#define MODBUS_SLAVE_ID       g_modbus_slave_id

// DI 输入信号接口定义 (M500 ~ M515, 共16个)
// 前6个为控制面板标准信号(语义名), 后10个为工艺传感器(通用序号名)
typedef struct {
    bool start_button;           // M500 启动按钮
    bool reset_button;           // M501 复位按钮
    bool pause_button;           // M502 暂停按钮
    bool manual_auto_button;     // M503 手自动按钮
    bool emergency_stop;          // M504 急停按钮
    bool air_supply;             // M505 气源输入
    bool di_M506;                // M506 通用输入 - YAML desc定义实际用途
    bool di_M507;                // M507
    bool di_M508;                // M508
    bool di_M509;                // M509
    bool di_M510;                // M510
    bool di_M511;                // M511
    bool di_M512;                // M512
    bool di_M513;                // M513
    bool di_M514;                // M514
    bool di_M515;                // M515
} DI_Interface;

// DO 输出信号接口定义 (M800 ~ M815, 共16个)
// 前7个为指示灯/蜂鸣器标准信号(语义名), 后9个为工艺执行器(通用序号名)
typedef struct {
    bool start_button_light;     // M800 启动按钮灯
    bool reset_button_light;     // M801 复位按钮灯
    bool pause_button_light;    // M802 暂停按钮灯
    bool buzzer;                // M803 蜂鸣器
    bool red_light;             // M804 三色红灯
    bool yellow_light;          // M805 三色黄灯
    bool green_light;           // M806 三色绿灯
    bool do_M807;               // M807 通用输出 - YAML desc定义实际用途
    bool do_M808;               // M808
    bool do_M809;               // M809
    bool do_M810;               // M810
    bool do_M811;               // M811
    bool do_M812;               // M812
    bool do_M813;               // M813
    bool do_M814;               // M814
    bool do_M815;               // M815
} DO_Interface;

// 编译期校验：结构体字段数必须与 COUNT 宏一致
static_assert(sizeof(DI_Interface) / sizeof(bool) == DI_SIGNAL_COUNT,
              "DI_SIGNAL_COUNT 与 DI_Interface 字段数不匹配! 请同步修改。");
static_assert(sizeof(DO_Interface) / sizeof(bool) == DO_SIGNAL_COUNT,
              "DO_SIGNAL_COUNT 与 DO_Interface 字段数不匹配! 请同步修改。");

// ===== IO 配置加载（从 io_config.yaml 读取）=====
bool load_io_config(const std::string &yaml_path);

// 初始化函数
int init_modbus_interface(const char* di_ip, int di_port, int di_slave_id, 
                         const char* do_ip, int do_port, int do_slave_id);
void cleanup_modbus_interface();

// DI 读取接口
DI_Interface read_all_di_signals();
bool read_single_di_signal(int di_address);

// DO 写入接口
int write_do_signals(DO_Interface do_signals);
int write_single_do_signal(int do_address, bool state);

// 获取当前DO状态
DO_Interface get_current_do_state();
void refresh_do_state();  // 强制刷新DO状态

// 模块状态检查
bool is_di_module_enabled();
bool is_do_module_enabled();

// 非阻塞延时检查
int should_execute_sequence(time_t *last_time, int interval_seconds);
// DO控制命令消息格式
struct DOControlCommand {
    std::string do_address;  // DO地址，如 "801"
    bool state;              // 状态：true(1)/false(0)
};

// DO控制命令解析函数
bool parse_do_control_command(const std::string& command, DOControlCommand& do_cmd);
#endif