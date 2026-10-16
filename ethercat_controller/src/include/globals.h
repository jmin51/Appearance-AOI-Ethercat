// globals.h
#ifndef GLOBALS_H
#define GLOBALS_H

#include <modbus/modbus.h>
#include <atomic>
#include <memory>

// 前向声明
class EthercatNode;
extern std::shared_ptr<EthercatNode> global_node;

// EtherCAT相关全局变量
extern ec_master_t *master;
extern ec_domain_t *domain1;
extern uint8_t *domain1_pd;
extern std::atomic<bool> g_should_exit;

// 状态变量
extern ec_master_state_t master_state;
extern ec_domain_state_t domain1_state;
extern unsigned int counter;
extern unsigned int sync_ref_counter;
extern const struct timespec cycletime;
extern pthread_t thread;
extern bool running;

// Modbus相关全局变量
extern modbus_t *mb_ctx;
extern pthread_t modbus_thread;
extern volatile int modbus_running;
extern std::atomic<int> di13_state;

// 复位灯控制标志（AL states 0x08且全部轴自动模式时熄灭）
extern std::atomic<bool> g_should_clear_reset_light;

// 完整关闭请求（急停/长按暂停触发）
extern std::atomic<bool> g_full_shutdown_requested;

// 复位按钮3秒确认后：急停完成，待执行回原流程
extern std::atomic<bool> g_reset_pending_after_estop;

// 硬件（物理）急停电平：true = 急停按下（由主循环按 DI 极性归一化后写入）。
// 与 g_virtual_estop_active（软件锁存、HMI 可解除）严格独立，二者互不写入：
//   - 物理急停是电平型，松开即恢复；虚拟急停是锁存型，只能由 HMI toggle 解除。
//   - 物理侧的任何动作都不影响软件锁存（否则会静默解除虚拟急停，属安全漏洞）。
extern std::atomic<bool> g_hw_estop_active;

// 按设计流程停机锁存（急停 / 复位回原 / 待启动触发）。
// 用途：安全停机后轴状态会被冻结（实时线程已停、主站已释放），
//       若继续按轴状态聚合上报，状态栏会残留 READY/AUTO，与实际"已停机"不符。
//       publish_system_status() 据此把基态报告为 STOPPED。
extern std::atomic<bool> g_system_halted;

#endif