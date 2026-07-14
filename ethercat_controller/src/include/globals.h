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

#endif