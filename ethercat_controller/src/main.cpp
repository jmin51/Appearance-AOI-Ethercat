#include "ethercat_node.hpp"
#include "io_interface.hpp"
#include "lights_controller.hpp"  // 灯光控制器
#include <pthread.h>
#include <sched.h>
#include "globals.h"
#include <sys/mman.h> 
#include <string.h>  

// 添加缺失的cycletime定义
const struct timespec cycletime = {0, PERIOD_NS};

// 时间计算函数
struct timespec timespec_add(struct timespec time1, struct timespec time2) {
    struct timespec result;
    if ((time1.tv_nsec + time2.tv_nsec) >= NSEC_PER_SEC) {
        result.tv_sec = time1.tv_sec + time2.tv_sec + 1;
        result.tv_nsec = time1.tv_nsec + time2.tv_nsec - NSEC_PER_SEC;
    } else {
        result.tv_sec = time1.tv_sec + time2.tv_sec;
        result.tv_nsec = time1.tv_nsec + time2.tv_nsec;
    }
    return result;
}

#define TIMESPEC2NS(T) ((uint64_t)(T).tv_sec * NSEC_PER_SEC + (T).tv_nsec)

// 主站状态检查
void check_master_state(void) {
    ec_master_state_t ms;
    ecrt_master_state(master, &ms);
    
    if (ms.slaves_responding != master_state.slaves_responding)
        printf("%u slave(s).\n", ms.slaves_responding);
    if (ms.al_states != master_state.al_states)
        printf("AL states: 0x%02X.\n", ms.al_states);
    if (ms.link_up != master_state.link_up)
        printf("Link is %s.\n", ms.link_up ? "up" : "down");
        
    master_state = ms;
}

// 信号处理
void signal_handler(int signum) {
    static std::atomic<bool> shutdown_initiated{false};
    
    // 防止重复调用
    if (shutdown_initiated.exchange(true)) {
        return;
    }
    printf("\n收到信号 %d, 开始安全关闭...\n", signum);
    running = 0;
    g_should_exit = true;
    safe_shutdown(false);  // false表示完全关闭模式
}

// 安全关闭
void safe_shutdown(bool is_pause = false) {
    static std::atomic<bool> shutdown_in_progress{false};
    
    if (shutdown_in_progress.exchange(true)) {
        printf("关闭流程已在执行中...\n");
        return;
    }
    
    if (is_pause) {
        printf("\n开始安全暂停流程...\n");
        // 暂停时复位关键DO信号（M810-M813）
        printf("[暂停] 复位关键DO信号 M810-M813...\n");
        write_single_do_signal(810, false);  // 顶升气缸下降
        write_single_do_signal(811, false);  // 齿轮对接气缸伸出
        write_single_do_signal(812, false);  // 皮带正转启动
        write_single_do_signal(813, false);  // 皮带反转启动
        printf("[暂停] DO信号复位完成\n");
    } else {
        printf("\n开始安全关闭流程...\n");
        g_should_exit.store(true, std::memory_order_release);
        // 退出时复位所有 DO 信号（M800-M813）
        printf("[关闭] 复位所有 DO 信号 M800-M813...\n");
        for (int addr = 800; addr <= 813; addr++) {
            write_single_do_signal(addr, false);
        }
        //     g_should_exit.store(true);
    }
    
    // 1. 设置退出标志
    running = 0;
    
    if (!is_pause) {
        // 完全关闭时才停止ROS2执行器
        printf("[DEBUG] 步骤1: 停止ROS2执行器\n");
        if (rclcpp::ok()) {
            rclcpp::shutdown();
            usleep(200000); // 200ms等待ROS2线程退出
        }
    }
    
    printf("[DEBUG] 步骤2: 等待100ms让实时线程检测退出标志\n");
    usleep(100000); // 100ms
    
    // 2. 处理实时线程
    if (thread) {
        printf("[DEBUG] 步骤3: 处理实时线程\n");
        
        int wait_count = 0;
        const int max_wait = 30; // 3秒超时
        
        while (wait_count < max_wait) {
            int result = pthread_tryjoin_np(thread, NULL);
            if (result == 0) {
                printf("[DEBUG] 实时线程正常退出\n");
                break;
            } else if (result == EBUSY) {
                printf("[DEBUG] 实时线程仍在运行，等待计数: %d/%d\n", wait_count + 1, max_wait);
                usleep(100000); // 100ms
                wait_count++;
            } else {
                printf("[DEBUG] pthread_tryjoin_np错误: %d\n", result);
                break;
            }
        }
        
        if (wait_count >= max_wait) {
            printf("[DEBUG] 超时，强制取消实时线程\n");
            pthread_cancel(thread);
            pthread_join(thread, NULL);
        }
        
        thread = 0;
        printf("[DEBUG] 实时线程处理完成\n");
    } else {
        printf("[DEBUG] 实时线程句柄为空\n");
    }
    
    // 3. 暂停模式下不停止Modbus线程和IO监控（删除）
    
    // 4. 禁用驱动器
    if (master && global_node && domain1_pd) {
        printf("禁用所有驱动器...\n");
        auto& axes = global_node->get_servo_axes();
        for (auto& axis : axes) {
            unsigned int offset = axis->get_control_word_offset();
            EC_WRITE_U16(domain1_pd + offset, 0x0006); // 禁用命令
        }
        
        // 发送最后一次命令
        ecrt_domain_queue(domain1);
        usleep(10000);
    }
    
    // 5. 释放EtherCAT资源
    if (master) {
        if (is_pause) {
            printf("暂停模式：释放EtherCAT主站资源\n");
        } else {
            printf("释放EtherCAT资源...\n");
        }
        ecrt_master_deactivate(master);
        ecrt_release_master(master);
        master = nullptr;
    }
    
    // 6. 完全关闭时的额外清理
    if (!is_pause) {
        printf("解除内存锁定...\n");
        munlockall();
    } else {
        printf("暂停模式：保持内存锁定和全局节点\n");
    }
    
    if (is_pause) {
        printf("安全暂停完成\n");
    } else {
        // 清理全局节点
        if (global_node) {
            global_node.reset();
        }
        printf("安全关闭完成\n");
    }
    
    shutdown_in_progress.store(false);
}

// 域状态检查
void check_domain1_state(void) {
    ec_domain_state_t ds;
    ecrt_domain_state(domain1, &ds);
    
    if (ds.working_counter != domain1_state.working_counter)
        printf("Domain1 WC: %u\n", ds.working_counter);
    if (ds.wc_state != domain1_state.wc_state)
        printf("Domain1 State: %u\n", ds.wc_state);
        
    domain1_state = ds;
}

// 实时任务线程
void* rt_task_wrapper(void* arg) {
    printf("实时线程启动 (优先级: %d)\n", sched_get_priority_max(SCHED_FIFO));
    if (!master || !domain1 || !domain1_pd) {
        printf("EtherCAT资源未就绪，等待初始化...\n");
        return nullptr;
    }
    // 设置CPU亲和性
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(0, &cpuset);
    if (pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset)) {
        perror("设置CPU亲和性失败");
    }
    
    struct timespec wakeup_time, current_time;
    clock_gettime(CLOCK_TO_USE, &wakeup_time);
    
    while (running && !g_should_exit.load()) {
        // 检查暂停按钮
        if (g_pause_button_pressed.load()) {
            printf("检测到暂停按钮，准备安全关闭...\n");
            break;
        }
        
        // 检查系统运行状态
        if (!g_system_running.load()) {
            usleep(50000); // 50ms等待
            continue;
        }
        
        wakeup_time = timespec_add(wakeup_time, cycletime);
        clock_nanosleep(CLOCK_TO_USE, TIMER_ABSTIME, &wakeup_time, NULL);
        
        // 设置应用时间
        ecrt_master_application_time(master, TIMESPEC2NS(wakeup_time));
        
        // EtherCAT通信处理
        ecrt_master_receive(master);
        ecrt_domain_process(domain1);
        
        // 状态检查
        // check_domain1_state();
        
        if (counter == 0) {
            counter = FREQUENCY;
            check_master_state();
        } else {
            counter--;
        }
        
        // 处理轴状态机
        if (global_node && domain1_pd) {
            global_node->handle_axes_state_machines(domain1_pd);
        }
        
        // // 发布关节状态（每10ms） 后期改为在ethercat_node创建10ms定时器
        // static int joint_state_counter = 0;
        // if (joint_state_counter++ >= 10) {
        //     joint_state_counter = 0;
        //     if (global_node) {
        //         global_node->publish_joint_states();
        //         global_node->check_layer_motion_completion();
        //     }
        // }
        
        // 同步参考时钟
        if (sync_ref_counter == 0) {
            sync_ref_counter = 1;
            clock_gettime(CLOCK_TO_USE, &current_time);
            ecrt_master_sync_reference_clock_to(master, TIMESPEC2NS(current_time));
        } else {
            sync_ref_counter--;
        }
        
        // 同步从站时钟并发送数据
        ecrt_master_sync_slave_clocks(master);
        ecrt_domain_queue(domain1);
        ecrt_master_send(master);
    }
    
    printf("实时线程退出\n");
    return nullptr;
}

// 主函数
int main(int argc, char **argv) {
    // 初始化ROS2
    rclcpp::init(argc, argv);
    global_node = std::make_shared<EthercatNode>("ethercat_controller");
    
    // 设置信号处理
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    // 锁定内存
    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
        perror("mlockall失败");
        return 1;
    }
      
    // 启动IO监控
    printf("启动IO监控模块...\n");
    global_node->start_io_monitoring();
    // 在轴初始化后初始化业务逻辑和层处理器
    // global_node->initialize_after_axes();
    
    // 设置实时线程属性
    pthread_attr_t attr;
    struct sched_param param;
    
    if (pthread_attr_init(&attr)) {
        perror("线程属性初始化失败");
        safe_shutdown(false);
        return 1;
    }
    
    if (pthread_attr_setstacksize(&attr, PTHREAD_STACK_MIN + 32768)) {
        perror("设置栈大小失败");
        safe_shutdown(false);
        return 1;
    }
    
    if (pthread_attr_setschedpolicy(&attr, SCHED_FIFO)) {
        perror("设置调度策略失败");
        safe_shutdown(false);
        return 1;
    }
    
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);
    if (pthread_attr_setschedparam(&attr, &param)) {
        perror("设置优先级失败");
        safe_shutdown(false);
        return 1;
    }
    
    if (pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED)) {
        perror("设置继承调度失败");
        safe_shutdown(false);
        return 1;
    }
    
    // 创建实时线程
    if (pthread_create(&thread, &attr, rt_task_wrapper, NULL)) {
        perror("创建实时线程失败");
        safe_shutdown(false);
        return 1;
    }
    
    pthread_setname_np(thread, "ethercat-rt");
    printf("实时线程创建成功 (优先级: %d)\n", param.sched_priority);
    
    // 运行ROS2执行器
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(global_node);
    
    // 初始化灯光控制器
    init_lights_controller();
    
    printf("=== EtherCAT控制系统启动完成 ===\n");
    printf("系统状态:\n");
    printf("  - EtherCAT主站: 已激活\n");
    printf("  - IO监控: %s\n", global_node->is_io_running() ? "已启动" : "未启动");
    printf("  - 伺服轴数量: %zu\n", global_node->get_servo_axes().size());
    printf("  - DI模块: %s\n", is_di_module_enabled() ? "启用" : "禁用");
    printf("  - DO模块: %s\n", is_do_module_enabled() ? "启用" : "禁用");
    printf("按Ctrl+C退出程序\n\n");
    printf("  - 实时线程: 等待启动按钮\n");
    
    // 使用非阻塞的spin方式，添加启动/暂停检测
    while (rclcpp::ok() && !g_should_exit) {
        executor.spin_some(std::chrono::milliseconds(100));
        
        // 灯光控制（按钮灯 + 三色灯）
        DI_Interface di = read_all_di_signals();
        update_button_lights(di.start_button, di.reset_button, di.pause_button, di.emergency_stop);
        update_tricolor_lights();
        
        // 检查完整关闭请求（急停或复位触发的急停流程）
        if (g_full_shutdown_requested.load()) {
            printf("检测到完整关闭请求（急停）...\n");
            g_system_running.store(false);
            g_start_button_pressed.store(false);
            g_reset_button_pressed.store(false);
            g_pause_button_pressed.store(false);
            g_full_shutdown_requested.store(false);
            safe_shutdown(true);  // true表示暂停模式
            
            // 复位触发的急停完成后，衔接复位回原流程
            if (g_reset_pending_after_estop.load()) {
                g_reset_pending_after_estop.store(false);
                g_reset_button_pressed.store(true);
                printf("[复位] 急停流程完成，衔接复位回原流程...\n");
            } else {
                printf("系统已完全暂停，等待启动按钮...\n");
            }
        }
        
        // 检查所有轴就绪，绿灯闪烁→常亮
        static bool all_axes_ready_notified = false;
        if (g_system_running.load() && global_node && !all_axes_ready_notified) {
            auto& axes = global_node->get_servo_axes();
            bool all_ready = !axes.empty();
            for (auto& axis : axes) {
                if (axis->get_current_state() != AxisState::READY) {
                    all_ready = false;
                    break;
                }
            }
            if (all_ready) {
                notify_all_axes_ready();
                all_axes_ready_notified = true;
            }
        }
        if (!g_system_running.load()) {
            all_axes_ready_notified = false;  // 系统停止后重置标志
        }
        
        // 检查启动按钮
        if (g_start_button_pressed.load() && !g_system_running.load()) {
            printf("开始重新启动系统...\n");
            g_system_running.store(true);
            g_start_button_pressed.store(false);
            
            // 重新初始化EtherCAT资源
            if (!master) {
                printf("重新初始化EtherCAT资源...\n");
                
                // 重新请求主站
                master = ecrt_request_master(0);
                if (!master) {
                    fprintf(stderr, "重新请求EtherCAT主站失败\n");
                    g_system_running.store(false);
                    continue;
                }
                
                // 重新创建域
                domain1 = ecrt_master_create_domain(master);
                if (!domain1) {
                    fprintf(stderr, "重新创建域失败\n");
                    ecrt_release_master(master);
                    master = nullptr;
                    g_system_running.store(false);
                    continue;
                }
                
                // 重新配置从站和PDO
                global_node->init_axes(master);
                global_node->register_pdo_entries(domain1);
                
                // 重新激活主站
                if (ecrt_master_activate(master)) {
                    fprintf(stderr, "主站重新激活失败\n");
                    safe_shutdown(false);
                    continue;
                }
                
                // 重新获取域数据指针
                domain1_pd = ecrt_domain_data(domain1);
                if (!domain1_pd) {
                    fprintf(stderr, "重新获取域数据失败\n");
                    safe_shutdown(false);
                    continue;
                }
                // 在轴初始化后初始化业务逻辑和层处理器
                global_node->initialize_after_axes();
            }
            
            // 创建新的实时线程
            running = 1;
            if (pthread_create(&thread, &attr, rt_task_wrapper, NULL)) {
                perror("创建实时线程失败");
                safe_shutdown(false);
                continue;
            }
            pthread_setname_np(thread, "ethercat-rt");
            printf("实时线程重新创建成功\n");
            
            printf("系统重新启动完成\n");
        }
        else if (g_reset_button_pressed.load() && !g_system_running.load())
        {
            printf("回原启动系统...\n");
            g_system_running.store(true);
            
            // 重新初始化EtherCAT资源
            if (!master) {
                printf("重新初始化EtherCAT资源...\n");
                
                // 重新请求主站
                master = ecrt_request_master(0);
                if (!master) {
                    fprintf(stderr, "重新请求EtherCAT主站失败\n");
                    g_system_running.store(false);
                    continue;
                }
                
                // 重新创建域
                domain1 = ecrt_master_create_domain(master);
                if (!domain1) {
                    fprintf(stderr, "重新创建域失败\n");
                    ecrt_release_master(master);
                    master = nullptr;
                    g_system_running.store(false);
                    continue;
                }
                
                // 重新配置从站和PDO
                global_node->init_axes(master);
                global_node->register_pdo_entries(domain1);
                
                // 重新激活主站
                if (ecrt_master_activate(master)) {
                    fprintf(stderr, "主站重新激活失败\n");
                    safe_shutdown(false);
                    continue;
                }
                
                // 重新获取域数据指针
                domain1_pd = ecrt_domain_data(domain1);
                if (!domain1_pd) {
                    fprintf(stderr, "重新获取域数据失败\n");
                    safe_shutdown(false);
                    continue;
                }
                // 在轴初始化后初始化业务逻辑和层处理器
                global_node->initialize_after_axes();
            }
            
            // 创建新的实时线程
            running = 1;
            if (pthread_create(&thread, &attr, rt_task_wrapper, NULL)) {
                perror("创建实时线程失败");
                safe_shutdown(false);
                continue;
            }
            pthread_setname_np(thread, "ethercat-rt");
            printf("实时线程重新创建成功\n");
            
            printf("系统重新启动完成\n");    
        }

        // 检查暂停按钮
        if (g_pause_button_pressed.load() && g_system_running.load()) {
            printf("开始安全暂停系统...\n");
            g_system_running.store(false);
            g_start_button_pressed.store(false);
            g_reset_button_pressed.store(false);
            g_pause_button_pressed.store(false);
            // // 安全暂停操作 to do后续可简化方向
            // if (thread) {
            //     printf("等待实时线程退出...\n");
            //     pthread_join(thread, nullptr);
            //     thread = 0;
            //     printf("实时线程已退出\n");
            // }
            
            // // 禁用EtherCAT主站
            // if (master) {
            //     printf("禁用EtherCAT主站...\n");
            //     ecrt_master_deactivate(master);
            //     ecrt_release_master(master);
            //     master = nullptr;
            // }
            safe_shutdown(true);  // true表示暂停模式
            
            printf("系统已暂停，等待启动按钮...\n");
        }
        
        // 定期检查退出标志
        if (g_should_exit) {
            break;
        }
    }
    return 0; // 添加适当的返回值
}