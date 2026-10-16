#include "huichuan_servo_axis.hpp"
#include "ethercat_node.hpp"
#include <iostream>
#include <rclcpp/rclcpp.hpp>

// 汇川PDO配置定义
ec_pdo_entry_info_t huichuan_slave_pdo_entries[] = {
    {0x6040, 0x00, 16}, {0x607a, 0x00, 32}, {0x60b8, 0x00, 16},
    {0x60fe, 0x01, 32}, {0x603f, 0x00, 16}, {0x6041, 0x00, 16},
    {0x6064, 0x00, 32}, {0x6077, 0x00, 16}, {0x60f4, 0x00, 32},
    {0x60b9, 0x00, 16}, {0x60ba, 0x00, 32}, {0x60bc, 0x00, 32},
    {0x60fd, 0x00, 32},
};

ec_pdo_info_t huichuan_slave_pdos[] = {
    {0x1701, 4, huichuan_slave_pdo_entries + 0},
    {0x1b01, 9, huichuan_slave_pdo_entries + 4},
};

ec_sync_info_t huichuan_slave_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, huichuan_slave_pdos + 0, EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 1, huichuan_slave_pdos + 1, EC_WD_DISABLE},
    {0xff}
};
// 添加缺失的全局变量声明
const int HOMING_TOLERANCE = 100;
const int HOMING_STEP = 20;

HuichuanServoAxis::HuichuanServoAxis(const std::string& name, uint16_t position, AxisType axis_type, 
                                                uint32_t product_code, double gear_ratio, AxisCalcMode calc_mode)
    : ServoAxisBase(name, position, axis_type, DriveBrand::HUICHUAN, product_code, gear_ratio, calc_mode),
      product_code_(product_code) {
    
    std::cout << "创建汇川伺服轴: " << name << ", 产品号: 0x" << std::hex << product_code_ 
              << std::dec << ", 减速比: " << gear_ratio << std::endl;
}

void HuichuanServoAxis::configure(ec_master_t* master) {
    sc_ = ecrt_master_slave_config(master, 0, slave_position_,
                                  HUICHUAN_VENDOR_ID, product_code_);
    if (!sc_) {
        RCLCPP_FATAL(rclcpp::get_logger("huichuan_servo"), 
                    "创建汇川 %s 轴从站配置失败, 产品号: 0x%08x", axis_name_.c_str(), product_code_);
        return;
    }
    
    if (ecrt_slave_config_pdos(sc_, EC_END, huichuan_slave_syncs)) {
        RCLCPP_FATAL(rclcpp::get_logger("huichuan_servo"), 
                    "%s 轴PDO配置失败", axis_name_.c_str());
    }
    
    // 回原轴在复位流程中切换为 HM 模式，其余情况保持 CSP 模式
    // 是否回原由 YAML 的 operation_mode 字段决定，不再硬编码轴名
    if (homing_mode_ && g_reset_button_pressed.load()) {
        ecrt_slave_config_sdo8(sc_, 0x6060, 0x00, 0x06);
        // 本次会话必须等驱动器回原达标（状态字 bit12=Homing attained，完成瞬间 0x1637）才算就绪
        begin_homing_session(true);
        RCLCPP_INFO(rclcpp::get_logger("huichuan_servo"),
                   "汇川轴 %s 配置为 HM 模式 (复位/回原流程)", axis_name_.c_str());
    } else {
        ecrt_slave_config_sdo8(sc_, 0x6060, 0x00, 0x08);
        // CSP 会话：bit12 语义为"驱动器跟随指令值"，不参与回原/就绪判定
        begin_homing_session(false);
        RCLCPP_INFO(rclcpp::get_logger("huichuan_servo"),
                   "汇川轴 %s 配置为 CSP 模式", axis_name_.c_str());
    }

    // DC 配置：0x0300=SYNC0 使能，SYNC0 周期 = 1ms。
    // 偏移（shift）取半周期而非 0：主站是"唤醒后才 receive/process/send"，
    // 数据到达必然晚于周期边界；shift=0 时 SYNC0 与周期边界同相，
    // 发送时刻的调度抖动会直接变成 DC 偏差/SYNC0 相位抖动（严重时丢帧）。
    // 取半周期（500us）给过程数据留出稳定窗口，让驱动器在 SYNC0 时刻
    // 锁存到的始终是本周期已稳定写入的数据。
    ecrt_slave_config_dc(sc_, 0x0300, PERIOD_NS, PERIOD_NS / 2, 0, 0);
    RCLCPP_INFO(rclcpp::get_logger("huichuan_servo"), 
                "汇川轴 %s 配置完成", axis_name_.c_str());
}

void HuichuanServoAxis::register_pdo_entries(ec_pdo_entry_reg_t* reg_list, int& index) {
    // 汇川特有的PDO注册
    control_word_ = ecrt_slave_config_reg_pdo_entry(
        sc_, 0x6040, 0, domain1, NULL);
    status_word_ = ecrt_slave_config_reg_pdo_entry(
        sc_, 0x6041, 0, domain1, NULL);
    off_target_position_ = ecrt_slave_config_reg_pdo_entry(
        sc_, 0x607A, 0, domain1, NULL);
    off_actual_position_ = ecrt_slave_config_reg_pdo_entry(
        sc_, 0x6064, 0, domain1, NULL);
    off_error_code_ = ecrt_slave_config_reg_pdo_entry(
        sc_, 0x603F, 0, domain1, NULL);
}

void HuichuanServoAxis::handle_state_machine(uint8_t* domain1_pd) {
    int32_t current_pos = EC_READ_S32(domain1_pd + off_actual_position_);
    raw_encoder_pulses_.store(current_pos, std::memory_order_relaxed);  // 供 /joint_states 直读绝对编码器
    uint16_t read_status_word = EC_READ_U16(domain1_pd + status_word_);
    uint16_t error_code = EC_READ_U16(domain1_pd + off_error_code_);

    // 统一解码 CiA402 PDS 状态（屏蔽厂商位，兼容 0x0650/0x2650/0x1e37 等）
    const PdsState pds = decode_pds_state(read_status_word);

    // 回原达标判定（仅 HM 会话生效）：维护 bit12 连续稳定计数并锁存，供就绪判定/INITIALIZING 拦截使用
    update_homing_attainment(pds, read_status_word);

    // 检查故障清除请求
    if (clear_fault_requested_ && current_state_ == AxisState::FAULT) {
        clear_fault_requested_ = false;
        fault_clearing_in_progress_ = true;
        fault_clear_step_ = 0;
        fault_clear_counter_ = 0;
        printf("轴 %s 开始清除故障流程\n", axis_name_.c_str());
    }

    // 检查重置请求
    if (reset_requested_) {
        reset_requested_ = false;
        init_delay_ = 0;  // 重置延时计数器，确保重新进入UNINITIALIZED时重新等待
        current_state_ = AxisState::UNINITIALIZED;
        // 回原达标锁存清零：重置后必须重新回原达标，不能用旧的达标结果放行就绪
        homing_attained_.store(false, std::memory_order_release);
        homing_attained_stable_count_ = 0;
        // 重置运动状态，避免恢复时误判目标到达
        reset_motion_state();
        printf("轴 %s 执行重置，回到未初始化状态\n", axis_name_.c_str());
    }

    // 状态转换逻辑
    switch (current_state_) {
        case AxisState::UNINITIALIZED:
                init_delay_++;
                // 首次上电前2000个周期只观察，等待通信稳定
                if (init_delay_ < 2000) {
                    break;
                }
                current_state_ = AxisState::INITIALIZING;
                printf("轴 %s 进入初始化状态\n", axis_name_.c_str());                          
            break;
            
        case AxisState::INITIALIZING:
            // 按 CiA402 标准掩码解码状态，屏蔽厂商位：
            // 0x0250/0x0650/0x1650/0x2650/0x0670/0x0770 等统一识别为 SWITCH_ON_DISABLED
            switch (pds) {
                case PdsState::SWITCH_ON_DISABLED:
                    // 开关断开 → 发送 Shutdown，进入 Ready to switch on
                    EC_WRITE_U16(domain1_pd + control_word_, 0x0006);
                    init_stuck_counter_ = 0;
                    break;

                case PdsState::READY_TO_SWITCH_ON:
                    // Ready to switch on → 发送 Switch on
                    EC_WRITE_U16(domain1_pd + control_word_, 0x0007);
                    init_stuck_counter_ = 0;
                    break;

                case PdsState::SWITCHED_ON:
                case PdsState::OPERATION_ENABLED:
                    // 已到 Switched on 或已使能 → Enable operation（HM 下控制字 bit4 即"回原启动"）
                    // 覆盖 0x1637/0x1e37/0x1eb7 等限位状态，避免卡死在初始化
                    joint_position_ = current_pos;
                    EC_WRITE_S32(domain1_pd + off_target_position_, current_pos);
                    EC_WRITE_U16(domain1_pd + control_word_, 0x001F);

                    // 回原轴（HM 会话）：必须等状态字 bit12=Homing attained 达标才进 READY。
                    // 若在此放行，轴会停在 0x0633(Switched on) 就被判"就绪"，上层随之判定
                    // "系统就绪 → 回原完成 → 待启动"并执行 safe_shutdown，把驱动器正在进行的
                    // 回原打断（现象：回原未完成却日志显示"回原完成"）。
                    // 等待期间每周期都要写控制字，否则驱动器会掉出操作使能。
                    if (is_homing_required() && !is_homing_attained()) {
                        if (++init_stuck_counter_ >= 3000) {  // 约 3s @1ms，仅告警不自动放行
                            init_stuck_counter_ = 0;
                            printf("轴 %s 等待回原完成: 状态字0x%04X（HM 回原中，bit12 未置位）\n",
                                axis_name_.c_str(), read_status_word);
                        }
                        break;  // 留在 INITIALIZING
                    }

                    current_state_ = AxisState::READY;
                    init_stuck_counter_ = 0;
                    printf("轴 %s 进入就绪状态（状态字0x%04X, %s）\n",
                        axis_name_.c_str(), read_status_word, pds_state_name(pds));
                    break;

                case PdsState::FAULT:
                case PdsState::FAULT_REACTION_ACTIVE:
                {
                    uint16_t huichuang_error_code = EC_READ_U16(domain1_pd + off_error_code_);
                    // 特殊处理：0x0E08 通讯错误完全忽略，继续初始化
                    if (huichuang_error_code == 0x0E08 || huichuang_error_code == 0x0000) {
                        printf(" %s --", axis_name_.c_str());
                    } else {
                        current_state_ = AxisState::FAULT;
                        printf("轴 %s 检测到故障状态字0x%04X(%s),错误代码: 0x%04X\n",
                            axis_name_.c_str(), read_status_word, pds_state_name(pds),
                            huichuang_error_code);
                    }
                    init_stuck_counter_ = 0;
                    break;
                }

                default:
                    // NOT_READY / QUICK_STOP_ACTIVE / UNKNOWN：上电瞬态或未定义值，等待稳定；
                    // 超时后打印诊断，避免状态字不匹配导致静默卡死
                    if (++init_stuck_counter_ >= 3000) {  // 约 3s @1ms 周期
                        init_stuck_counter_ = 0;
                        printf("轴 %s 初始化等待超时: 状态字0x%04X(%s), 错误码0x%04X\n",
                            axis_name_.c_str(), read_status_word, pds_state_name(pds),
                            EC_READ_U16(domain1_pd + off_error_code_));
                    }
                    break;
            }
            break;
            
        case AxisState::READY:
            // 处理启动请求 —— 按 PDS 解码判断是否已使能（含 0x1637/0x1e37/0x1eb7 等）
            if (start_manual_requested_ && pds == PdsState::OPERATION_ENABLED) {
                start_manual_requested_ = false;
                operation_mode_ = OperationMode::MANUAL;
                current_state_ = AxisState::MANUAL_MODE;
                printf("轴 %s 进入手动模式（状态字0x%04X）\n", axis_name_.c_str(), read_status_word);

                // 初始化手动模式位置
                // 退出自动模式时重置回零状态
                homing_completed_ = false;
                homing_in_progress_ = false;
                position_initialized_ = false;
            } 
            else if (start_auto_requested_ && pds == PdsState::OPERATION_ENABLED) {
                start_auto_requested_ = false;
                operation_mode_ = OperationMode::AUTO;
                current_state_ = AxisState::AUTO_MODE;

                // 清除非自动模式期间可能残留的位移指令：手动模式下 /displacement_command(_pct)
                // 会被节点接受并写入 displacement_updated_，但手动处理函数只认点动、不消费该标志，
                // 若不在切入自动时清掉，handle_huichuan_auto_operation 会把旧目标当成新指令执行
                displacement_updated_ = false;

                printf("轴 %s 进入自动模式（状态字0x%04X）\n", axis_name_.c_str(), read_status_word);
            }
            else {
                // 保持在READY状态，保持当前位置
                // 使用 joint_position_（进入READY时缓存的固定值）而非 current_pos（每周期读0x6064），
                // 避免正反馈闭环导致编码器位置持续漂移
                EC_WRITE_S32(domain1_pd + off_target_position_, joint_position_);
                EC_WRITE_U16(domain1_pd + control_word_, 0x001F);
                
                // 检测故障位(Bit3)，与 MANUAL_MODE 一致
                if (read_status_word & 0x0008) {
                    uint16_t huichuang_error_code = EC_READ_U16(domain1_pd + off_error_code_);
                    
                    // 汇川 0x0E08 通讯错误不上报，自动修复
                    if (huichuang_error_code == 0x0E08) {
                        printf("轴 %s 0x0E08通讯故障，自动跳转初始化\n", axis_name_.c_str());
                        current_state_ = AxisState::INITIALIZING;
                    } else {
                        printf("轴 %s 检测到故障位(Bit3)，状态字0x%04X，进入故障模式\n",
                            axis_name_.c_str(), read_status_word);
                        current_state_ = AxisState::FAULT;
                    }
                    break;
                }

                // 如果有启动请求但操作使能状态不满足，检查是否真正异常
                if (start_manual_requested_ || start_auto_requested_) {
                    static int warning_counter = 0;
                    if (warning_counter++ % 200 == 0) {
                        printf("轴 %s 等待操作使能状态才能进入模式切换，当前状态字: 0x%04x (mask=0x%04x)\n", 
                            axis_name_.c_str(), read_status_word, read_status_word & 0x0037);
                    }
                    
                    // 只有真正异常（操作使能位不满足、或故障位激活）才跳转初始化
                    if (pds != PdsState::OPERATION_ENABLED && !(read_status_word & 0x0008)) {
                        printf("轴 %s 操作使能位异常(0x%04X)，自动跳转回初始化状态\n", 
                            axis_name_.c_str(), read_status_word);
                        current_state_ = AxisState::INITIALIZING;
                        
                        // 可选：清除启动请求标志，避免重复触发
                        // start_manual_requested_ = false;
                        // start_auto_requested_ = false;
                    }
                }
            }
            break;
            
        case AxisState::MANUAL_MODE:
            // 手动模式处理
            // === 1. 检测故障位 (Bit3 = 0x0008) ===
            // 通用故障检测，覆盖 0x1638/0x0638/0x1eb7 等所有含 Bit3 的状态字
            if (read_status_word & 0x0008) {
                uint16_t huichuang_error_code = EC_READ_U16(domain1_pd + off_error_code_);
                
                // 汇川 0x0E08 通讯错误不上报，自动修复
                if (huichuang_error_code == 0x0E08) {
                    printf("轴 %s 0x0E08通讯故障，自动跳转初始化\n", axis_name_.c_str());
                    current_state_ = AxisState::INITIALIZING;
                    prof_reset_requested_.store(true, std::memory_order_release);  // 退出手动模式：清剖面
                    break;
                }
                
                printf("轴 %s 检测到故障位(Bit3)，状态字0x%04X，错误码0x%04X，进入故障模式\n",
                    axis_name_.c_str(), read_status_word, huichuang_error_code);
                current_state_ = AxisState::FAULT;
                prof_reset_requested_.store(true, std::memory_order_release);  // 退出手动模式：清剖面
                break;
            }
            
            // === 2. 检测内部限位 (Bit11 = 0x0800)，拦截点动 ===
            // 与自动模式共用同一拦截逻辑（按触发方向判断正/负限位，反方向始终放行）
            intercept_jog_by_limit(read_status_word, domain1_pd, current_pos);
            // 不改变 current_state_，保持在 MANUAL_MODE
            
            // === 3. 处理停止请求（结束作业）===
            if (stop_requested_) {
                // 发送停止指令：保持当前位置
                EC_WRITE_S32(domain1_pd + off_target_position_, current_pos);
                EC_WRITE_U16(domain1_pd + control_word_, 0x001F);
                // 在循环线程中清理运动状态（线程安全）
                target_pulses_ = current_pos;
                joint_position_ = current_pos;
                prof_reset_requested_.store(true, std::memory_order_release);  // 硬停：清剖面
                displacement_updated_ = false;
                jog_forward_requested_ = false;
                jog_reverse_requested_ = false;
                jog_stop_requested_ = false;
                target_reached_ = false;
                {
                    std::lock_guard<std::mutex> lock(flag_mutex_);
                    target_reached_flag_ = false;
                }
                stop_requested_ = false;
                current_state_ = AxisState::STOPPED;
                printf("轴 %s 收到停止请求，进入停止状态\n", axis_name_.c_str());
                break;
            }
            
            // === 4. 状态字判断与点动执行 ===
            // 按 PDS 解码判断是否已使能（屏蔽厂商位），而非精确值比较
            // Bit3=0(无故障)已由step1保证
            // 接受 0x1637(正常)、0x1e37(限位)、0x1eb7(限位+警告) 等所有非故障运行状态
            if (pds == PdsState::OPERATION_ENABLED) {
                handle_huichuan_manual_operation(domain1_pd, current_pos);
            } else {
                current_state_ = AxisState::INITIALIZING;
                prof_reset_requested_.store(true, std::memory_order_release);  // 退出手动模式：清剖面
                printf("轴 %s 退出手动模式（非操作使能状态字0x%04X，mask=0x%04X）\n",
                    axis_name_.c_str(), read_status_word, read_status_word & 0x0037);
            }
            break;
            
        case AxisState::AUTO_MODE:
            // 自动模式处理
            // === 检测故障位(Bit3) ===
            if (read_status_word & 0x0008) {
                uint16_t huichuang_error_code = EC_READ_U16(domain1_pd + off_error_code_);
                if (huichuang_error_code == 0x0E08) {
                    printf("轴 %s 0x0E08通讯故障，自动跳转初始化\n", axis_name_.c_str());
                    current_state_ = AxisState::INITIALIZING;
                } else {
                    printf("轴 %s 检测到故障位(Bit3)，状态字0x%04X，进入故障模式\n",
                        axis_name_.c_str(), read_status_word);
                    current_state_ = AxisState::FAULT;
                }
                break;
            }
            
            // === 检测内部限位 (Bit11 = 0x0800)，拦截点动 ===
            // 自动模式内也允许点动（分时），故同样受内部限位约束：禁止朝限位方向点动、反方向放行。
            // 与手动模式共用同一拦截逻辑；不改变 current_state_，保持在 AUTO_MODE。
            intercept_jog_by_limit(read_status_word, domain1_pd, current_pos);

            // === 处理停止请求（结束作业）===
            if (stop_requested_) {
                // 发送停止指令：保持当前位置
                EC_WRITE_S32(domain1_pd + off_target_position_, current_pos);
                EC_WRITE_U16(domain1_pd + control_word_, 0x001F);
                // 在循环线程中清理运动状态（线程安全）
                target_pulses_ = current_pos;
                joint_position_ = current_pos;
                displacement_updated_ = false;
                target_reached_ = false;
                {
                    std::lock_guard<std::mutex> lock(flag_mutex_);
                    target_reached_flag_ = false;
                }
                stop_requested_ = false;
                current_state_ = AxisState::STOPPED;
                printf("轴 %s 收到停止请求，进入停止状态\n", axis_name_.c_str());
                break;
            }
            
            if (pds != PdsState::OPERATION_ENABLED) {
                current_state_ = AxisState::INITIALIZING;
                printf("轴 %s 退出自动模式（非操作使能状态字0x%04X）\n",
                    axis_name_.c_str(), read_status_word);
            } else {
                handle_huichuan_auto_operation(domain1_pd, current_pos);
            }
            break;
            
        case AxisState::STOPPED:
            // 停止状态：保持当前位置，等待回到INITIALIZING
            // === 检测故障位(Bit3) ===
            if (read_status_word & 0x0008) {
                printf("轴 %s 检测到故障位(Bit3)，状态字0x%04X，进入故障模式\n",
                    axis_name_.c_str(), read_status_word);
                current_state_ = AxisState::FAULT;
                break;
            }
            
            EC_WRITE_S32(domain1_pd + off_target_position_, current_pos);
            EC_WRITE_U16(domain1_pd + control_word_, 0x001F);
            
            // +++ 关键修复：在STOPPED状态持续清理运动状态，确保不会继续执行原指令 +++
            displacement_updated_ = false;
            target_pulses_ = current_pos;
            jog_forward_requested_ = false;
            jog_reverse_requested_ = false;
            
            // 检查是否可以回到READY状态（操作使能位mask检查，允许限位状态）
            if (pds == PdsState::OPERATION_ENABLED) {
                current_state_ = AxisState::INITIALIZING;
                // +++ 关键修复：状态转换前再次确认运动状态已清理 +++
                displacement_updated_ = false;
                target_pulses_ = current_pos;
                joint_position_ = current_pos;
                position_initialized_ = false;  // 重新进入时需要重新初始化位置
                printf("轴 %s 从停止状态回到就绪状态\n", axis_name_.c_str());
            }
            break;
            
        case AxisState::FAULT:
            // 故障状态处理
            {
                uint16_t huichuang_error_code = EC_READ_U16(domain1_pd + off_error_code_);
                
                // 汇川 0x0E08 通讯错误自动修复，不上报故障
                if (huichuang_error_code == 0x0E08) {
                    printf("轴 %s 0x0E08通讯故障自动修复，跳转回初始化\n", axis_name_.c_str());
                    current_state_ = AxisState::INITIALIZING;
                } else if (fault_clearing_in_progress_) {
                    handle_fault_clear(domain1_pd);
                } else {
                    EC_WRITE_U16(domain1_pd + control_word_, 0x0006);
                }
            }
            break;
    }
    
    check_state_changes(read_status_word, error_code);
}


void HuichuanServoAxis::handle_huichuan_homing(uint8_t* domain1_pd, int32_t current_pos) {
    // 已经回零完成退出回零逻辑
    if (homing_completed_) {
        return;
    }
    
    // 开始回零序列
    if (!homing_in_progress_) {
        if (current_pos != 0) {
            homing_in_progress_ = true;
            home_target_position_ = current_pos;
            printf("轴 %s 开始软件回零，当前位置: %d\n", axis_name_.c_str(), current_pos);
        } else {
            complete_homing_sequence(domain1_pd);
        }
        return;
    }
    
    // 执行回零运动
    int32_t error = current_pos;
    printf("轴 %s 回零中 - 当前位置: %d, 目标: 0, 误差: %d\n", 
           axis_name_.c_str(), current_pos, error);

    if (abs(error) < HOMING_TOLERANCE) {
        complete_homing_sequence(domain1_pd);
    } else {
        if (home_target_position_ > 0) {
            home_target_position_ -= std::min(HOMING_STEP, home_target_position_);
        } else if (home_target_position_ < 0) {
            home_target_position_ += std::min(HOMING_STEP, -home_target_position_);
        }
        EC_WRITE_S32(domain1_pd + off_target_position_, home_target_position_);
    }
}

// 内部限位 (Bit11 = 0x0800) 对点动的拦截 —— 手动/自动模式共用
//   方向判定：首次触发按脉冲位置推断（原点脉冲=0，不依赖点动方向，自动模式切手动也能正确判定），
//             之后沿用记录的方向；Bit11 清除时复位。
//   拦截范围：仅拦"朝限位方向"的点动（硬停：冻结指令位置 + 清剖面，防限位解除后残留速度冲出），
//             反方向点动始终放行。
//   返回 true 表示本次已拦截并硬停（点动标志已被消费）。
bool HuichuanServoAxis::intercept_jog_by_limit(uint16_t read_status_word, uint8_t* domain1_pd,
                                               int32_t current_pos)
{
    if ((read_status_word & 0x0800) == 0) {
        // Bit11 清除，重置触发方向
        limit_trigger_direction_ = 0;
        return false;
    }

    const bool first_trigger = (limit_trigger_direction_ == 0);
    if (first_trigger) {
        // 首次触发：通过脉冲位置推断限位方向（原点脉冲=0）
        // 正向位置→正限位, 负向位置(含0xFFFFxxxx)→负限位
        limit_trigger_direction_ = (current_pos > 0) ? 1 : -1;
    }

    // 仅拦截触发方向的点动；反方向点动不拦截，由调用方正常执行
    if ((limit_trigger_direction_ == 1  && !jog_forward_requested_) ||
        (limit_trigger_direction_ == -1 && !jog_reverse_requested_)) {
        return false;
    }

    // 硬停：立即冻结指令位置 + 清剖面（限位时驱动器已限住轴，主站减速只会累积位置误差；
    //       且限位不切状态，必须清剖面防解除后冲出）
    if (limit_trigger_direction_ == 1) {
        jog_forward_requested_ = false;
    } else {
        jog_reverse_requested_ = false;
    }
    jog_stop_requested_ = true;
    target_pulses_ = current_pos;
    joint_position_ = current_pos;
    prof_reset_requested_.store(true, std::memory_order_release);
    EC_WRITE_S32(domain1_pd + off_target_position_, current_pos);

    if (limit_trigger_direction_ == 1) {
        if (first_trigger) {
            printf("轴 %s 正限位触发(Bit11, pos=%d)，停止正转（可反转）\n",
                axis_name_.c_str(), current_pos);
        } else {
            printf("轴 %s 正限位持续(Bit11)，禁止正转\n", axis_name_.c_str());
        }
    } else {
        if (first_trigger) {
            printf("轴 %s 负限位触发(Bit11, pos=%d)，停止反转（可正转）\n",
                axis_name_.c_str(), current_pos);
        } else {
            printf("轴 %s 负限位持续(Bit11)，禁止反转\n", axis_name_.c_str());
        }
    }
    return true;
}

// 手动/自动模式共用的点动执行（速度型剖面）
// 自动模式内调用前须先丢弃未完成的位移目标，见 handle_huichuan_auto_operation()
void HuichuanServoAxis::handle_huichuan_jog(uint8_t* domain1_pd) {
    // 获取当前点动速度（线程安全）
    double current_jog_speed;
    {
        std::lock_guard<std::mutex> lock(speed_mutex_);
        current_jog_speed = jog_speed_;
    }

    // 手动点动速度剖面（软起动 / 软停止 / 软换向）
    //   dir: 正转 +1 / 反转 -1 / 无方向请求(含 jog_stop) 0 → 软停止
    //   起步与停止均按 jog_accel/jog_decel 平滑过渡；半途松键安全（每周期重算目标速度）
    const int dir = jog_forward_requested_ ? +1 : (jog_reverse_requested_ ? -1 : 0);
    manual_jog_profile(dir, current_jog_speed, domain1_pd);

    // 停稳后清一次性停止请求标志（软停止可持续多个周期，不能像硬停那样立即清）
    if (dir == 0 && prof_v_ == 0.0) {
        jog_stop_requested_ = false;
    }
}

void HuichuanServoAxis::handle_huichuan_manual_operation(uint8_t* domain1_pd, int32_t current_pos) {
    if (!position_initialized_) {
        joint_position_ = current_pos;
        target_pulses_ = current_pos;
        position_initialized_ = true;
    }

    handle_huichuan_jog(domain1_pd);
}

void HuichuanServoAxis::handle_huichuan_auto_operation(uint8_t* domain1_pd, int32_t current_pos) {

    if (!position_initialized_) {
        joint_position_ = current_pos;
        initial_position_ = current_pos; //current_pos;
        target_pulses_ = initial_position_;
        position_initialized_ = true;
        printf("轴 %s 自动模式位置初始化完成\n", axis_name_.c_str());
    }

    // === 点动优先（分时语义）===
    // 自动模式内允许点动：点动请求生效时抢占本周期，并**丢弃尚未到位的位移目标**。
    // 必须清 displacement_updated_：否则松键后本函数会读到残留标志，把 target 拉回点动
    // 之前的旧绝对目标 → 轴立刻回冲（手动分支不读该标志，故无此问题）。
    // 点动剖面每周期把 target_pulses_ 置为指令位置（见 manual_jog_profile），
    // 故松键后自动段从"当前位置"重新起步，绝对语义不受影响。
    const int jog_dir = jog_forward_requested_ ? +1 : (jog_reverse_requested_ ? -1 : 0);
    if (jog_dir != 0 || jog_stop_requested_) {
        // 仅在"确有方向请求"时抢占：软停尾段（jog_dir==0 && jog_stop_requested_）期间
        // 新到的位移指令保留待生效，避免被静默吞掉
        if (jog_dir != 0 && displacement_updated_) {
            displacement_updated_ = false;
            printf("轴 %s 点动抢占：丢弃未完成的位移目标 %.3f\n",
                   axis_name_.c_str(), target_displacement_);
        }
        handle_huichuan_jog(domain1_pd);
        return;   // 本周期不再逐步逼近自动目标
    }

    if (displacement_updated_) {
        displacement_updated_ = false;
        target_reached_ = false;
        {
            std::lock_guard<std::mutex> lock(flag_mutex_);
            target_reached_flag_ = false; // 新运动开始，清除标志
        }
        // 绝对位置模式：根据轴计算模式选择换算函数
        if (calc_mode_ == AxisCalcMode::ANGLE) {
            target_pulses_ = angle_to_pulses(target_displacement_);
            printf("轴 %s 绝对位置更新: %.3f° -> 目标脉冲 %d (初始: %d, 当前: %d)\n", 
                    axis_name_.c_str(), target_displacement_, target_pulses_, initial_position_, joint_position_);
        } else {
            target_pulses_ = displacement_to_pulses(target_displacement_);
            printf("轴 %s 绝对位置更新: %.3fmm -> 目标脉冲 %d (初始: %d, 当前: %d)\n", 
                    axis_name_.c_str(), target_displacement_, target_pulses_, initial_position_, joint_position_);
        }

    }
    
    // 使用逐步逼近
    if (target_pulses_ != joint_position_) {
        gradual_approach(target_pulses_, domain1_pd);
    } else {
        // 已经到达目标位置，确保标志位被设置
        const int32_t TOLERANCE = 50;
        int32_t error = target_pulses_ - joint_position_;
        if (abs(error) <= TOLERANCE && !target_reached_) {
            std::lock_guard<std::mutex> lock(flag_mutex_);
            target_reached_flag_ = true;
            target_reached_ = true;
            printf("轴 %s 已到达目标位置!(汇川自动模式)\n", axis_name_.c_str());
        }
    }
}

void HuichuanServoAxis::set_huichuan_specific_parameter(double param) {
    huichuan_specific_param_ = param;
}

double HuichuanServoAxis::get_huichuan_specific_parameter() const {
    return huichuan_specific_param_;
}

void HuichuanServoAxis::complete_homing_sequence(uint8_t* domain1_pd) {
    home_target_position_ = 0;
    EC_WRITE_S32(domain1_pd + off_target_position_, home_target_position_);
    homing_in_progress_ = false;
    homing_completed_ = true;
    printf("轴 %s 回零完成, 初始化前target_pulses_: %d\n", 
        axis_name_.c_str(), target_pulses_);
    check_system_initialization();
}


uint16_t HuichuanServoAxis::get_error_code() const {
    return current_error_code_;
}

AxisState HuichuanServoAxis::get_current_state() const {
    return current_state_;
}

// 注：get_max_step() 已基类移除调用，自动模式速度现由 auto_speed_ 控制
// 如需为特定轴设置不同的默认自动速度，可在 YAML 中配置 auto_speed 字段