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
    
    ecrt_slave_config_dc(sc_, 0x0300, PERIOD_NS, 0, 0, 0);
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
            // 优先检查：已处于操作使能状态(Bits 0,1,2,4,5=1)则直接跳READY
            // 覆盖 0x1637/0x1e37/0x1eb7 等，避免限位状态卡死在初始化
            if ((read_status_word & 0x0037) == 0x0037) {
                joint_position_ = current_pos;
                EC_WRITE_S32(domain1_pd + off_target_position_, current_pos);
                current_state_ = AxisState::READY;
                printf("轴 %s 已在操作使能状态(0x%04X)，直接进入就绪\n",
                    axis_name_.c_str(), read_status_word);
                break;
            }
            // 执行初始化序列至1633状态
            if (read_status_word == 0x1650) {
                EC_WRITE_U16(domain1_pd + control_word_, 0x0006);
            } else if (read_status_word == 0x1631) {
                EC_WRITE_U16(domain1_pd + control_word_, 0x0007);
            } else if (read_status_word == 0x1633) {
                joint_position_ = current_pos;
                EC_WRITE_S32(domain1_pd + off_target_position_, current_pos);
                EC_WRITE_U16(domain1_pd + control_word_, 0x000F);
                current_state_ = AxisState::READY;
                printf("轴 %s 进入就绪状态\n", axis_name_.c_str());
            } else if (read_status_word == 0x1637) {
                joint_position_ = current_pos;
                EC_WRITE_S32(domain1_pd + off_target_position_, current_pos);
                current_state_ = AxisState::READY;
                printf("轴 %s 进入就绪状态\n", axis_name_.c_str());
            } else if (read_status_word == 0x1638) {
                // 读取错误代码并打印详细故障信息
                uint16_t huichuang_error_code = EC_READ_U16(domain1_pd + off_error_code_);
                
                // 特殊处理：0x0E08错误代码完全忽略，继续初始化
                if (huichuang_error_code == 0x0E08 || huichuang_error_code == 0x0000) {
                    printf(" %s --", axis_name_.c_str());
                    // 不改变current_state_，继续执行初始化序列
                } else {
                    // 其他错误代码正常进入故障模式
                    current_state_ = AxisState::FAULT;
                    printf("轴 %s 检测到故障状态字0x1638,错误代码: 0x%04X\n", 
                        axis_name_.c_str(), huichuang_error_code);
                }
            }
            break;
            
        case AxisState::READY:
            // 处理启动请求 —— 使用操作使能位掩码(0x0037)检查，允许 0x1637/0x1e37/0x1eb7
            if (start_manual_requested_ && (read_status_word & 0x0037) == 0x0037) {
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
            else if (start_auto_requested_ && (read_status_word & 0x0037) == 0x0037) {
                start_auto_requested_ = false;
                operation_mode_ = OperationMode::AUTO;
                current_state_ = AxisState::AUTO_MODE;

                printf("轴 %s 进入自动模式（状态字0x%04X）\n", axis_name_.c_str(), read_status_word);
            }
            else {
                // 保持在READY状态，保持当前位置
                // 使用 joint_position_（进入READY时缓存的固定值）而非 current_pos（每周期读0x6064），
                // 避免正反馈闭环导致编码器位置持续漂移
                EC_WRITE_S32(domain1_pd + off_target_position_, joint_position_);
                EC_WRITE_U16(domain1_pd + control_word_, 0x000F);
                
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
                    if ((read_status_word & 0x0037) != 0x0037 && !(read_status_word & 0x0008)) {
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
                    break;
                }
                
                printf("轴 %s 检测到故障位(Bit3)，状态字0x%04X，错误码0x%04X，进入故障模式\n",
                    axis_name_.c_str(), read_status_word, huichuang_error_code);
                current_state_ = AxisState::FAULT;
                break;
            }
            
            // === 2. 检测内部限位 (Bit11 = 0x0800) ===
            // 根据触发时的点动方向判断正/负限位，反方向始终放行
            if (read_status_word & 0x0800) {
                if (limit_trigger_direction_ == 0) {
                    // 首次触发：通过脉冲位置推断限位方向（原点脉冲=0）
                    // 正向位置→正限位, 负向位置(含0xFFFFxxxx)→负限位
                    // 不依赖点动方向，自动模式切手动也能正确判定
                    if (current_pos > 0) {
                        limit_trigger_direction_ = 1;   // 正限位
                    } else {
                        limit_trigger_direction_ = -1;  // 负限位
                    }
                    
                    // 按推断方向处理当前点动请求
                    if (limit_trigger_direction_ == 1 && jog_forward_requested_) {
                        jog_forward_requested_ = false;
                        jog_stop_requested_ = true;
                        target_pulses_ = current_pos;
                        joint_position_ = current_pos;
                        EC_WRITE_S32(domain1_pd + off_target_position_, current_pos);
                        printf("轴 %s 正限位触发(Bit11, pos=%d)，停止正转（可反转）\n",
                            axis_name_.c_str(), current_pos);
                    } else if (limit_trigger_direction_ == -1 && jog_reverse_requested_) {
                        jog_reverse_requested_ = false;
                        jog_stop_requested_ = true;
                        target_pulses_ = current_pos;
                        joint_position_ = current_pos;
                        EC_WRITE_S32(domain1_pd + off_target_position_, current_pos);
                        printf("轴 %s 负限位触发(Bit11, pos=%d)，停止反转（可正转）\n",
                            axis_name_.c_str(), current_pos);
                    }
                } else {
                    // Bit11 持续，仅拦截触发方向的点动，反方向放行
                    if (limit_trigger_direction_ == 1 && jog_forward_requested_) {
                        jog_forward_requested_ = false;
                        jog_stop_requested_ = true;
                        target_pulses_ = current_pos;
                        joint_position_ = current_pos;
                        EC_WRITE_S32(domain1_pd + off_target_position_, current_pos);
                        printf("轴 %s 正限位持续(Bit11)，禁止正转\n", axis_name_.c_str());
                    } else if (limit_trigger_direction_ == -1 && jog_reverse_requested_) {
                        jog_reverse_requested_ = false;
                        jog_stop_requested_ = true;
                        target_pulses_ = current_pos;
                        joint_position_ = current_pos;
                        EC_WRITE_S32(domain1_pd + off_target_position_, current_pos);
                        printf("轴 %s 负限位持续(Bit11)，禁止反转\n", axis_name_.c_str());
                    }
                    // 反方向点动不拦截，由 step4 正常执行 handle_manual_operation
                }
                // 不改变 current_state_，保持在 MANUAL_MODE
            } else {
                // Bit11 清除，重置触发方向
                limit_trigger_direction_ = 0;
            }
            
            // === 3. 处理停止请求（结束作业）===
            if (stop_requested_) {
                // 发送停止指令：保持当前位置
                EC_WRITE_S32(domain1_pd + off_target_position_, current_pos);
                EC_WRITE_U16(domain1_pd + control_word_, 0x000F);
                // 在循环线程中清理运动状态（线程安全）
                target_pulses_ = current_pos;
                joint_position_ = current_pos;
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
            // 使用操作使能位掩码(0x0037)检查，而非精确值比较
            // Bit3=0(无故障)已由step1保证，这里验证操作使能位(Bits 0,1,2,4,5)
            // 接受 0x1637(正常)、0x1e37(限位)、0x1eb7(限位+警告) 等所有非故障运行状态
            if ((read_status_word & 0x0037) == 0x0037) {
                handle_huichuan_manual_operation(domain1_pd, current_pos);
            } else {
                current_state_ = AxisState::INITIALIZING;
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
            
            // === 处理停止请求（结束作业）===
            if (stop_requested_) {
                // 发送停止指令：保持当前位置
                EC_WRITE_S32(domain1_pd + off_target_position_, current_pos);
                EC_WRITE_U16(domain1_pd + control_word_, 0x000F);
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
            
            if ((read_status_word & 0x0037) != 0x0037) {
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
            EC_WRITE_U16(domain1_pd + control_word_, 0x000F);
            
            // +++ 关键修复：在STOPPED状态持续清理运动状态，确保不会继续执行原指令 +++
            displacement_updated_ = false;
            target_pulses_ = current_pos;
            jog_forward_requested_ = false;
            jog_reverse_requested_ = false;
            
            // 检查是否可以回到READY状态（操作使能位mask检查，允许限位状态）
            if ((read_status_word & 0x0037) == 0x0037) {
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

void HuichuanServoAxis::handle_huichuan_manual_operation(uint8_t* domain1_pd, int32_t current_pos) {
    if (!position_initialized_) {
        joint_position_ = current_pos;
        target_pulses_ = current_pos;
        position_initialized_ = true;
    }
    
    // 获取当前点动速度（线程安全）
    double current_jog_speed;
    {
        std::lock_guard<std::mutex> lock(speed_mutex_);
        current_jog_speed = jog_speed_;
    }
    // 处理点动控制 —— 根据轴计算模式选择对应的转换函数
    // 角度轴: jog_speed单位为 °/s, 使用 angle_to_pulses()
    // 位移轴: jog_speed单位为 mm/s, 使用 displacement_to_pulses()
    if (jog_forward_requested_) {
        int32_t speed_pulses;
        if (calc_mode_ == AxisCalcMode::ANGLE) {
            speed_pulses = angle_to_pulses(current_jog_speed * PERIOD);
        } else {
            speed_pulses = displacement_to_pulses(current_jog_speed * PERIOD);
        }
        target_pulses_ += speed_pulses;
    } else if (jog_reverse_requested_) {
        int32_t speed_pulses;
        if (calc_mode_ == AxisCalcMode::ANGLE) {
            speed_pulses = angle_to_pulses(current_jog_speed * PERIOD);
        } else {
            speed_pulses = displacement_to_pulses(current_jog_speed * PERIOD);
        }
        target_pulses_ -= speed_pulses;
    } else if (jog_stop_requested_) {
        // 停止：保持当前位置
        target_pulses_ = current_pos;
        jog_stop_requested_ = false;
    }
    
    // 限制最大速度 —— 同样根据轴类型选择转换方式
    int32_t MAX_STEP;
    if (calc_mode_ == AxisCalcMode::ANGLE) {
        MAX_STEP = angle_to_pulses(max_jog_speed_ * PERIOD);
    } else {
        MAX_STEP = displacement_to_pulses(max_jog_speed_ * PERIOD);
    }
    int32_t error = target_pulses_ - joint_position_;
    int32_t step = (abs(error) > MAX_STEP) ? 
                  ((error > 0) ? MAX_STEP : -MAX_STEP) : error;
    
    joint_position_ += step;
    EC_WRITE_S32(domain1_pd + off_target_position_, joint_position_);
}

void HuichuanServoAxis::handle_huichuan_auto_operation(uint8_t* domain1_pd, int32_t current_pos) {

    if (!position_initialized_) {
        joint_position_ = current_pos;
        initial_position_ = current_pos; //current_pos;
        target_pulses_ = initial_position_;
        position_initialized_ = true;
        printf("轴 %s 自动模式位置初始化完成\n", axis_name_.c_str());
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