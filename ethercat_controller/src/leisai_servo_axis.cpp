#include "leisai_servo_axis.hpp"
#include "ethercat_node.hpp"
#include <iostream>
#include <rclcpp/rclcpp.hpp>

// 雷赛PDO配置定义
ec_pdo_entry_info_t leisai_slave_pdo_entries[] = {
    {0x6040, 0x00, 16}, /* Control Word */
    {0x607a, 0x00, 32}, /* Profile Target Position */
    {0x60b8, 0x00, 16}, /* Touch Probe Function */
    {0x6840, 0x00, 16}, /* Control Word */
    {0x687a, 0x00, 32}, /* Profile Target Position */
    {0x68b8, 0x00, 16}, /* Touch Probe Function */
    {0x603f, 0x00, 16}, /* Last Error Code */
    {0x6041, 0x00, 16}, /* Status Word */
    {0x6061, 0x00, 8}, /* Modes of Operation Display */
    {0x6064, 0x00, 32}, /* Actual Motor Position */
    {0x60b9, 0x00, 16}, /* Touch Probe Status */
    {0x60ba, 0x00, 32}, /* Touch Probe 1 Positive Value */
    {0x60fd, 0x00, 32}, /* Digital Inputs */
    {0x683f, 0x00, 16}, /* Last Error Code */
    {0x6841, 0x00, 16}, /* Status Word */
    {0x6861, 0x00, 8}, /* Modes of Operation Display */
    {0x6864, 0x00, 32}, /* Actual Motor Position */
    {0x68b9, 0x00, 16}, /* Touch Probe Status */
    {0x68ba, 0x00, 32}, /* Touch Probe 1 Positive Value */
    {0x68fd, 0x00, 32}, /* Digital Inputs */
};

ec_pdo_info_t leisai_slave_pdos[] = {
    {0x1600, 3, leisai_slave_pdo_entries + 0}, /* Axis 1 Receive PDO 1 Mapping Parameter */
    {0x1610, 3, leisai_slave_pdo_entries + 3}, /* Axis 2 Receive PDO 1 Mapping Parameter */
    {0x1a00, 7, leisai_slave_pdo_entries + 6}, /* Axis 1 Transmit PDO 1 Mapping Parameter */
    {0x1a10, 7, leisai_slave_pdo_entries + 13}, /* Axis 2 Transmit PDO 1 Mapping Parameter */
};

ec_sync_info_t leisai_slave_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 2, leisai_slave_pdos + 0, EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 2, leisai_slave_pdos + 2, EC_WD_DISABLE},
    {0xff}
};
// 添加缺失的常量定义
const int HOMING_TOLERANCE = 100;
const int HOMING_STEP = 50;

LeisaiServoAxis::LeisaiServoAxis(const std::string& name, uint16_t position, AxisType axis_type, 
                                               uint32_t product_code, double gear_ratio, AxisCalcMode calc_mode)
    : ServoAxisBase(name, position, axis_type, DriveBrand::LEISAI, product_code, gear_ratio, calc_mode), 
      leisai_specific_param_(0),
      product_code_(product_code) {  // 存储产品号
}

void LeisaiServoAxis::configure(ec_master_t* master) {
    sc_ = ecrt_master_slave_config(master, 0, slave_position_,
                                 LEISAI_VENDOR_ID, product_code_);  // 使用存储的产品号
    if (!sc_) {
        RCLCPP_FATAL(rclcpp::get_logger("ethercat_controller"), 
                    "创建 %s 轴从站配置失败，产品号: 0x%08x", axis_name_.c_str(), product_code_);
        return;
    }

    if (ecrt_slave_config_pdos(sc_, EC_END, leisai_slave_syncs)) {
        RCLCPP_FATAL(rclcpp::get_logger("ethercat_controller"), 
                    "%s 轴PDO配置失败", axis_name_.c_str());
    }
    
    // 判断是否为axis3并且复位按钮被按下
    if (axis_name_ == "axis3" && g_reset_button_pressed.load()) {
        ecrt_slave_config_sdo8(sc_, 0x6060, 0x00, 0x06);
        RCLCPP_INFO(rclcpp::get_logger("ethercat_controller"),
                   "轴 %s 配置为 HM 模式 (复位/回原流程)", axis_name_.c_str());
        // 重置复位按钮状态，避免重复配置
        g_reset_button_pressed.store(false);
    } else {
        // 其他轴，或 axis3 在非复位状态下，配置为 CSP 模式
        ecrt_slave_config_sdo8(sc_, 0x6060, 0x00, 0x08);
        RCLCPP_INFO(rclcpp::get_logger("ethercat_controller"),
                   "轴 %s 配置为 CSP 模式", axis_name_.c_str());
    }
    ecrt_slave_config_dc(sc_, 0x0300, PERIOD_NS, 0, 0, 0);
}

void LeisaiServoAxis::register_pdo_entries(ec_pdo_entry_reg_t* reg_list, int& index) {
    
    if (axis_type_ == AxisType::AXIS1) {
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
    } else {
        control_word_ = ecrt_slave_config_reg_pdo_entry(
            sc_, 0x6840, 0, domain1, NULL);
        status_word_ = ecrt_slave_config_reg_pdo_entry(
            sc_, 0x6841, 0, domain1, NULL);
        off_target_position_ = ecrt_slave_config_reg_pdo_entry(
            sc_, 0x687A, 0, domain1, NULL);
        off_actual_position_ = ecrt_slave_config_reg_pdo_entry(
            sc_, 0x6864, 0, domain1, NULL);
        off_error_code_ = ecrt_slave_config_reg_pdo_entry(
            sc_, 0x683F, 0, domain1, NULL);
    }
}

void LeisaiServoAxis::handle_state_machine(uint8_t* domain1_pd) {
    int32_t current_pos = EC_READ_S32(domain1_pd + off_actual_position_); // 修复：使用domain_pd
    raw_encoder_pulses_.store(current_pos, std::memory_order_relaxed);  // 供 /joint_states 直读绝对编码器
    uint16_t read_status_word = EC_READ_U16(domain1_pd + status_word_);
    uint16_t error_code = EC_READ_U16(domain1_pd + off_error_code_);

    // 更新错误码
    current_error_code_ = error_code;

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
        current_state_ = AxisState::UNINITIALIZED;
        // 重置运动状态，避免恢复时误判目标到达
        reset_motion_state();
        printf("轴 %s 执行重置，回到未初始化状态\n", axis_name_.c_str());
    }

    // 状态转换逻辑
    switch (current_state_) {
        case AxisState::UNINITIALIZED:
                current_state_ = AxisState::INITIALIZING;
                printf("轴 %s 进入初始化状态\n", axis_name_.c_str());
            break;
            
        case AxisState::INITIALIZING:
            static unsigned int init_delay = 0;
            init_delay++;
            
            // 前100个周期只观察状态，不发送控制命令
            if (init_delay < 100) {
                break;
            }
            // 新增：检查821b通讯错误并自动清除
            if (error_code == 0x821b) {
                printf("轴 %s 检测到通讯错误0x821b, 开始自动清除故障流程\n", axis_name_.c_str());
                fault_clearing_in_progress_ = true;
                fault_clear_step_ = 0;
                fault_clear_counter_ = 0;
                current_state_ = AxisState::FAULT;
                break;
            }

            // 执行初始化序列至1633状态
            if (read_status_word == 0x0250 || read_status_word == 0x0650 || read_status_word == 0x0670 || read_status_word == 0x0770) {
                EC_WRITE_U16(domain1_pd + control_word_, 0x0006);
                printf("发送状态转换命令: 0x0006\n");
            } else if (read_status_word == 0x0631 || read_status_word == 0x0731) {
                EC_WRITE_U16(domain1_pd + control_word_, 0x0007);
                printf("发送状态转换命令: 0x0007\n");
            } else if (read_status_word == 0x0633 || read_status_word == 0x0733) {
                EC_WRITE_S32(domain1_pd + off_target_position_, current_pos);
                EC_WRITE_U16(domain1_pd + control_word_, 0x001F);
                printf("发送状态转换命令: 0x001F\n");
                current_state_ = AxisState::READY;
                printf("轴 %s 进入就绪状态\n", axis_name_.c_str());
            } else if (read_status_word == 0x1637 || read_status_word == 0x1237 || read_status_word == 0x16b7) {
                EC_WRITE_S32(domain1_pd + off_target_position_, current_pos);
                current_state_ = AxisState::READY;
                printf("轴 %s 进入就绪状态\n", axis_name_.c_str());
            } 
            else if (read_status_word & 0x0008) {  // bit3(Fault)置位
                // 读取错误代码并打印详细故障信息
                uint16_t leisai_error_code = EC_READ_U16(domain1_pd + off_error_code_);

                // 特殊处理：0x821b错误代码完全忽略，继续初始化
                if (leisai_error_code == 0x821b || leisai_error_code == 0x0000) {
                    printf("轴 %s -----", axis_name_.c_str());
                    // 不改变current_state_，继续执行初始化序列
                } else {
                    // 其他错误代码正常进入故障模式
                    current_state_ = AxisState::FAULT;
                    printf("轴 %s 检测到故障状态字0x%04X,错误代码: 0x%04X\n",
                        axis_name_.c_str(), read_status_word, leisai_error_code);
                }
            }
            break;
            
        case AxisState::READY:
            // 处理启动请求
            if (start_manual_requested_ && (read_status_word == 0x1637 || read_status_word == 0x1237)) {
                start_manual_requested_ = false;
                operation_mode_ = OperationMode::MANUAL;
                current_state_ = AxisState::MANUAL_MODE;
                printf("轴 %s 进入手动模式\n", axis_name_.c_str());

                // 初始化手动模式位置
                // 退出自动模式时重置回零状态
                homing_completed_ = false;
                homing_in_progress_ = false;
                position_initialized_ = false;
            } 
            else if (start_auto_requested_ && (read_status_word == 0x1637 || read_status_word == 0x1237)) {
                start_auto_requested_ = false;
                operation_mode_ = OperationMode::AUTO;
                current_state_ = AxisState::AUTO_MODE;

                printf("轴 %s 进入自动模式\n", axis_name_.c_str());
            }
            else {
                // 保持在READY状态，保持当前位置
                // EC_WRITE_S32(domain1_pd + off_target_position_, current_pos);
                // EC_WRITE_U16(domain1_pd + control_word_, 0x001F);
                
                // 新增：检测故障状态字 (bit3 Fault位置位)
                if (read_status_word & 0x0008) {
                    printf("轴 %s 检测到故障状态字0x%04X，进入故障模式\n",
                        axis_name_.c_str(), read_status_word);
                    current_state_ = AxisState::FAULT;
                    // 记录故障信息
                    break; // 立即跳出，不再执行后续逻辑
                }

                // 如果有启动请求但状态字不满足，检查是否状态异常
                if (start_manual_requested_ || start_auto_requested_) {
                    static int warning_counter = 0;
                    if (warning_counter++ % 1000 == 0) {
                        printf("轴 %s 等待状态字0x1637才能进入模式切换，当前状态字: 0x%04x\n", 
                            axis_name_.c_str(), read_status_word);
                    }
                    
                    // 新增：如果状态字表示故障或严重错误，才跳转回初始化
                    // bit3(Fault) = 故障状态, 0x0000 = 未初始化/通信中断
                    // 注意：0x0633(Switch on disabled)和0x0637(Switched on)是正常中间状态，不视为异常
                    if ((read_status_word & 0x0008) || read_status_word == 0x0000) {
                        printf("轴 %s 检测到故障/异常状态字0x%04x，自动跳转回初始化状态\n",
                            axis_name_.c_str(), read_status_word);
                        current_state_ = AxisState::INITIALIZING;

                        // 清除启动请求标志，避免重复触发
                        start_manual_requested_ = false;
                        start_auto_requested_ = false;
                    }
                }
            }
            break;
            
        case AxisState::MANUAL_MODE:
            // 手动模式处理
            // === 检测故障状态字 (bit3 Fault位置位) ===
            if (read_status_word & 0x0008) {
                printf("轴 %s 检测到故障状态字0x%04X，进入故障模式\n",
                    axis_name_.c_str(), read_status_word);
                current_state_ = AxisState::FAULT;
                break;
            }
            
            // === 处理停止请求（结束作业）===
            if (stop_requested_) {
                // 发送停止指令：保持当前位置
                EC_WRITE_S32(domain1_pd + off_target_position_, current_pos);
                EC_WRITE_U16(domain1_pd + control_word_, 0x0006);
                
                // 检查轴是否已停止（速度为0或状态字变化）
                // 这里简单处理：直接跳转，实际可根据需求添加停止确认
                stop_requested_ = false;
                current_state_ = AxisState::STOPPED;
                printf("轴 %s 收到停止请求，进入停止状态\n", axis_name_.c_str());
                break;
            }
            
            if (!(read_status_word == 0x1637 || read_status_word == 0x1237)) {
                current_state_ = AxisState::INITIALIZING;
                printf("轴 %s 退出手动模式\n", axis_name_.c_str());
            } else {
                handle_leisai_manual_operation(domain1_pd, current_pos);
            }
            break;
            
        case AxisState::AUTO_MODE:
            // 自动模式处理
            // === 检测故障状态字 (bit3 Fault位置位) ===
            if (read_status_word & 0x0008) {
                printf("轴 %s 检测到故障状态字0x%04X，进入故障模式\n",
                    axis_name_.c_str(), read_status_word);
                current_state_ = AxisState::FAULT;

                break;
            }
            
            // === 处理停止请求（结束作业）===
            if (stop_requested_) {
                // 发送停止指令：保持当前位置
                EC_WRITE_S32(domain1_pd + off_target_position_, current_pos);
                EC_WRITE_U16(domain1_pd + control_word_, 0x0006);
                
                stop_requested_ = false;
                current_state_ = AxisState::STOPPED;
                printf("轴 %s 收到停止请求，进入停止状态\n", axis_name_.c_str());
                break;
            }
            
            if (!(read_status_word == 0x1637 || read_status_word == 0x1237)) {
                current_state_ = AxisState::INITIALIZING;
                printf("轴 %s 退出自动模式\n", axis_name_.c_str());
            } else {
                handle_leisai_auto_operation(domain1_pd, current_pos);
            }
            break;
            
        case AxisState::STOPPED:
            // 停止状态：保持当前位置，等待回到INITIALIZING
            // === 检测故障状态字 (bit3 Fault位置位) ===
            if (read_status_word & 0x0008) {
                printf("轴 %s 检测到故障状态字0x%04X，进入故障模式\n",
                    axis_name_.c_str(), read_status_word);
                current_state_ = AxisState::FAULT;
                break;
            }
            
            EC_WRITE_S32(domain1_pd + off_target_position_, current_pos);
            EC_WRITE_U16(domain1_pd + control_word_, 0x0006);
            
            // +++ 关键修复：在STOPPED状态持续清理运动状态，确保不会继续执行原指令 +++
            displacement_updated_ = false;
            target_pulses_ = current_pos;
            jog_forward_requested_ = false;
            jog_reverse_requested_ = false;

            // 检查是否可以回到READY状态
            // if (read_status_word == 0x1637 && !global_data_blocked.load()) {
            if (read_status_word == 0x1637 ) {
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
            // if (error_code == 0x821b && fault_clearing_in_progress_) {
            //     handle_fault_clear(domain1_pd);
            //     // 故障清除完成后，回到初始化状态
            //     if (!fault_clearing_in_progress_) {
            //         current_state_ = AxisState::INITIALIZING;
            //         printf("轴 %s 821b故障清除完成, 回到初始化状态\n", axis_name_.c_str());
            //     }
            // }
            // // 原有的故障处理逻辑保持不变
            // else if (clear_fault_requested_ && current_state_ == AxisState::FAULT) {
            //     clear_fault_requested_ = false;
            //     fault_clearing_in_progress_ = true;
            //     fault_clear_step_ = 0;
            //     fault_clear_counter_ = 0;
            //     printf("轴 %s 开始清除故障流程\n", axis_name_.c_str());
            // }

            // if (fault_clearing_in_progress_) {
            //     handle_fault_clear(domain1_pd);
            // } else {
            //     EC_WRITE_U16(domain1_pd + control_word_, 0x0006);
            // }
            if (fault_clearing_in_progress_) {
                handle_fault_clear(domain1_pd);
                
                // 检查清除是否完成
                if (!fault_clearing_in_progress_) {
                    current_state_ = AxisState::INITIALIZING;
                    fault_clear_step_ = 0;  // 重置步骤
                    fault_clear_counter_ = 0;
                    current_error_code_ = 0; // 清除错误码，防止重新触发
                    printf("轴 %s 故障清除完成，回到初始化状态\n", axis_name_.c_str());
                }
            } 
            // 检查是否需要启动清除流程（仅当错误码存在或收到清除请求）
            else if ((current_error_code_ == 0x821b && error_code == 0x821b) || clear_fault_requested_) {
                fault_clearing_in_progress_ = true;
                fault_clear_step_ = 0;
                fault_clear_counter_ = 0;
                clear_fault_requested_ = false;
                printf("轴 %s 开始故障清除流程\n", axis_name_.c_str());
            }
            // 默认故障处理
            else {
                EC_WRITE_U16(domain1_pd + control_word_, 0x0006);
            }
            break;
    }    

    check_state_changes(read_status_word, error_code);
}

void LeisaiServoAxis::handle_leisai_initialization(uint8_t* domain1_pd, uint16_t read_status_word) {
    static unsigned int init_delay = 0;
    init_delay++;
    
    if (init_delay < 100) {
        return; // 修复：使用return而不是break
    }
    
    // 雷赛特有初始化序列
    if (read_status_word == 0x0250 || read_status_word == 0x0650 || 
        read_status_word == 0x0670 || read_status_word == 0x0770) {
        EC_WRITE_U16(domain1_pd + control_word_, 0x0006);
    } else if (read_status_word == 0x0631 || read_status_word == 0x0731) {
        EC_WRITE_U16(domain1_pd + control_word_, 0x0007);
    } else if (read_status_word == 0x0633 || read_status_word == 0x0733) {
        int32_t current_pos = EC_READ_S32(domain1_pd + off_actual_position_); // 修复：使用domain_pd
        EC_WRITE_S32(domain1_pd + off_target_position_, current_pos); // 修复：使用domain_pd
        EC_WRITE_U16(domain1_pd + control_word_, 0x001F);
        current_state_ = AxisState::READY;
        printf("轴 %s 进入就绪状态\n", axis_name_.c_str());
    } else if (read_status_word == 0x1637 || read_status_word == 0x1237 || 
               read_status_word == 0x16b7) {
        int32_t current_pos = EC_READ_S32(domain1_pd + off_actual_position_); // 修复：使用domain_pd
        EC_WRITE_S32(domain1_pd + off_target_position_, current_pos); // 修复：使用domain_pd
        current_state_ = AxisState::READY;
        printf("轴 %s 进入就绪状态\n", axis_name_.c_str());
    }
}
// 实现缺失的函数
void LeisaiServoAxis::handle_leisai_ready_state(uint8_t* domain1_pd, uint16_t status_word) {
    // 实现代码
    if (start_manual_requested_ && status_word == 0x1237) {
        start_manual_requested_ = false;
        operation_mode_ = OperationMode::MANUAL;
        current_state_ = AxisState::MANUAL_MODE;
    }
}

void LeisaiServoAxis::handle_leisai_manual_operation(uint8_t* domain1_pd, int32_t current_pos) {
    // 实现手动模式操作逻辑
    if (!position_initialized_) {
        joint_position_ = current_pos;
        initial_position_ = current_pos;
        target_pulses_ = current_pos;
        position_initialized_ = true;
        printf("轴 %s 手动模式位置初始化完成\n", axis_name_.c_str());
    }
    
    // 获取当前点动速度（线程安全）
    double current_jog_speed;
    {
        std::lock_guard<std::mutex> lock(speed_mutex_);
        current_jog_speed = jog_speed_;
    }
    
    // 处理点动控制 —— 根据轴计算模式选择对应的转换函数
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

void LeisaiServoAxis::handle_leisai_auto_operation(uint8_t* domain1_pd, int32_t current_pos) {
    // 实现手动模式操作逻辑
    if (!position_initialized_) {
        joint_position_ = current_pos;
        initial_position_ = current_pos;
        target_pulses_ = initial_position_;
        position_initialized_ = true;
        printf("轴 %s 自动模式位置初始化完成\n", axis_name_.c_str());
    }
    
    // 处理点动控制 —— 根据轴计算模式选择对应的转换函数
    // 获取当前点动速度（线程安全）
    double current_jog_speed;
    {
        std::lock_guard<std::mutex> lock(speed_mutex_);
        current_jog_speed = jog_speed_;
    }
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
        // 停止
        target_pulses_ = current_pos;
        jog_stop_requested_ = false;
    }
    
    if (displacement_updated_) {
        displacement_updated_ = false;
        target_reached_ = false;
        {
            std::lock_guard<std::mutex> lock(flag_mutex_);
            target_reached_flag_ = false; // 新运动开始，清除标志
        }
        // 绝对位置模式：直接计算目标脉冲数
        // target_pulses_ = initial_position_ + displacement_to_pulses(target_displacement_);
        target_pulses_ = displacement_to_pulses(target_displacement_);
        printf("轴 %s 绝对位置更新: %.3fmm -> 目标脉冲 %d (初始: %d, 当前: %d)\n", 
                axis_name_.c_str(), target_displacement_, target_pulses_, initial_position_, joint_position_);
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
            printf("轴 %s 已到达目标位置!(雷赛自动模式)\n", axis_name_.c_str());
        }
    }
}

void LeisaiServoAxis::handle_leisai_fault_state(uint8_t* domain1_pd, uint16_t error_code) {
    // 实现故障状态处理逻辑
    if (fault_clearing_in_progress_) {
        handle_fault_clear(domain1_pd);
    }
}

void LeisaiServoAxis::handle_fault_clear(uint8_t* domain1_pd) {
    fault_clear_counter_++;
    uint16_t current_status = EC_READ_U16(domain1_pd + status_word_);

    switch (fault_clear_step_) {
        case 0:
            // 步骤0：非通讯错误时，等待外部SDO写入0x2057完成
            // 通讯错误(0x821b)自动清除，无需额外操作
            if (current_error_code_ != 0x821b) {
                // 标记需要外部SDO处理，然后继续标准清除流程
                // 实际0x2057写入需在外部非实时线程中执行
                printf("轴 %s 非通讯错误(0x%04X)，需先写入0x2057=1再清除\n",
                       axis_name_.c_str(), current_error_code_);
            }
            // 短暂延迟确保外部SDO写入完成（如已触发）
            if (fault_clear_counter_ > 5) {
                fault_clear_step_ = 1;
                fault_clear_counter_ = 0;
            }
            break;

        case 1:
            // 步骤1：发送故障复位命令0x0080
            EC_WRITE_U16(domain1_pd + control_word_, 0x0080);
            printf("轴 %s 故障清除步骤1: 发送0x0080\n", axis_name_.c_str());
            if (fault_clear_counter_ > 10) {
                fault_clear_step_ = 2;
                fault_clear_counter_ = 0;
            }
            break;

        case 2:
            // 步骤2：写0x0000准备使能
            EC_WRITE_U16(domain1_pd + control_word_, 0x0000);
            printf("轴 %s 故障清除步骤2: 发送0x0000\n", axis_name_.c_str());
            if (fault_clear_counter_ > 5) {
                fault_clear_step_ = 3;
                fault_clear_counter_ = 0;
            }
            break;

        case 3:
            // 步骤3：写0x0006切换到准备开关ON
            EC_WRITE_U16(domain1_pd + control_word_, 0x0006);
            printf("轴 %s 故障清除步骤3: 发送0x0006\n", axis_name_.c_str());
            if (fault_clear_counter_ > 5) {
                fault_clear_step_ = 4;
                fault_clear_counter_ = 0;
            }
            break;

        case 4:
            // 步骤4：检查状态字，确认故障已清除
            if ((current_status & 0x0008) == 0) {
                fault_clearing_in_progress_ = false;
                int32_t current_pos = EC_READ_S32(domain1_pd + off_actual_position_);
                target_pulses_ = current_pos;
                joint_position_ = current_pos;
                initial_position_ = current_pos;
                printf("轴 %s 故障清除完成，状态字: 0x%04x\n",
                       axis_name_.c_str(), current_status);
            } else if (fault_clear_counter_ > 100) {
                fault_clearing_in_progress_ = false;
                printf("轴 %s 故障清除超时，当前状态字: 0x%04x\n",
                       axis_name_.c_str(), current_status);
            }
            break;
    }
}

uint16_t LeisaiServoAxis::get_error_code() const {
    return current_error_code_;
}

AxisState LeisaiServoAxis::get_current_state() const {
    return current_state_;
}

// 注：get_max_step() 已基类移除调用，自动模式速度现由 auto_speed_ 控制
// 如需为不同轴设置不同的自动速度，可在 YAML 中单独配置每个轴的 auto_speed 字段
