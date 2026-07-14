#ifndef FAULT_MANAGEMENT_SYSTEM_HPP
#define FAULT_MANAGEMENT_SYSTEM_HPP

#include "fault_codes.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <map>
#include <string>
#include <functional>
#include <mutex>
#include <sstream>
#include <iomanip>

namespace fault_management {

/**
 * @brief 故障管理系统 - 独立的故障管理
 * 
 * 该系统提供：
 * 1. 手动添加故障的API
 * 2. 手动模式下自动指令的特殊处理
 * 3. 故障状态发布（无故障时不发布）
 */
class FaultManagementSystem {
public:
    /**
     * @brief 构造函数
     * @param node ROS2节点指针
     * @param fault_topic 故障发布话题名称（默认"/fault_code"）
     */
    explicit FaultManagementSystem(rclcpp::Node* node, 
                                  const std::string& fault_topic = "/fault_code");
    
    ~FaultManagementSystem();
    
    /**
     * @brief 初始化故障管理系统
     */
    void initialize();
    
    // ==================== 故障管理API ====================
    
    /**
     * @brief 手动添加故障
     * @param source 故障源
     * @param code 故障码（使用fault_codes中的定义）
     * @param description 故障描述
     */
    void add_fault(const std::string& source, uint16_t code, 
                   const std::string& description = "");
    
    /**
     * @brief 添加轴故障
     * @param axis_name 轴名称
     * @param fault_code 故障码
     * @param description 故障描述
     */
    void add_axis_fault(const std::string& axis_name, uint16_t fault_code, 
                        const std::string& description = "");
    
    /**
     * @brief 添加系统警告
     * @param warning_msg 警告消息
     */
    void add_system_warning(const std::string& warning_msg);
    
    /**
     * @brief 添加系统错误
     * @param error_msg 错误消息
     */
    void add_system_error(const std::string& error_msg);
    
    /**
     * @brief 清除特定故障
     * @param source 故障源
     * @param code 故障码（0表示清除该源的所有故障）
     */
    void clear_fault(const std::string& source, uint16_t code = 0);
    
    /**
     * @brief 清除所有故障
     */
    void clear_all_faults();
    
    /**
     * @brief 检查是否有活动故障
     * @return true 如果有活动故障
     */
    bool has_active_faults() const;
    
    /**
     * @brief 获取故障字符串表示
     * @return 故障字符串
     */
    std::string get_fault_string() const;
    
    /**
     * @brief 发布故障状态
     */
    void publish_fault_status();
    
    /**
     * @brief 设置故障发布器
     * @param pub 发布器共享指针
     */
    void set_fault_publisher(rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub);
    
    // ==================== 高级功能 ====================
    
    /**
     * @brief 添加故障并立即发布
     * @param source 故障源
     * @param code 故障码
     * @param description 故障描述
     */
    void add_fault_and_publish(const std::string& source, uint16_t code,
                               const std::string& description = "");
    
    /**
     * @brief 处理手动模式下接收自动指令的情况
     * @param axis_name 轴名称（可选）
     * @return 返回故障码（FAULT_MODE_MANUAL_RECEIVED_AUTO）
     */
    uint16_t handle_auto_command_in_manual_mode(const std::string& axis_name = "");
    
    /**
     * @brief 获取当前故障状态（JSON格式）
     * @return JSON字符串
     */
    std::string get_fault_status_json() const;

private:
    /**
     * @brief 生成故障键（内部使用，调用者需持有锁）
     * @param source 故障源
     * @param code 故障码
     * @return 故障键字符串
     */
    std::string generate_fault_key(const std::string& source, uint16_t code) const;
    
    /**
     * @brief 内部获取故障字符串（调用者需持有锁）
     * @return 故障字符串
     */
    std::string get_fault_string_internal() const;
    
    /**
     * @brief 内部记录故障
     * @param source 故障源
     * @param code 故障码
     * @param description 故障描述
     * @param is_error 是否为错误（true）或警告（false）
     */
    void log_fault_internal(const std::string& source, uint16_t code,
                           const std::string& description, bool is_error = true);
    
    rclcpp::Node* node_;
    std::string fault_topic_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr fault_publisher_;
    
    // 故障存储
    mutable std::mutex fault_mutex_;
    std::map<std::string, uint16_t> fault_map_;  // 故障键 -> 故障码
};

} // namespace fault_management

#endif // FAULT_MANAGEMENT_SYSTEM_HPP