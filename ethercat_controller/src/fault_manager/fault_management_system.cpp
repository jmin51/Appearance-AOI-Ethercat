#include "fault_management_system.hpp"
#include <rclcpp/logging.hpp>
#include <sstream>
#include <iomanip>

namespace fault_management {

using namespace std::chrono_literals;

// ============================================================================
// 构造函数与析构函数
// ============================================================================

FaultManagementSystem::FaultManagementSystem(rclcpp::Node* node, 
                                             const std::string& fault_topic)
    : node_(node)
    , fault_topic_(fault_topic) {
    
    // 创建故障发布器
    fault_publisher_ = node_->create_publisher<std_msgs::msg::String>(
        fault_topic_, rclcpp::QoS(10).reliable());
}

FaultManagementSystem::~FaultManagementSystem() {
    // 无需清理
}

// ============================================================================
// 公共方法实现
// ============================================================================

void FaultManagementSystem::initialize() {
    RCLCPP_INFO(node_->get_logger(), "故障管理系统初始化完成");
}

void FaultManagementSystem::add_fault(const std::string& source, uint16_t code,
                                      const std::string& description) {
    std::lock_guard<std::mutex> lock(fault_mutex_);
    std::string key = generate_fault_key(source, code);
    
    if (fault_map_.find(key) == fault_map_.end()) {
        fault_map_[key] = code;
        bool is_error = fault_codes::is_error_code(code);
        log_fault_internal(source, code, description, is_error);
        publish_fault_status();
    }
}

void FaultManagementSystem::add_axis_fault(const std::string& axis_name, uint16_t fault_code, 
                                          const std::string& description) {
    add_fault(axis_name, fault_code, description.empty() ? "轴故障" : description);
}

// 预定义的系统警告故障码映射
uint16_t get_system_warning_code(const std::string& warning_msg) {
    // DI/DO模块相关
    if (warning_msg.find("DI模块已禁用") != std::string::npos) return 0x9001;
    if (warning_msg.find("DO模块已禁用") != std::string::npos) return 0x9002;
    
    // 板宽相关警告
    if (warning_msg.find("板宽") != std::string::npos && warning_msg.find("超出分辨率") != std::string::npos) return 0x9003;
    
    // 模式相关警告
    if (warning_msg.find("手动模式下收到自动指令") != std::string::npos) return 0x9004;
    if (warning_msg.find("未知控制命令") != std::string::npos) return 0x9005;
    if (warning_msg.find("IO监控线程已在运行") != std::string::npos) return 0x9006;
    
    // 默认：未定义的警告使用通用码
    return 0x9000;
}

// 预定义的系统错误故障码映射
uint16_t get_system_error_code(const std::string& error_msg) {
    // 轴相关错误
    if (error_msg.find("未找到轴") != std::string::npos) return 0x9101;
    if (error_msg.find("位移指令解析失败") != std::string::npos) return 0x9102;
    if (error_msg.find("无效指令格式") != std::string::npos) return 0x9103;
    if (error_msg.find("轴名或值为空") != std::string::npos) return 0x9104;
    if (error_msg.find("数值转换失败") != std::string::npos) return 0x9105;
    if (error_msg.find("设置轴") != std::string::npos && error_msg.find("点动速度失败") != std::string::npos) return 0x9106;
    if (error_msg.find("速度值转换失败") != std::string::npos) return 0x9107;
    
    // Modbus/IO相关错误
    if (error_msg.find("Modbus初始化失败") != std::string::npos) return 0x9108;
    if (error_msg.find("创建IO监控线程失败") != std::string::npos) return 0x9109;
    
    // 层指令相关错误
    if (error_msg.find("层指令处理器未初始化") != std::string::npos) return 0x910A;
    
    // DO控制相关错误
    if (error_msg.find("DO控制命令解析失败") != std::string::npos) return 0x910B;
    if (error_msg.find("DO控制失败") != std::string::npos) return 0x910C;
    
    // 点动相关错误
    if (error_msg.find("点动速度命令解析失败") != std::string::npos) return 0x910D;
    if (error_msg.find("轴名或速度值为空") != std::string::npos) return 0x910E;
    
    // 板宽相关错误
    if (error_msg.find("无效板宽") != std::string::npos) return 0x910F;
    if (error_msg.find("未找到axis3") != std::string::npos) return 0x9110;
    if (error_msg.find("未找到axis4") != std::string::npos) return 0x9111;
    if (error_msg.find("未找到axis5") != std::string::npos) return 0x9112;
    
    // 默认：未定义的错误使用通用码
    return 0x9100;
}

void FaultManagementSystem::add_system_warning(const std::string& warning_msg) {
    // 使用预定义的固定故障码映射
    uint16_t warning_code = get_system_warning_code(warning_msg);
    add_fault("system_warning", warning_code, warning_msg);
}

void FaultManagementSystem::add_system_error(const std::string& error_msg) {
    // 使用预定义的固定故障码映射
    uint16_t error_code = get_system_error_code(error_msg);
    add_fault("system_error", error_code, error_msg);
}

void FaultManagementSystem::clear_fault(const std::string& source, uint16_t code) {
    std::lock_guard<std::mutex> lock(fault_mutex_);
    
    if (code == 0) {
        auto it = fault_map_.begin();
        while (it != fault_map_.end()) {
            size_t colon_pos = it->first.find(':');
            if (colon_pos != std::string::npos) {
                std::string fault_source = it->first.substr(0, colon_pos);
                if (fault_source == source) {
                    RCLCPP_INFO(node_->get_logger(), "清除故障: %s", source.c_str());
                    it = fault_map_.erase(it);
                } else {
                    ++it;
                }
            } else {
                ++it;
            }
        }
    } else {
        std::string key = generate_fault_key(source, code);
        auto it = fault_map_.find(key);
        if (it != fault_map_.end()) {
            RCLCPP_INFO(node_->get_logger(), "清除故障: %s (0x%04X)", source.c_str(), code);
            fault_map_.erase(it);
        }
    }
    
    publish_fault_status();
}

void FaultManagementSystem::clear_all_faults() {
    std::lock_guard<std::mutex> lock(fault_mutex_);
    fault_map_.clear();
    // 清除后发布更新状态（发布 "0" 表示无故障）
    publish_fault_status();
}

bool FaultManagementSystem::has_active_faults() const {
    std::lock_guard<std::mutex> lock(fault_mutex_);
    return !fault_map_.empty();
}

std::string FaultManagementSystem::get_fault_string() const {
    std::lock_guard<std::mutex> lock(fault_mutex_);
    
    if (fault_map_.empty()) {
        return "0";
    }
    
    std::stringstream ss;
    bool first = true;
    
    for (const auto& pair : fault_map_) {
        if (!first) ss << ",";
        first = false;
        
        size_t colon_pos = pair.first.find(':');
        if (colon_pos != std::string::npos) {
            std::string source = pair.first.substr(0, colon_pos);
            ss << source << ":" << "0x" 
               << std::hex << std::setw(4) << std::setfill('0') 
               << pair.second;
        }
    }
    
    return ss.str();
}

void FaultManagementSystem::publish_fault_status() {
    if (!fault_publisher_) {
        RCLCPP_WARN(node_->get_logger(), "故障发布器未设置");
        return;
    }
    
    auto msg = std_msgs::msg::String();
    msg.data = get_fault_string_internal();
    fault_publisher_->publish(msg);
}

void FaultManagementSystem::set_fault_publisher(rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub) {
    fault_publisher_ = pub;
}

void FaultManagementSystem::add_fault_and_publish(const std::string& source, uint16_t code,
                                                  const std::string& description) {
    add_fault(source, code, description);
    // add_fault内部已发布
}

uint16_t FaultManagementSystem::handle_auto_command_in_manual_mode(const std::string& axis_name) {
    std::string source = axis_name.empty() ? "system_mode" : axis_name;
    std::string description = "手动模式下收到自动模式指令，请手动切换模式旋钮";
    
    // 使用新的故障码映射 0x9004（替换旧的 0x6201）
    uint16_t fault_code = 0x9004;
    
    // === 故障去重：检查是否已存在相同故障，避免重复上报 ===
    std::lock_guard<std::mutex> lock(fault_mutex_);
    std::string key = generate_fault_key(source, fault_code);
    if (fault_map_.find(key) != fault_map_.end()) {
        // 故障已存在，静默返回（不重复日志，不重复发布）
        return fault_code;
    }
    
    // 首次上报：添加到故障映射
    fault_map_[key] = fault_code;
    log_fault_internal(source, fault_code, description, false);
    publish_fault_status();
    
    // 返回故障码，供调用者使用
    return fault_code;
}

std::string FaultManagementSystem::get_fault_status_json() const {
    std::lock_guard<std::mutex> lock(fault_mutex_);
    
    std::stringstream ss;
    ss << "{\"faults\":[";
    
    bool first = true;
    for (const auto& pair : fault_map_) {
        if (!first) ss << ",";
        first = false;
        
        size_t colon_pos = pair.first.find(':');
        if (colon_pos != std::string::npos) {
            std::string source = pair.first.substr(0, colon_pos);
            uint16_t code = pair.second;
            
            // 生成十六进制字符串
            std::stringstream hex_ss;
            hex_ss << "0x" << std::hex << std::setw(4) << std::setfill('0') << code;
            
            ss << "{";
            ss << "\"source\":\"" << source << "\",";
            ss << "\"code\":" << code << ",";
            ss << "\"code_hex\":\"" << hex_ss.str() << "\",";
            ss << "\"category\":\"" << fault_codes::category_to_string(code) << "\",";
            ss << "\"is_error\":" << (fault_codes::is_error_code(code) ? "true" : "false");
            ss << "}";
        }
    }
    
    ss << "],\"has_active_faults\":" << (fault_map_.empty() ? "false" : "true") << ",";
    ss << "\"timestamp\":" << static_cast<int>(node_->now().seconds());
    ss << "}";
    
    return ss.str();
}

// ============================================================================
// 私有方法实现
// ============================================================================

std::string FaultManagementSystem::generate_fault_key(const std::string& source, uint16_t code) const {
    std::stringstream ss;
    ss << source << ":" << std::hex << code;
    return ss.str();
}

std::string FaultManagementSystem::get_fault_string_internal() const {
    // 调用者必须已持有 fault_mutex_ 锁
    
    if (fault_map_.empty()) {
        return "0";
    }
    
    std::stringstream ss;
    bool first = true;
    
    for (const auto& pair : fault_map_) {
        if (!first) ss << ",";
        first = false;
        
        size_t colon_pos = pair.first.find(':');
        if (colon_pos != std::string::npos) {
            std::string source = pair.first.substr(0, colon_pos);
            ss << source << ":" << "0x" 
               << std::hex << std::setw(4) << std::setfill('0') 
               << pair.second;
        }
    }
    
    return ss.str();
}

void FaultManagementSystem::log_fault_internal(const std::string& source, uint16_t code,
                                              const std::string& description, bool is_error) {
    if (is_error) {
        RCLCPP_ERROR(node_->get_logger(), "故障: %s (0x%04X) - %s", 
                    source.c_str(), code, description.c_str());
    } else {
        RCLCPP_WARN(node_->get_logger(), "警告: %s (0x%04X) - %s", 
                   source.c_str(), code, description.c_str());
    }
}

} // namespace fault_management
