#ifndef FAULT_CODES_HPP
#define FAULT_CODES_HPP

#include <cstdint>

namespace fault_codes {

// 故障码类别掩码
constexpr uint16_t CATEGORY_MASK      = 0xF000;
constexpr uint16_t SUBCATEGORY_MASK   = 0x0F00;
constexpr uint16_t SPECIFIC_CODE_MASK = 0x00FF;

// 故障码类别（高4位）
enum FaultCategory : uint16_t {
    CATEGORY_SYSTEM        = 0x1000,  // 系统错误
    CATEGORY_AXIS          = 0x2000,  // 轴错误
    CATEGORY_IO            = 0x3000,  // IO错误
    CATEGORY_COMMUNICATION = 0x4000,  // 通信错误
    CATEGORY_BUSINESS      = 0x5000,  // 业务逻辑错误
    CATEGORY_MODE          = 0x6000,  // 模式错误
    CATEGORY_WARNING       = 0x7000,  // 警告
    CATEGORY_LEGACY_AXIS   = 0x8000,  // 遗留轴错误（向后兼容）
    CATEGORY_LEGACY_WARN   = 0x9000,  // 遗留警告（向后兼容）
    CATEGORY_LEGACY_ERROR  = 0x9100,  // 遗留系统错误（向后兼容）
};

// 系统错误子类别（次高4位）
enum SystemSubcategory : uint16_t {
    SYSTEM_GENERAL     = 0x0000,
    SYSTEM_INIT        = 0x0100,
    SYSTEM_CONFIG      = 0x0200,
    SYSTEM_RESOURCE    = 0x0300,
    SYSTEM_TIMEOUT     = 0x0400,
};

// 轴错误子类别
enum AxisSubcategory : uint16_t {
    AXIS_GENERAL       = 0x0000,
    AXIS_HOMING        = 0x0100,
    AXIS_MOTION        = 0x0200,
    AXIS_LIMIT         = 0x0300,
    AXIS_OVERLOAD      = 0x0400,
    AXIS_COMM          = 0x0500,  // 轴通信错误
};

// IO错误子类别
enum IOSubcategory : uint16_t {
    IO_GENERAL         = 0x0000,
    IO_DIGITAL_IN      = 0x0100,
    IO_DIGITAL_OUT     = 0x0200,
    IO_ANALOG_IN       = 0x0300,
    IO_ANALOG_OUT      = 0x0400,
};

// 通信错误子类别
enum CommSubcategory : uint16_t {
    COMM_GENERAL       = 0x0000,
    COMM_ETHERCAT      = 0x0100,
    COMM_MODBUS        = 0x0200,
    COMM_ROS2          = 0x0300,
    COMM_SERIAL        = 0x0400,
};

// 业务逻辑错误子类别
enum BusinessSubcategory : uint16_t {
    BUSINESS_GENERAL   = 0x0000,
    BUSINESS_LAYER     = 0x0100,
    BUSINESS_SEQUENCE  = 0x0200,
    BUSINESS_STATE     = 0x0300,
};

// 模式错误子类别
enum ModeSubcategory : uint16_t {
    MODE_GENERAL       = 0x0000,
    MODE_TRANSITION    = 0x0100,
    MODE_CONFLICT      = 0x0200,
};

// 警告子类别
enum WarningSubcategory : uint16_t {
    WARNING_GENERAL    = 0x0000,
    WARNING_PERFORMANCE = 0x0100,
    WARNING_TEMPERATURE = 0x0200,
    WARNING_VOLTAGE    = 0x0300,
};

// ============================================================================
// 具体故障码定义
// ============================================================================

// 系统错误 (0x1xxx)
constexpr uint16_t FAULT_SYSTEM_GENERAL_ERROR    = CATEGORY_SYSTEM | SYSTEM_GENERAL | 0x01;
constexpr uint16_t FAULT_SYSTEM_INIT_FAILED      = CATEGORY_SYSTEM | SYSTEM_INIT | 0x01;
constexpr uint16_t FAULT_SYSTEM_CONFIG_INVALID   = CATEGORY_SYSTEM | SYSTEM_CONFIG | 0x01;
constexpr uint16_t FAULT_SYSTEM_RESOURCE_EXHAUST = CATEGORY_SYSTEM | SYSTEM_RESOURCE | 0x01;
constexpr uint16_t FAULT_SYSTEM_TIMEOUT          = CATEGORY_SYSTEM | SYSTEM_TIMEOUT | 0x01;

// 轴错误 (0x2xxx)
constexpr uint16_t FAULT_AXIS_HOMING_FAILED      = CATEGORY_AXIS | AXIS_HOMING | 0x01;
constexpr uint16_t FAULT_AXIS_MOTION_ERROR       = CATEGORY_AXIS | AXIS_MOTION | 0x01;
constexpr uint16_t FAULT_AXIS_POSITION_LIMIT     = CATEGORY_AXIS | AXIS_LIMIT | 0x01;
constexpr uint16_t FAULT_AXIS_OVERLOAD           = CATEGORY_AXIS | AXIS_OVERLOAD | 0x01;
constexpr uint16_t FAULT_AXIS_COMM_LOST          = CATEGORY_AXIS | AXIS_COMM | 0x01;

// IO错误 (0x3xxx)
constexpr uint16_t FAULT_IO_DI_READ_FAILED       = CATEGORY_IO | IO_DIGITAL_IN | 0x01;
constexpr uint16_t FAULT_IO_DO_WRITE_FAILED      = CATEGORY_IO | IO_DIGITAL_OUT | 0x01;
constexpr uint16_t FAULT_IO_AI_READ_FAILED       = CATEGORY_IO | IO_ANALOG_IN | 0x01;
constexpr uint16_t FAULT_IO_AO_WRITE_FAILED      = CATEGORY_IO | IO_ANALOG_OUT | 0x01;

// 通信错误 (0x4xxx)
constexpr uint16_t FAULT_COMM_ETHERCAT_LOST      = CATEGORY_COMMUNICATION | COMM_ETHERCAT | 0x01;
constexpr uint16_t FAULT_COMM_MODBUS_TIMEOUT     = CATEGORY_COMMUNICATION | COMM_MODBUS | 0x01;
constexpr uint16_t FAULT_COMM_ROS2_PUB_FAILED    = CATEGORY_COMMUNICATION | COMM_ROS2 | 0x01;
constexpr uint16_t FAULT_COMM_SERIAL_ERROR       = CATEGORY_COMMUNICATION | COMM_SERIAL | 0x01;

// 业务逻辑错误 (0x5xxx)
constexpr uint16_t FAULT_BUSINESS_LAYER_INVALID  = CATEGORY_BUSINESS | BUSINESS_LAYER | 0x01;
constexpr uint16_t FAULT_BUSINESS_SEQUENCE_ERROR = CATEGORY_BUSINESS | BUSINESS_SEQUENCE | 0x01;
constexpr uint16_t FAULT_BUSINESS_STATE_CONFLICT = CATEGORY_BUSINESS | BUSINESS_STATE | 0x01;

// 模式错误 (0x6xxx) - 特别关注手动模式相关
constexpr uint16_t FAULT_MODE_MANUAL_RECEIVED_AUTO = CATEGORY_MODE | MODE_CONFLICT | 0x01;
constexpr uint16_t FAULT_MODE_AUTO_RECEIVED_MANUAL = CATEGORY_MODE | MODE_CONFLICT | 0x02;
constexpr uint16_t FAULT_MODE_TRANSITION_FAILED    = CATEGORY_MODE | MODE_TRANSITION | 0x01;

// 警告 (0x7xxx)
constexpr uint16_t WARNING_GENERAL_CODE         = CATEGORY_WARNING | WARNING_GENERAL | 0x01;
constexpr uint16_t WARNING_PERFORMANCE_SLOW      = CATEGORY_WARNING | WARNING_PERFORMANCE | 0x01;
constexpr uint16_t WARNING_TEMPERATURE_HIGH      = CATEGORY_WARNING | WARNING_TEMPERATURE | 0x01;
constexpr uint16_t WARNING_VOLTAGE_LOW           = CATEGORY_WARNING | WARNING_VOLTAGE | 0x01;

// 遗留错误码（向后兼容）
constexpr uint16_t LEGACY_FAULT_AXIS_BASE        = 0x8000;
constexpr uint16_t LEGACY_WARNING_BASE           = 0x9000;
constexpr uint16_t LEGACY_SYSTEM_ERROR_BASE      = 0x9100;

// ============================================================================
// 工具函数
// ============================================================================

constexpr bool is_error_code(uint16_t code) {
    uint16_t category = code & CATEGORY_MASK;
    return category == CATEGORY_SYSTEM ||
           category == CATEGORY_AXIS ||
           category == CATEGORY_IO ||
           category == CATEGORY_COMMUNICATION ||
           category == CATEGORY_BUSINESS ||
           category == CATEGORY_MODE ||
           category == CATEGORY_LEGACY_AXIS ||
           category == CATEGORY_LEGACY_ERROR;
}

constexpr bool is_warning_code(uint16_t code) {
    uint16_t category = code & CATEGORY_MASK;
    return category == CATEGORY_WARNING ||
           category == CATEGORY_LEGACY_WARN;
}

constexpr const char* category_to_string(uint16_t code) {
    uint16_t category = code & CATEGORY_MASK;
    switch (category) {
        case CATEGORY_SYSTEM:        return "系统错误";
        case CATEGORY_AXIS:          return "轴错误";
        case CATEGORY_IO:            return "IO错误";
        case CATEGORY_COMMUNICATION: return "通信错误";
        case CATEGORY_BUSINESS:      return "业务逻辑错误";
        case CATEGORY_MODE:          return "模式错误";
        case CATEGORY_WARNING:       return "警告";
        case CATEGORY_LEGACY_AXIS:   return "遗留轴错误";
        case CATEGORY_LEGACY_WARN:   return "遗留警告";
        case CATEGORY_LEGACY_ERROR:  return "遗留系统错误";
        default:                     return "未知类别";
    }
}

} // namespace fault_codes

#endif // FAULT_CODES_HPP