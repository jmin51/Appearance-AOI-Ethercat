#include "servo_axis_factory.hpp"
#include <stdexcept>

std::shared_ptr<ServoAxisBase> ServoAxisFactory::create_servo_axis(
    DriveBrand brand, const std::string& name, uint16_t position, 
    AxisType axis_type, uint32_t product_code, double gear_ratio, AxisCalcMode calc_mode) {
    // 如果没有指定产品号，使用默认值
    if (product_code == 0) {
        product_code = get_default_product_code(brand);
    }
    
    switch (brand) {
        case DriveBrand::LEISAI:
            return std::make_shared<LeisaiServoAxis>(name, position, axis_type, product_code, gear_ratio, calc_mode);
        case DriveBrand::HUICHUAN:
            return std::make_shared<HuichuanServoAxis>(name, position, axis_type, product_code, gear_ratio, calc_mode);
        default:
            throw std::invalid_argument("未知的驱动器品牌");
    }
}

uint32_t ServoAxisFactory::get_default_product_code(DriveBrand brand) {
    switch (brand) {
        case DriveBrand::LEISAI:
            return LEISAI_PRODUCT_CODE_DEFAULT;
        case DriveBrand::HUICHUAN:
            return HUICHUAN_PRODUCT_CODE_DEFAULT;
        default:
            return 0;
    }
}

DriveBrand ServoAxisFactory::string_to_brand(const std::string& brand_str) {
    if (brand_str == "leisai" || brand_str == "LEISAI" || brand_str == "雷赛") {
        return DriveBrand::LEISAI;
    } else if (brand_str == "huichuan" || brand_str == "HUICHUAN" || brand_str == "汇川") {
        return DriveBrand::HUICHUAN;
    } else {
        throw std::invalid_argument("不支持的驱动器品牌: " + brand_str);
    }
}

std::string ServoAxisFactory::brand_to_string(DriveBrand brand) {
    switch (brand) {
        case DriveBrand::LEISAI: return "LEISAI";
        case DriveBrand::HUICHUAN: return "HUICHUAN";
        default: return "UNKNOWN";
    }
}

AxisType ServoAxisFactory::string_to_axis_type(const std::string& type_str) {
    if (type_str == "AXIS1" || type_str == "axis1") {
        return AxisType::AXIS1;
    } else if (type_str == "AXIS2" || type_str == "axis2") {
        return AxisType::AXIS2;
    } else {
        throw std::invalid_argument("不支持的轴类型: " + type_str);
    }
}

std::string ServoAxisFactory::axis_type_to_string(AxisType type) {
    switch (type) {
        case AxisType::AXIS1: return "AXIS1";
        case AxisType::AXIS2: return "AXIS2";
        default: return "UNKNOWN";
    }
}

AxisCalcMode ServoAxisFactory::string_to_calc_mode(const std::string& mode_str) {
    if (mode_str == "ANGLE" || mode_str == "angle") {
        return AxisCalcMode::ANGLE;
    } else if (mode_str == "DISPLACEMENT" || mode_str == "displacement") {
        return AxisCalcMode::DISPLACEMENT;
    } else {
        throw std::invalid_argument("不支持的计算模式: " + mode_str);
    }
}

std::string ServoAxisFactory::calc_mode_to_string(AxisCalcMode mode) {
    switch (mode) {
        case AxisCalcMode::ANGLE: return "ANGLE";
        case AxisCalcMode::DISPLACEMENT: return "DISPLACEMENT";
        default: return "UNKNOWN";
    }
}