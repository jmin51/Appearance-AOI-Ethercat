#ifndef SERVO_AXIS_FACTORY_HPP
#define SERVO_AXIS_FACTORY_HPP

#include "servo_axis_base.hpp"
#include "leisai_servo_axis.hpp"
#include "huichuan_servo_axis.hpp"
#include <memory>
#include <string>

class ServoAxisFactory {
public:
    static std::shared_ptr<ServoAxisBase> create_servo_axis(
        DriveBrand brand,
        const std::string& name,
        uint16_t position,
        AxisType axis_type = AxisType::AXIS1,
        uint32_t product_code = 0,  // 0表示使用默认值
        double gear_ratio = 1.0,  // 减速比参数
        AxisCalcMode calc_mode = AxisCalcMode::DISPLACEMENT  // 计算模式：位移/角度
    );
    
    static DriveBrand string_to_brand(const std::string& brand_str);
    static std::string brand_to_string(DriveBrand brand);

    // 新增：AxisType 字符串转换
    static AxisType string_to_axis_type(const std::string& type_str);
    static std::string axis_type_to_string(AxisType type);

    // 新增：AxisCalcMode 字符串转换
    static AxisCalcMode string_to_calc_mode(const std::string& mode_str);
    static std::string calc_mode_to_string(AxisCalcMode mode);

    // 新增：根据品牌获取默认产品号
    static uint32_t get_default_product_code(DriveBrand brand);
};

#endif // SERVO_AXIS_FACTORY_HPP