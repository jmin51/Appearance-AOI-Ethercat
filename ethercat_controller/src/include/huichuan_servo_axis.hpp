#ifndef HUICHUAN_SERVO_AXIS_HPP
#define HUICHUAN_SERVO_AXIS_HPP

#include "servo_axis_base.hpp"
#include <ecrt.h>


// 汇川伺服驱动器配置
#define HUICHUAN_VENDOR_ID 0x00100000
#define HUICHUAN_PRODUCT_CODE_DEFAULT 0x000c0112 // 默认产品号（0x000c010d为另一型号）

// 汇川PDO配置
extern ec_pdo_entry_info_t huichuan_slave_pdo_entries[];
extern ec_pdo_info_t huichuan_slave_pdos[];
extern ec_sync_info_t huichuan_slave_syncs[];

class HuichuanServoAxis : public ServoAxisBase {
public:
    HuichuanServoAxis(const std::string& name, uint16_t position, AxisType axis_type, 
                      uint32_t product_code = HUICHUAN_PRODUCT_CODE_DEFAULT,
                      double gear_ratio = 1.0,
                      AxisCalcMode calc_mode = AxisCalcMode::DISPLACEMENT);
    virtual ~HuichuanServoAxis() = default;

    // 实现基类纯虚函数
    void configure(ec_master_t* master) override;
    void register_pdo_entries(ec_pdo_entry_reg_t* reg_list, int& index) override;
    void handle_state_machine(uint8_t* domain1_pd) override;
    // 添加纯虚函数声明
    AxisState get_current_state() const override;
    uint16_t get_error_code() const override;

    // 汇川特有功能
    void set_huichuan_specific_parameter(double param);
    double get_huichuan_specific_parameter() const;
    void complete_homing_sequence(uint8_t* domain1_pd);
    // 获取产品号
    uint32_t get_product_code() const override { return product_code_; }

private:
    // 汇川特有实现
    void handle_huichuan_initialization(uint8_t* domain1_pd, uint16_t status_word);
    void handle_huichuan_homing(uint8_t* domain1_pd, int32_t current_pos);
    void handle_huichuan_manual_operation(uint8_t* domain1_pd, int32_t current_pos);
    void handle_huichuan_auto_operation(uint8_t* domain1_pd, int32_t current_pos);
    
    uint32_t product_code_;          // 存储实际使用的产品号
    double huichuan_specific_param_;
    // 添加缺失的成员变量
    int32_t home_target_position_;
    unsigned int init_delay_ = 0;    // 每轴独立初始化延时计数器
};

#endif // HUICHUAN_SERVO_AXIS_HPP