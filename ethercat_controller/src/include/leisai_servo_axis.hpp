#ifndef LEISAI_SERVO_AXIS_HPP
#define LEISAI_SERVO_AXIS_HPP

#include "servo_axis_base.hpp"
#include <ecrt.h>
#include <map>
#include <memory>

// 雷赛伺服驱动器配置
#define LEISAI_VENDOR_ID 0x00004321
#define LEISAI_PRODUCT_CODE_1 0x0000a400
#define LEISAI_PRODUCT_CODE_2 0x00002700
#define LEISAI_PRODUCT_CODE_3 0x0000a500
// 默认产品号（可根据需要选择其中一个作为默认）
#define LEISAI_PRODUCT_CODE_DEFAULT LEISAI_PRODUCT_CODE_1

// 雷赛PDO配置
extern ec_pdo_entry_info_t leisai_slave_pdo_entries[];
extern ec_pdo_info_t leisai_slave_pdos[];
extern ec_sync_info_t leisai_slave_syncs[];

class LeisaiServoAxis : public ServoAxisBase {
public:
    LeisaiServoAxis(const std::string& name, uint16_t position, AxisType axis_type, 
                    uint32_t product_code = LEISAI_PRODUCT_CODE_DEFAULT, 
                    double gear_ratio = 1.0, 
                    AxisCalcMode calc_mode = AxisCalcMode::DISPLACEMENT);
    virtual ~LeisaiServoAxis() = default;

    // 实现基类纯虚函数
    void configure(ec_master_t* master) override;
    void register_pdo_entries(ec_pdo_entry_reg_t* reg_list, int& index) override;
    void handle_state_machine(uint8_t* domain1_pd) override;
    // 添加纯虚函数声明
    AxisState get_current_state() const override;
    uint16_t get_error_code() const override;

    // 雷赛特有功能
    void set_leisai_specific_parameter(int param);
    int get_leisai_specific_parameter() const;
    // 添加产品号获取函数
    uint32_t get_product_code() const override { return product_code_; }

private:
    // 雷赛特有实现
    void handle_leisai_initialization(uint8_t* domain1_pd, uint16_t read_status_word);
    void handle_leisai_ready_state(uint8_t* domain1_pd, uint16_t status_word);
    void handle_leisai_manual_operation(uint8_t* domain1_pd, int32_t current_pos);
    void handle_leisai_auto_operation(uint8_t* domain1_pd, int32_t current_pos);
    void handle_leisai_fault_state(uint8_t* domain1_pd, uint16_t error_code);

    // 重写故障清除 - 雷赛轴特有处理（区分通讯错误与其他故障）
    void handle_fault_clear(uint8_t* domain1_pd) override;

    int leisai_specific_param_;
    uint32_t product_code_;  // 存储实际使用的产品号
};

#endif // LEISAI_SERVO_AXIS_HPP