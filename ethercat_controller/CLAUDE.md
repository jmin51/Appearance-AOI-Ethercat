# EtherCAT 控制器架构文档

## 项目结构

```
ethercat_controller/
├── src/
│   ├── main.cpp                          # 主入口
│   ├── ethercat_node.cpp                 # EtherCAT 主节点
│   ├── servo_axis_base.cpp               # 伺服轴基类
│   ├── leisai_servo_axis.cpp             # 雷赛伺服轴实现
│   ├── huichuan_servo_axis.cpp           # 汇川伺服轴实现
│   ├── servo_axis_factory.cpp            # 伺服轴工厂
│   ├── business/
│   │   └── LayerCommandProcessor.cpp     # 层指令处理器
│   ├── io_modules/
│   │   ├── io_interface.cpp              # IO 模块接口
│   │   ├── lights_controller.cpp         # 灯光控制器（按钮灯+三色灯）
│   │   └── lights_controller.hpp         # 灯光控制器头文件
│   └── fault_manager/
│       ├── fault_management_system.cpp   # 故障管理系统（独立实现）
│       ├── fault_management_system.hpp
│       └── fault_codes.hpp               # 故障码表定义
├── include/
│   ├── ethercat_node.hpp
│   ├── servo_axis_base.hpp
│   ├── servo_axis_factory.hpp
│   ├── io_interface.hpp
│   └── LayerCommandProcessor.hpp
├── CMakeLists.txt
└── package.xml
```

## 故障管理系统架构

### 设计哲学
- **三级故障分类**：系统错误、轴错误、IO错误、通信错误、业务逻辑错误、模式错误、警告
- **自动日志捕获**：通过 ROS2 日志回调自动捕获 RCLCPP_ERROR 和 RCLCPP_WARN
- **实时故障发布**：通过 `/fault_code` 话题实时发布故障状态（无故障时不发布）
- **模式冲突处理**：手动模式下收到自动指令时，上报特定故障码

### 核心组件

#### 1. 故障码表 (fault_codes.hpp)
- **16位故障码结构**：高4位类别，次4位子类别，低8位具体代码
- **七大类故障**：
  - 0x1xxx: 系统错误（初始化、配置、资源、超时）
  - 0x2xxx: 轴错误（回零、运动、限位、过载、通信）
  - 0x3xxx: IO错误（数字输入/输出、模拟输入/输出）
  - 0x4xxx: 通信错误（EtherCAT、Modbus、ROS2、串口）
  - 0x5xxx: 业务逻辑错误（层指令、序列、状态）
  - 0x6xxx: 模式错误（模式冲突、转换失败）
  - 0x7xxx: 警告（性能、温度、电压）
- **遗留兼容**：保留 0x8xxx、0x9xxx、0x91xx 范围用于向后兼容

#### 2. 故障管理系统 (FaultManagementSystem)
- **独立实现**：完整的故障管理功能，不依赖 SimpleFaultManager
- **日志回调**：注册 ROS2 根记录器回调，自动转换 ERROR/WARN 日志为故障码
- **模式冲突处理**：`handle_auto_command_in_manual_mode()` 方法处理手动模式下的自动指令
- **可配置映射**：支持日志消息模式到故障码的映射规则
- **JSON 状态输出**：`get_fault_status_json()` 提供结构化故障状态
- **循环避免**：使用专用日志器名称避免日志回调循环

#### 3. 集成点 (ethercat_node.cpp)
- **初始化**：在节点初始化时创建 FaultManagementSystem 实例
- **自动捕获**：启用日志回调，自动监控所有错误和警告
- **模式检查**：在 `handle_control_command()` 中检查手动模式下的自动指令
- **故障发布**：通过 `/fault_code` 话题发布故障状态字符串

### 关键工作流程

#### 自动故障检测
```
RCLCPP_ERROR/WARN 日志 → ROS2 日志回调 → 故障管理系统 → 映射为故障码 → 发布到 /fault_code
```

#### 手动模式自动指令处理
```
收到 start_auto 命令 → 检查所有轴操作模式 → 发现轴处于手动模式 → 调用 handle_auto_command_in_manual_mode()
→ 添加故障码 FAULT_MODE_MANUAL_RECEIVED_AUTO (0x6201) → 发布故障状态 → 记录警告日志
```

### 故障码示例
- `0x1201`: 系统初始化失败
- `0x2201`: 轴回零失败
- `0x6201`: 手动模式下收到自动指令
- `0x7101`: 通用警告
- `0x8001`: 遗留轴错误

### 使用示例

```cpp
// 手动添加故障
fault_manager_->add_fault("axis1", fault_codes::FAULT_AXIS_HOMING_FAILED, "回零超时");

// 处理模式冲突
uint16_t fault_code = fault_manager_->handle_auto_command_in_manual_mode("axis2");

// 获取故障状态
std::string json_status = fault_manager_->get_fault_status_json();
```

### 配置与扩展

1. **添加新故障码**：在 `fault_codes.hpp` 中定义新常量
2. **扩展映射规则**：调用 `add_fault_mapping()` 添加日志模式到故障码的映射
3. **自定义发布格式**：重写 `publish_fault_status()` 方法
4. **集成其他日志源**：通过 `add_fault()` 方法直接添加故障

### 向后兼容
- 故障管理系统提供与先前版本兼容的 API，现有集成代码无需修改
- 遗留故障码范围 (0x8xxx, 0x9xxx, 0x91xx) 继续支持
- `is_error_code()` 函数同时支持新旧故障码分类

## 灯光控制系统架构

### 设计哲学
- **职责分离**：灯光控制独立于主流程，避免main.cpp臃肿
- **状态机驱动**：三色灯通过状态机管理（OFF → YELLOW_BLINK → GREEN_BLINK）
- **边沿触发**：按钮灯通过边沿检测触发，避免抖动
- **变化检测**：DO写入仅在状态变化时执行，减少Modbus通信

### 核心组件

#### 1. 灯光控制器 (lights_controller)
- **独立模块**：封装按钮灯和三色灯所有逻辑
- **线程安全**：通过互斥锁保护Modbus访问（与io_interface配合）
- **自动闪烁**：500ms周期自动切换黄灯/绿灯闪烁状态

#### 2. 状态定义
```cpp
enum TricolorLightState {
    LIGHT_OFF,          // 全部熄灭
    LIGHT_YELLOW_BLINK, // 黄灯闪烁（复位中）
    LIGHT_GREEN_BLINK,  // 绿灯闪烁（就绪）
    LIGHT_GREEN_ON      // 绿灯常亮（运行中）
};
```

#### 3. 控制流程
| 触发条件 | 按钮灯行为 | 三色灯行为 | 蜂鸣器 |
|----------|-----------|-----------|--------|
| 启动按钮 | 启动灯亮 | 绿灯常亮 | 停 |
| 复位按钮（按住3秒确认） | 复位灯亮 | 黄灯闪烁 | 响 |
| 暂停按钮 | 暂停灯亮 | 全部熄灭 | 停 |
| AL states 0x08 + 全部自动 | 复位灯灭 | 黄灯停闪，绿灯闪烁 | 停 |

### 关键接口

```cpp
// 初始化
void init_lights_controller();

// 主循环调用（每100ms）
void update_button_lights(bool start_btn, bool reset_btn, bool pause_btn);
void update_tricolor_lights();

// 系统就绪通知（由ethercat_node调用）
void notify_system_ready();
```

### 集成点

1. **main.cpp**：初始化后调用 `init_lights_controller()`，主循环中调用更新函数
2. **ethercat_node.cpp**：检测到 AL states 0x08 且全部轴自动时调用 `notify_system_ready()`
3. **io_interface.cpp**：通过互斥锁保护Modbus并发访问

---

## 设计决策记录

1. **为何选择日志回调而非宏替换**：
   - 无需修改现有代码中的 RCLCPP_ERROR/WARN 调用
   - 能够捕获第三方库产生的日志
   - 避免宏替换带来的编译复杂性

2. **为何使用16位故障码**：
   - 提供足够的分类空间（16个主类 × 16个子类 × 256个具体代码）
   - 与现有工业标准兼容
   - 便于十六进制阅读和调试

3. **为何保留字符串发布格式**：
   - 保持与现有监控系统兼容
   - 简化调试和日志分析
   - 可通过 JSON 输出提供结构化数据

4. **为何将故障映射设计为可配置**：
   - 适应不同应用场景的日志模式
   - 支持运行时动态更新映射规则
   - 便于测试和模拟特定故障场景

---

## 未来扩展方向

1. **故障历史记录**：添加时间戳和故障持续时间跟踪
2. **严重度分级**：在故障码中嵌入严重度信息（致命、错误、警告、信息）
3. **自动恢复策略**：根据故障类型触发预设的恢复动作
4. **远程监控接口**：通过 ROS2 服务提供故障查询和清除功能
5. **统计分析**：故障频率、MTBF（平均无故障时间）等指标计算

---

*文档最后更新：2026-03-14*
*对应架构版本：v2.0（故障管理系统集成）*