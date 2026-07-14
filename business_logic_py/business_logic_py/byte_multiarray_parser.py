#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import MultiArrayLayout, MultiArrayDimension, ByteMultiArray, String, Int8, Empty, Float64, Bool
import struct
from enum import Enum
import threading
import time

class CommandType(Enum):
    # 根据图片中的指令码定义
    START_OPERATION = 0x0105      # 开始作业 /control_command -> start_auto
    NOTIFY_STORAGE = 0x0101       # 通知存放 /warehouse_start
    NOTIFY_RETRIEVAL = 0x0103     # 通知取出 /outbound_start
    END_OPERATION = 0x0107        # 结束作业 /warehouse_stop /outbound_stop
    AXIS_JOG = 0x010D             # 轴点动 /jog_command
    AXIS_STOP = 0x010F            # 轴停止 /jog_command
    WRITE_IO = 0x0115             # 写IO /do_control
    CLEAR_SYSTEM_FAULT = 0x0117   # 告警和错误清除 /fault_code
    CLEAR_AXIS_FAULT = 0x011B     # 清除轴故障 /control_command

    # 原有的指令码定义（不在图片中的）
    SYSTEM_CONTROL = 0x01
    JOG_SPEED = 0x03
    POSITION_CONTROL = 0x04
    LAYER_COMMAND = 0x05
    BOARD_WIDTH = 0x0B
    
    # 新增：IO状态更新指令
    UPDATE_DI_STATUS = 0x0111  # 更新输入IO状态
    UPDATE_DO_STATUS = 0x0113  # 更新输出IO状态

# 故障码定义（业务逻辑层）
class FaultCode(Enum):
    """故障码定义 - 高字节0xE0表示业务逻辑层错误"""
    NO_FAULT = 0x0000           # 无故障
    BOARD_WIDTH_TIMEOUT = 0xE001  # 板宽调整超时
    
class ByteMultiArrayParser(Node):
    def __init__(self):
        super().__init__('byte_multiarray_parser')
        
        # 订阅统一的ByteMultiArray话题
        self.unified_sub = self.create_subscription(
            ByteMultiArray,
            '/integrated_control',
            self.unified_callback,
            10
        )
        
        # 新增：订阅IO状态话题
        self.io_status_sub = self.create_subscription(
            String,
            '/io_status',
            self.io_status_callback,
            10
        )
        
        # 新增：订阅业务逻辑完成状态话题
        self.warehouse_completed_sub = self.create_subscription(
            Bool,
            '/warehouse_completed',
            self.warehouse_completed_callback,
            10
        )
        
        self.outbound_completed_sub = self.create_subscription(
            Bool,
            '/outbound_completed',
            self.outbound_completed_callback,
            10
        )
        self.product_arrival_sub = self.create_subscription(
            Bool,
            '/product_arrival',
            self.product_arrival_callback,
            10
        )
        self.fault_sub = self.create_subscription(
            String,
            '/fault_code',
            self.fault_callback,
            10
        )       

        # 创建原有的多个话题发布器
        self.control_pub = self.create_publisher(String, '/py_control_command', 10)  # 改为专用话题
        self.jog_pub = self.create_publisher(String, '/jog_command', 10)
        self.jog_speed_pub = self.create_publisher(String, '/jog_speed_command', 10)
        self.displacement_pub = self.create_publisher(String, '/displacement_command', 10)
        self.layer_pub = self.create_publisher(Int8, '/layer_command', 10)
        self.warehouse_start_pub = self.create_publisher(Int8, '/warehouse_start', 10)
        self.warehouse_stop_pub = self.create_publisher(Empty, '/warehouse_stop', 10)
        self.outbound_start_pub = self.create_publisher(Int8, '/outbound_start', 10)
        self.outbound_stop_pub = self.create_publisher(Empty, '/outbound_stop', 10)
        self.do_control_pub = self.create_publisher(String, '/do_control', 10)
        self.board_width_pub = self.create_publisher(Float64, '/board_width_command', 10)
        self.axis3_width_pub = self.create_publisher(Float64, '/axis3_width_command', 10)
        
        # 新增：创建统一控制状态发布器
        self.integrated_pub = self.create_publisher(ByteMultiArray, '/integrated_control_status', 10)
        
        # 新增：创建开始作业信号发布器（用于通知business_logic_processor启动产品到位周期）
        self.start_operation_signal_pub = self.create_publisher(Bool, '/start_operation_signal', 10)
        
        # 新增：订阅板宽调整状态话题
        self.board_width_status_sub = self.create_subscription(
            String,
            '/board_width_status',
            self.board_width_status_callback,
            10
        )
        self.axis3_width_status_sub = self.create_subscription(
            String,
            '/axis3_width_status',
            self.axis3_width_status_callback,
            10
        )
        
        # 新增：板宽调整状态追踪
        self.axis4_width_adjusting = False
        self.axis3_width_adjusting = False
        self.axis4_width_completed = False
        self.axis3_width_completed = False
        self.pending_start_result = False  # 标记是否有待发布的0x0106响应
        self.start_result_wait_start_time = None  # 等待开始时间
        self.START_RESULT_TIMEOUT = 100.0  # 超时时间100秒
        
        # 新增：创建定时器检查超时
        self.start_result_timer = self.create_timer(0.5, self.check_start_result_timeout)
        
        # 新增：IO状态变量
        self.current_di_bits = 0  # 32位DI状态位掩码
        self.current_do_bits = 0  # 32位DO状态位掩码
        self.last_di_bits = 0     # 上一次DI状态（用于变化检测）
        self.last_do_bits = 0     # 上一次DO状态（用于变化检测）
        self.io_status_received = False  # 标记是否收到过IO状态
        
        # 新增：定时发布线程
        self.publish_timer = self.create_timer(0.1, self.publish_integrated_status)  # 100ms间隔
        
        # 添加IO状态跟踪
        self.last_io_state = 0  # 初始状态为全0
        
        # 新增：当前故障码存储
        self.current_fault_code = 0x0000  # 默认无故障
        self.last_published_fault_code = 0x0000  # 上次发布的故障码（避免重复上报）
        
        # 新增：命令追踪（用于故障时回包）
        self.pending_command = None  # 当前待响应的命令: 0x0101/0x0103/0x0105
        self.pending_response_sent = False  # 标记是否已发送故障响应
        
        self.get_logger().info('ByteMultiArray解析器已启动（支持统一IO状态发布和业务完成状态发布）')

    def ensure_int(self, value):
        """确保值为整数类型"""
        if isinstance(value, int):
            return value
        elif isinstance(value, (bytes, bytearray)):
            return int.from_bytes(value, byteorder='little')
        else:
            try:
                return int(value)
            except (ValueError, TypeError):
                self.get_logger().warn(f'无法转换为整数: {value}')
                return 0

    def board_width_status_callback(self, msg):
        """处理axis4板宽状态回调"""
        try:
            status_str = msg.data
            # 解析状态字符串: "current:XX,target:XX,moving:true/false,status:XXX"
            status_dict = {}
            for item in status_str.split(','):
                if ':' in item:
                    key, value = item.split(':', 1)
                    status_dict[key.strip()] = value.strip()
            
            moving = status_dict.get('moving', 'false') == 'true'
            status = status_dict.get('status', '')
            
            # 更新调整状态
            if moving:
                self.axis4_width_adjusting = True
                self.axis4_width_completed = False
            elif self.axis4_width_adjusting and not moving:
                # 从运动中变为停止，表示调整完成
                self.axis4_width_adjusting = False
                self.axis4_width_completed = True
                self.get_logger().info('Axis4板宽调整完成')
                # 检查是否可以发布0x0106
                self.check_and_publish_start_result()
            elif '已到达目标板宽' in status or '调整完成' in status:
                self.axis4_width_completed = True
                self.axis4_width_adjusting = False
                self.get_logger().info('Axis4板宽已到达目标值（无需调整）')
                # 检查是否可以发布0x0106
                self.check_and_publish_start_result()
                
        except Exception as e:
            self.get_logger().error(f'板宽状态解析错误: {e}')

    def axis3_width_status_callback(self, msg):
        """处理axis3板宽状态回调"""
        try:
            status_str = msg.data
            # 解析状态字符串: "current:XX,target:XX,moving:true/false,status:XXX"
            status_dict = {}
            for item in status_str.split(','):
                if ':' in item:
                    key, value = item.split(':', 1)
                    status_dict[key.strip()] = value.strip()
            
            moving = status_dict.get('moving', 'false') == 'true'
            status = status_dict.get('status', '')
            
            # 更新调整状态
            if moving:
                self.axis3_width_adjusting = True
                self.axis3_width_completed = False
            elif self.axis3_width_adjusting and not moving:
                # 从运动中变为停止，表示调整完成
                self.axis3_width_adjusting = False
                self.axis3_width_completed = True
                self.get_logger().info('Axis3板宽调整完成')
                # 检查是否可以发布0x0106
                self.check_and_publish_start_result()
            elif '已到达目标板宽' in status or '调整完成' in status:
                self.axis3_width_completed = True
                self.axis3_width_adjusting = False
                self.get_logger().info('Axis3板宽已到达目标值（无需调整）')
                # 检查是否可以发布0x0106
                self.check_and_publish_start_result()
                
        except Exception as e:
            self.get_logger().error(f'Axis3板宽状态解析错误: {e}')

    def check_and_publish_start_result(self):
        """检查两个轴是否都完成，如果是则发布0x0106响应"""
        if self.pending_start_result and self.axis4_width_completed and self.axis3_width_completed:
            self.publish_start_result()
            self.pending_start_result = False
            self.start_result_wait_start_time = None
            # 重置完成标志
            self.axis4_width_completed = False
            self.axis3_width_completed = False
            self.get_logger().info('Axis3和Axis4板宽调整均完成，发布0x0106响应')

    def check_start_result_timeout(self):
        """检查0x0106响应是否超时"""
        if not self.pending_start_result:
            return
        
        if self.start_result_wait_start_time is None:
            self.start_result_wait_start_time = time.time()
            return
        
        elapsed = time.time() - self.start_result_wait_start_time
        if elapsed > self.START_RESULT_TIMEOUT:
            # 超时，强制发布0x0106，并附带板宽调整超时故障码
            fault_code = FaultCode.BOARD_WIDTH_TIMEOUT.value
            self.get_logger().warn(f'板宽调整等待超时({self.START_RESULT_TIMEOUT}秒)，强制发布0x0106响应，故障码=0x{fault_code:04X}')
            self.publish_start_result(extra_fault_code=fault_code)
            self.pending_start_result = False
            self.start_result_wait_start_time = None
            # 重置完成标志
            self.axis4_width_completed = False
            self.axis3_width_completed = False

    def fault_callback(self, msg):
        """处理故障码话题回调，存储当前故障码状态"""
        try:
            fault_data_str = msg.data

            # 解析故障码
            # 格式可能为："0"（无故障）或 "axis1_1:0x1234,axis4:0x5678"（多个故障）
            fault_code_combined = 0x0000  # 默认无故障

            if fault_data_str != "0" and fault_data_str:
                # 尝试解析多个轴的故障码。这里采用一种策略：取第一个非零错误码，或进行位组合。
                # 示例：简单取第一个遇到的错误码（根据实际需求调整逻辑）。
                import re
                # 匹配模式：轴名:0xXXXX
                pattern = r'0x([0-9A-Fa-f]+)'
                matches = re.findall(pattern, fault_data_str)
                if matches:
                    # 取第一个错误码，并确保其在0-65535（2字节）范围内
                    try:
                        first_code = int(matches[0], 16) & 0xFFFF
                        fault_code_combined = first_code
                    except ValueError:
                        self.get_logger().warn(f'无法解析故障码: {matches[0]}')
                        fault_code_combined = 0xFFFF  # 未知错误
                        
            # === 边沿检测：故障码变化时才处理 ===
            if fault_code_combined != self.current_fault_code:
                self.current_fault_code = fault_code_combined
                
                # 故障变化时记录日志
                if fault_code_combined != 0:
                    self.get_logger().warn(f'当前系统故障码: 0x{fault_code_combined:04X}')
                    
                    # 上报119前，先回包待处理的命令（102/104/106），带故障码
                    self._send_pending_response_with_fault(fault_code_combined)
                    
                    self.publish_fault_status(fault_code_combined)
                else:
                    # 故障已清除，重置上次发布的故障码记录
                    if self.last_published_fault_code != 0x0000:
                        self.get_logger().info('系统故障已清除，重置故障码发布记录')
                        self.last_published_fault_code = 0x0000
            else:
                # 故障码未变化，静默处理（避免海量打印）
                pass

        except Exception as e:
            self.get_logger().error(f'故障码处理错误: {e}')
    
    def _send_pending_response_with_fault(self, fault_code):
        """故障时回包待处理的命令响应（带故障码）"""
        try:
            # 情况1: 有待处理的入库命令(101)未回102
            if self.pending_command == 0x0101 and not self.pending_response_sent:
                self.get_logger().warn(f'故障时回包102(入库响应)，故障码=0x{fault_code:04X}')
                self.publish_command_response(0x0102, fault_code)
                self.pending_response_sent = True
            
            # 情况2: 有待处理的出库命令(103)未回104  
            elif self.pending_command == 0x0103 and not self.pending_response_sent:
                self.get_logger().warn(f'故障时回包104(出库响应)，故障码=0x{fault_code:04X}')
                self.publish_command_response(0x0104, fault_code)
                self.pending_response_sent = True
            
            # 情况3: 有待处理的开始作业命令(105)未回106
            elif self.pending_start_result:
                # 105的特殊处理：正在等待板宽调整完成，故障时立即回106带故障码
                self.get_logger().warn(f'故障时回包106(开始作业响应)，故障码=0x{fault_code:04X}')
                self.publish_command_response(0x0106, fault_code)
                self.pending_start_result = False
                self.start_result_wait_start_time = None
                # 重置板宽调整状态
                self.axis4_width_adjusting = False
                self.axis3_width_adjusting = False
                self.axis4_width_completed = False
                self.axis3_width_completed = False
                
        except Exception as e:
            self.get_logger().error(f'故障时回包失败: {e}')

    def publish_fault_status(self, fault_code):
        """发布故障状态归一化消息 (命令码0x0119)，只在有故障时上报，相同故障码只返回一次"""
        try:
            # 无故障时不上报
            if fault_code == 0x0000:
                return
            
            # 检查是否与上次发布的故障码相同，相同则跳过（避免重复上报）
            if fault_code == self.last_published_fault_code:
                self.get_logger().debug(f'故障码0x{fault_code:04X}已发布过，跳过重复上报')
                return
            
            # 记录本次发布的故障码
            self.last_published_fault_code = fault_code
            
            # 构建4字节消息 (小端序)
            # 格式: [命令码低8位, 命令码高8位, 故障码低8位, 故障码高8位]
            message_data = []

            # 命令码: 0x0119 (小端序: 0x19, 0x01)
            message_data.append(bytes([0x19]))  # 低字节
            message_data.append(bytes([0x01]))  # 高字节

            # 故障码: 16位小端序
            message_data.append(bytes([fault_code & 0xFF]))        # 低字节
            message_data.append(bytes([(fault_code >> 8) & 0xFF])) # 高字节

            # 创建并发布 ByteMultiArray 消息
            # 注意：这里发布到统一的状态话题，与 publish_integrated_status 一致。
            layout = MultiArrayLayout()
            layout.data_offset = 0
            layout.dim = [MultiArrayDimension()]
            layout.dim[0].label = 'fault_status'
            layout.dim[0].size = len(message_data)
            layout.dim[0].stride = 1

            msg = ByteMultiArray()
            msg.layout = layout
            msg.data = message_data
            self.integrated_pub.publish(msg)  # 发布到统一状态话题
            
            self.get_logger().warn(f'发布故障状态0x0119: 故障码=0x{fault_code:04X}')

        except Exception as e:
            self.get_logger().error(f'发布故障状态消息失败: {e}')

    def io_status_callback(self, msg):
        """处理IO状态话题回调"""
        try:
            # 解析IO状态字符串
            self.parse_io_status_string(msg.data)
            self.io_status_received = True
        except Exception as e:
            self.get_logger().error(f'IO状态解析错误: {e}')

    def warehouse_completed_callback(self, msg):
        """处理入库完成状态回调"""
        try:
            current_state = msg.data
            # 检测状态变化（从False变为True）
            if current_state:
                # 正常完成，回102带故障码0
                self.publish_command_response(0x0102, 0x0000)
                self.get_logger().info('检测到入库流程完成，发布102响应(正常)')
                # 重置待处理命令状态
                if self.pending_command == 0x0101:
                    self.pending_command = None
                    self.pending_response_sent = False
            
        except Exception as e:
            self.get_logger().error(f'入库完成状态处理错误: {e}')

    def outbound_completed_callback(self, msg):
        """处理出库完成状态回调"""
        try:
            current_state = msg.data
            # 检测状态变化（从False变为True）
            if current_state:
                # 正常完成，回104带故障码0
                self.publish_command_response(0x0104, 0x0000)
                self.get_logger().info('检测到出库流程完成，发布104响应(正常)')
                # 重置待处理命令状态
                if self.pending_command == 0x0103:
                    self.pending_command = None
                    self.pending_response_sent = False
        except Exception as e:
            self.get_logger().error(f'出库完成状态处理错误: {e}')
            
    # 添加新的回调函数
    def product_arrival_callback(self, msg):
        """处理产品到位回调 - 只处理到位，忽略离开"""
        if not msg.data:
            return  # 忽略False（产品离开）
        
        try:
            self.publish_product_arrival()
            self.get_logger().info('收到产品到位消息，发布0x0109命令')
        except Exception as e:
            self.get_logger().error(f'产品到位处理错误: {e}')

    def publish_product_arrival(self):
        """发布产品到位归一化消息 (0x0109) - 4字节小端序"""
        try:
            # 构建4字节消息：命令码0x0109 + 2字节数据0x0000（小端序）
            # ByteMultiArray.data 需要是 bytes 类型的列表
            message_data = [
                bytes([0x09]),  # 命令码低字节
                bytes([0x01]),  # 命令码高字节
                bytes([0x00]),  # 数据低字节
                bytes([0x00])   # 数据高字节
            ]
            
            # 创建并发布消息
            msg = ByteMultiArray()
            msg.data = message_data
            self.integrated_pub.publish(msg)
            
            self.get_logger().debug('发布产品到位归一化消息: 0x09 0x01 0x00 0x00')
            
        except Exception as e:
            self.get_logger().error(f'发布产品到位消息失败: {e}')

    def publish_warehouse_completion(self, result_code):
        """发布入库完成消息 (命令码0x0102)，根据/fault_code反馈决定异常码"""
        try:
            # 构建4字节消息 (小端序)
            # 格式: [命令码低8位, 命令码高8位, 异常码低8位, 异常码高8位]
            message_data = []
            
            # 命令码: 0x0102 (小端序: 0x02, 0x01)
            message_data.append(bytes([0x02]))  # 低字节
            message_data.append(bytes([0x01]))  # 高字节
            
            # 异常码: 有故障用故障码，无故障用0x0000
            error_code = self.current_fault_code if self.current_fault_code != 0 else 0x0000
            message_data.append(bytes([error_code & 0xFF]))   # 低字节
            message_data.append(bytes([(error_code >> 8) & 0xFF]))  # 高字节
            
            # 创建MultiArrayLayout
            layout = MultiArrayLayout()
            layout.data_offset = 0
            layout.dim = [MultiArrayDimension()]
            layout.dim[0].label = 'warehouse_completion'
            layout.dim[0].size = len(message_data)
            layout.dim[0].stride = 1
            
            # 创建并发布消息
            msg = ByteMultiArray()
            msg.layout = layout
            msg.data = message_data
            
            self.integrated_pub.publish(msg)
            
            self.get_logger().info(f'发布入库完成消息: 命令=0x0102, 异常码=0x{error_code:04X}')
            
        except Exception as e:
            self.get_logger().error(f'发布入库完成消息失败: {e}')

    def publish_outbound_completion(self, result_code):
        """发布出库完成消息 (命令码0x0104)，根据/fault_code反馈决定异常码"""
        try:
            # 构建4字节消息 (小端序)
            # 格式: [命令码低8位, 命令码高8位, 异常码低8位, 异常码高8位]
            message_data = []
            
            # 命令码: 0x0104 (小端序: 0x04, 0x01)
            message_data.append(bytes([0x04]))  # 低字节
            message_data.append(bytes([0x01]))  # 高字节
            
            # 异常码: 有故障用故障码，无故障用0x0000
            error_code = self.current_fault_code if self.current_fault_code != 0 else 0x0000
            message_data.append(bytes([error_code & 0xFF]))   # 低字节
            message_data.append(bytes([(error_code >> 8) & 0xFF]))  # 高字节
            
            # 创建MultiArrayLayout
            layout = MultiArrayLayout()
            layout.data_offset = 0
            layout.dim = [MultiArrayDimension()]
            layout.dim[0].label = 'outbound_completion'
            layout.dim[0].size = len(message_data)
            layout.dim[0].stride = 1
            
            # 创建并发布消息
            msg = ByteMultiArray()
            msg.layout = layout
            msg.data = message_data
            
            self.integrated_pub.publish(msg)
            
            self.get_logger().info(f'发布出库完成消息: 命令=0x0104, 异常码=0x{error_code:04X}')
            
        except Exception as e:
            self.get_logger().error(f'发布出库完成消息失败: {e}')

    def parse_io_status_string(self, status_str):
        """解析IO状态字符串，提取DI和DO值"""
        # 初始化位掩码
        di_bits = 0
        do_bits = 0
        
        # 分割DI和DO部分
        parts = status_str.split('|')
        if len(parts) < 2:
            self.get_logger().warn('IO状态格式错误，缺少DO部分')
            return
        
        di_part = parts[0].strip()  # "DI状态: DI00:0,DI01:0,..."
        do_part = parts[1].strip()  # "DO状态: DO00:0,DO01:0,..."
        
        # 解析DI状态
        if di_part.startswith('DI状态:'):
            di_data = di_part[5:].strip()  # 去除"DI状态:"
            di_items = di_data.split(',')
            
            for item in di_items:
                item = item.strip()
                if ':' in item:
                    key, value = item.split(':', 1)
                    key = key.strip()
                    value = value.strip()
                    
                    if key.startswith('DI'):
                        try:
                            # 提取DI编号
                            di_num = int(key[2:])
                            # 设置位掩码：1表示True/激活，0表示False/未激活
                            if value == '1':
                                di_bits |= (1 << di_num)
                        except ValueError:
                            self.get_logger().warn(f'无效的DI编号: {key}')
        
        # 解析DO状态
        if do_part.startswith('DO状态:'):
            do_data = do_part[5:].strip()  # 去除"DO状态:"
            do_items = do_data.split(',')
            
            for item in do_items:
                item = item.strip()
                if ':' in item:
                    key, value = item.split(':', 1)
                    key = key.strip()
                    value = value.strip()
                    
                    if key.startswith('DO'):
                        try:
                            # 提取DO编号
                            do_num = int(key[2:])
                            # 设置位掩码：1表示True/激活，0表示False/未激活
                            if value == '1':
                                do_bits |= (1 << do_num)
                        except ValueError:
                            self.get_logger().warn(f'无效的DO编号: {key}')
        
        # 更新当前状态
        self.current_di_bits = di_bits
        self.current_do_bits = do_bits
        
        # 调试信息
        self.get_logger().debug(f'解析IO状态: DI=0x{di_bits:08X}, DO=0x{do_bits:08X}')

    def publish_integrated_status(self):
        """每100ms发布统一的IO状态信息"""
    
        if not self.io_status_received:
            return  # 尚未收到IO状态数据
            
        # 检查状态是否变化
        di_changed = (self.current_di_bits != self.last_di_bits)
        do_changed = (self.current_do_bits != self.last_do_bits)
        
        # 发布DI状态更新 (0x0111)
        if di_changed:
            self.publish_di_status_update()
            self.last_di_bits = self.current_di_bits

        # 发布DO状态更新 (0x0113)
        if do_changed:
            self.publish_do_status_update()
            self.last_do_bits = self.current_do_bits

    def publish_di_status_update(self):
        """发布DI状态更新消息 (命令码0x0111) - 修改为附件二格式"""
        try:
            # 构建8字节消息 (小端序)
            # 格式: [命令码低8位, 命令码高8位, 组号低8位, 组号高8位, IO状态低8位, IO状态次低8位, IO状态次高8位, IO状态高8位]
            message_data = []
            
            # 命令码: 0x0111 (小端序: 0x11, 0x01)
            message_data.append(bytes([0x11]))  # 低字节
            message_data.append(bytes([0x01]))  # 高字节
            
            # 组号: 输入IO为0x0001 (小端序: 0x01, 0x00)
            message_data.append(bytes([0x01]))  # 低字节
            message_data.append(bytes([0x00]))  # 高字节
            
            # DI状态: 32位小端序
            di_state = self.current_di_bits
            message_data.append(bytes([(di_state >> 0) & 0xFF]))   # 最低字节
            message_data.append(bytes([(di_state >> 8) & 0xFF]))   # 次低字节
            message_data.append(bytes([(di_state >> 16) & 0xFF]))  
            message_data.append(bytes([(di_state >> 24) & 0xFF]))  
            message_data.append(bytes([(di_state >> 32) & 0xFF]))  # 次高字节
            message_data.append(bytes([(di_state >> 48) & 0xFF]))  # 最高字节
            
            # 创建MultiArrayLayout
            layout = MultiArrayLayout()
            layout.data_offset = 0
            layout.dim = [MultiArrayDimension()]
            layout.dim[0].label = 'di_status_update'
            layout.dim[0].size = len(message_data)
            layout.dim[0].stride = 1
            
            # 创建并发布消息
            msg = ByteMultiArray()
            msg.layout = layout
            msg.data = message_data
            
            self.integrated_pub.publish(msg)
            
            self.get_logger().debug(f'发布DI状态更新: 命令=0x0111, 组号=0x0001, 状态=0x{di_state:08X}')
            
        except Exception as e:
            self.get_logger().error(f'发布DI状态更新失败: {e}')

    def publish_do_status_update(self):
        """发布DO状态更新消息 (命令码0x0113) - 修改为附件二格式"""
        try:
            # 构建8字节消息 (小端序)
            message_data = []

            # 命令码: 0x0113 (小端序: 0x13, 0x01)
            message_data.append(bytes([0x13]))  # 低字节
            message_data.append(bytes([0x01]))  # 高字节
            
            # 组号: 输出IO为0x0000 (小端序: 0x00, 0x00)
            message_data.append(bytes([0x00]))  # 低字节
            message_data.append(bytes([0x00]))  # 高字节
            
            # DO状态: 32位小端序
            do_state = self.current_do_bits
            message_data.append(bytes([(do_state >> 0) & 0xFF]))   # 最低字节
            message_data.append(bytes([(do_state >> 8) & 0xFF]))   # 次低字节
            message_data.append(bytes([(do_state >> 16) & 0xFF]))  
            message_data.append(bytes([(do_state >> 24) & 0xFF]))  
            message_data.append(bytes([(do_state >> 32) & 0xFF]))  # 次高字节
            message_data.append(bytes([(do_state >> 48) & 0xFF]))  # 最高字节
            
            # 创建MultiArrayLayout
            layout = MultiArrayLayout()
            layout.data_offset = 0
            layout.dim = [MultiArrayDimension()]
            layout.dim[0].label = 'do_status_update'
            layout.dim[0].size = len(message_data)
            layout.dim[0].stride = 1
            
            # 创建并发布消息
            msg = ByteMultiArray()
            msg.layout = layout
            msg.data = message_data
            
            self.integrated_pub.publish(msg)

            self.get_logger().debug(f'发布DO状态更新: 命令=0x0113, 组号=0x0000, 状态=0x{do_state:08X}')
        except Exception as e:
            self.get_logger().error(f'发布DO状态更新失败: {e}')

    def unified_callback(self, msg):
        """处理统一的ByteMultiArray消息"""
        if len(msg.data) == 0:
            self.get_logger().warn('收到空数据消息')
            return
            
        try:
            # 关键修复：确保所有数据都是整数类型
            self.get_logger().info(f'=== 收到消息 ===')
            self.get_logger().info(f'数据长度: {len(msg.data)} 字节')
            data_list = []
            for item in msg.data:
                data_list.append(self.ensure_int(item))
            
            # 根据图片，指令码可能是16位（2字节）
            # 假设是小端序，先尝试解析为16位整数
            if len(data_list) >= 2:
                command_code = (data_list[1] << 8) | data_list[0]  # 小端序
                payload = data_list[2:]  # 去除命令码后的负载数据
            else:
                # 如果数据长度不足，尝试单字节命令码
                command_code = data_list[0]
                payload = data_list[1:]
            
            self.get_logger().info(f'解析命令: 0x{command_code:04X}, 负载长度: {len(payload)}')
            
            # 根据命令码分发处理
            if command_code == CommandType.NOTIFY_STORAGE.value:  # 通知存放
                self.process_notify_storage(payload)
            elif command_code == CommandType.NOTIFY_RETRIEVAL.value:  # 通知取出
                self.process_notify_retrieval(payload)
            elif command_code == CommandType.START_OPERATION.value:  # 开始作业
                self.process_start_operation(payload)
            elif command_code == CommandType.END_OPERATION.value:  # 结束作业
                self.process_end_operation(payload)
            elif command_code == CommandType.AXIS_JOG.value:  # 轴点动
                self.process_axis_jog(payload)
            elif command_code == CommandType.AXIS_STOP.value:  # 轴停止
                self.process_axis_stop(payload)
            elif command_code == CommandType.WRITE_IO.value:  # 写IO
                self.process_write_io(payload)
            elif command_code == CommandType.CLEAR_SYSTEM_FAULT.value:  # 清除系统故障和告警
                self.process_clear_system_fault(payload)
            elif command_code == CommandType.CLEAR_AXIS_FAULT.value:  # 清除轴故障
                self.process_clear_axis_fault(payload)
            # 原有的指令码处理
            elif command_code == CommandType.SYSTEM_CONTROL.value:
                self.process_system_control(payload)
            elif command_code == CommandType.JOG_SPEED.value:
                self.process_jog_speed(payload)
            elif command_code == CommandType.POSITION_CONTROL.value:
                self.process_position_control(payload)
            elif command_code == CommandType.LAYER_COMMAND.value:
                self.process_layer_command(payload)
            elif command_code == CommandType.BOARD_WIDTH.value:
                self.process_board_width(payload)
            else:
                self.get_logger().warn(f'未知命令码: 0x{command_code:04X}')
                
        except Exception as e:
            self.get_logger().error(f'消息解析错误: {e}')

    def process_start_operation(self, payload):
        """处理开始作业命令 (0x0105) - /control_command -> start_auto，并解析宽度信息"""
        # 0. 首先发布开始作业信号（启动产品到位发布周期）
        start_signal = Bool()
        start_signal.data = True
        self.start_operation_signal_pub.publish(start_signal)
        self.get_logger().info('发布开始作业信号(0x0105)，启动产品到位发布周期')
        
        # 1. 发布开始自动模式命令
        command_str = "start_auto"
        msg = String()
        msg.data = command_str
        self.control_pub.publish(msg)
        self.get_logger().info('发布开始作业命令: 进入自动模式')

        # 2. 解析并发布宽度信息（从第3和第4字节提取，放大10倍的小端序整数）
        if len(payload) >= 2:
            # 提取第3和第4字节（索引2和3）作为宽度数据
            # 消息格式：[0x05, 0x01, width_low, width_high, ...]
            width_low = payload[0]  # 第3字节（低位）
            width_high = payload[1]  # 第4字节（高位）
            
            # 小端序转换为整数（16位）
            width_integer = (width_high << 8) | width_low
            
            # 除以10.0得到实际宽度值（单位：厘米）
            actual_width_cm = width_integer * 0.1

            self.get_logger().info(f'解析到宽度信息: 原始值={width_integer}, 实际值={actual_width_cm}cm')

            # ========== 关键修复：先设置等待状态，再下发命令 ==========
            # 避免竞态条件：ethercat_node处理太快，状态在设置标志前就发布了
            
            # 1. 先重置板宽完成状态并标记等待（必须在publish之前！）
            self.axis4_width_completed = False
            self.axis3_width_completed = False
            self.axis4_width_adjusting = False
            self.axis3_width_adjusting = False
            self.pending_start_result = True
            self.start_result_wait_start_time = time.time()
            self.get_logger().info('准备下发板宽命令，已设置等待标志（竞态条件修复）')

            # 2. 再下发 axis4 板宽控制话题
            width_msg_axis4 = Float64()
            width_msg_axis4.data = actual_width_cm
            self.board_width_pub.publish(width_msg_axis4)
            self.get_logger().info(f'已下发axis4板宽命令: {actual_width_cm}cm')

            # 3. 再下发 axis3 板宽控制话题
            width_msg_axis3 = Float64()
            width_msg_axis3.data = actual_width_cm
            self.axis3_width_pub.publish(width_msg_axis3)
            self.get_logger().info(f'已下发axis3板宽命令: {actual_width_cm}cm')
            
            self.get_logger().info('等待axis3和axis4调整完成后发布0x0106响应')
        else:
            self.get_logger().warn('开始作业命令负载长度不足，需要至少4字节，仅启动自动模式。')
            # 没有板宽调整，直接发布0x0106
            self.publish_start_result()

    def publish_start_result(self, extra_fault_code=None):
        """发布开始结果响应 (命令码0x0106)，根据/fault_code反馈决定异常码
        
        Args:
            extra_fault_code: 额外的故障码（如板宽调整超时），优先级高于系统故障码
        """
        try:
            # 异常码优先级：extra_fault_code > current_fault_code > 0x0000
            if extra_fault_code is not None and extra_fault_code != 0:
                error_code = extra_fault_code
            elif self.current_fault_code != 0:
                error_code = self.current_fault_code
            else:
                error_code = 0x0000
            
            # 使用通用响应函数发布106
            self.publish_command_response(0x0106, error_code)
            
        except Exception as e:
            self.get_logger().error(f'发布开始结果响应失败: {e}')

    def publish_command_response(self, command_code, error_code):
        """发布通用命令响应 (102/104/106等)，带指定故障码
        
        Args:
            command_code: 响应命令码 (如 0x0102, 0x0104, 0x0106)
            error_code: 故障码 (0x0000 表示正常，非0表示故障)
        """
        try:
            # 构建4字节响应消息 (小端序)
            # 格式: [命令码低8位, 命令码高8位, 异常码低8位, 异常码高8位]
            message_data = []
            
            # 命令码 (小端序)
            message_data.append(bytes([command_code & 0xFF]))      # 低字节
            message_data.append(bytes([(command_code >> 8) & 0xFF]))  # 高字节
            
            # 异常码 (小端序)
            message_data.append(bytes([error_code & 0xFF]))        # 低字节
            message_data.append(bytes([(error_code >> 8) & 0xFF])) # 高字节
            
            # 创建MultiArrayLayout
            layout = MultiArrayLayout()
            layout.data_offset = 0
            layout.dim = [MultiArrayDimension()]
            layout.dim[0].label = 'command_response'
            layout.dim[0].size = len(message_data)
            layout.dim[0].stride = 1
            
            # 创建并发布消息
            msg = ByteMultiArray()
            msg.layout = layout
            msg.data = message_data
            
            self.integrated_pub.publish(msg)
            
            cmd_name = {0x0102: '入库完成(102)', 0x0104: '出库完成(104)', 0x0106: '开始作业响应(106)'}.get(command_code, f'未知(0x{command_code:04X})')
            self.get_logger().info(f'发布命令响应: {cmd_name}, 异常码=0x{error_code:04X}')
            
        except Exception as e:
            self.get_logger().error(f'发布命令响应失败: {e}')

    def process_write_io(self, payload):
        """处理写IO命令 (0x0115) - /do_control，支持状态翻转检测"""
        # IO地址映射表（0-15位）
        io_mapping = {
            0: 800,   # 启动按钮灯
            1: 801,   # 复位按钮灯
            2: 802,   # 暂停按钮灯
            3: 803,   # 蜂鸣器
            4: 804,   # 三色红灯
            5: 805,   # 三色黄灯
            6: 806,   # 三色绿灯
            7: 807,   # 预留
            8: 808,   # 预留
            9: 809,   # 预留
            10: 810,  # 顶升气缸下降
            11: 811,  # 齿轮对接气缸伸出
            12: 812,  # 皮带正转启动
            13: 813,  # 皮带反转启动
            14: 814,  # 预留
            15: 815   # 预留
        }
        
        # 检查负载长度：现在需要至少4字节（2字节组号 + 2字节IO状态）
        if len(payload) < 4:
            self.get_logger().warn('写IO命令负载长度不足，需要至少4字节')
            return
            
        # 解析组号（2字节，小端序）
        group_low = payload[0]  # 组号低位
        group_high = payload[1]  # 组号高位
        group_id = (group_high << 8) | group_low
        
        # 解析IO状态（2字节，小端序）
        io_low = payload[2]  # IO状态低位
        io_high = payload[3]  # IO状态高位
        current_io_state = (io_high << 8) | io_low
        
        self.get_logger().info(f'写IO命令解析: 组号=0x{group_id:04X}, IO状态=0x{current_io_state:04X}, 上次状态=0x{self.last_io_state:04X}')
        
        # 如果组号不是0x0000，记录警告但继续处理（根据您的需求，可以忽略组号）
        if group_id != 0x0000:
            self.get_logger().warn(f'非默认组号: 0x{group_id:04X}，将忽略组号继续处理')
        # 遍历所有位（0-15位）
        for bit_position in range(16):
            if bit_position in io_mapping:
                do_number = io_mapping[bit_position]
                
                # 获取当前位和上一次的位状态
                current_bit = (current_io_state >> bit_position) & 0x01
                last_bit = (self.last_io_state >> bit_position) & 0x01
                
                # 检查是否发生翻转
                if current_bit != last_bit:
                    # 状态发生变化，发布命令
                    command_str = f"{do_number}:{current_bit}"
                    msg = String()
                    msg.data = command_str
                    self.do_control_pub.publish(msg)
                    
                    if current_bit == 1:
                        self.get_logger().info(f'检测到翻转: {do_number} 从0变为1')
                    else:
                        self.get_logger().info(f'检测到翻转: {do_number} 从1变为0')
                    
                    self.get_logger().info(f'发布DO控制命令: {command_str} (位{bit_position})')
                else:
                    # 状态未变化，不发布命令
                    self.get_logger().debug(f'状态未变化: {do_number} 保持{current_bit}')
        
        # 更新上一次状态
        self.last_io_state = current_io_state

    def process_notify_storage(self, payload):
        """处理通知存放命令 (0x0101) - /warehouse_start"""
        # 检查负载长度：至少需要2字节（层高）
        if len(payload) < 2:
            self.get_logger().warn('通知存放命令负载长度不足')
            return
        
        # 解析层高（小端序）：最后2个字节
        # 如果payload长度大于2，取最后2个字节；否则取全部
        if len(payload) >= 2:
            # 取最后2个字节作为层高
            layer_low = payload[-2]  # 层高低位字节
            layer_high = payload[-1]  # 层高高位字节
            original_layer = (layer_high << 8) | layer_low  # 小端序组合
        else:
            # 如果只有2个字节，直接使用
            layer_low = payload[0]
            layer_high = payload[1]
            original_layer = (layer_high << 8) | layer_low
        
        # 层号映射：1-41 映射到 -15 到 28
        # 映射规律：
        # 1-15 -> -15 到 -1
        # 16-41 -> 3 到 28
        if 1 <= original_layer <= 15:
            mapped_layer = original_layer - 16
        elif 16 <= original_layer <= 41:
            mapped_layer = original_layer - 13
        else:
            # 如果层号超出范围，记录警告并尝试默认映射
            self.get_logger().warn(f'原始层号超出映射范围: {original_layer}，尝试使用默认映射')
            if original_layer <= 15:
                mapped_layer = original_layer - 16
            else:
                mapped_layer = original_layer - 13
        
        # 发布到/warehouse_start话题
        msg = Int8()
        msg.data = mapped_layer
        self.warehouse_start_pub.publish(msg)   
            
        # 记录详细信息
        self.get_logger().info(f'发布仓库启动命令: 原始层{original_layer} -> 映射层{mapped_layer} (字节序列: 0x{layer_low:02X} 0x{layer_high:02X})')
        
        # 如果有额外的数据（组号和IO状态），记录但不处理
        if len(payload) > 2:
            extra_data = payload[:-2]  # 除了最后2个字节外的所有数据
            self.get_logger().info(f'忽略额外数据: {extra_data}')
        
        # 追踪命令：标记有待响应的101命令
        self.pending_command = 0x0101
        self.pending_response_sent = False
        self.get_logger().debug('标记待响应命令: 0x0101（入库）')
            
    def process_notify_retrieval(self, payload):
        """处理通知取出命令 (0x0103) - /outbound_start，包含层号映射（1-41 映射到 -15 到 28）"""
        # 检查负载长度：至少需要2字节（层高）
        if len(payload) < 4:
            self.get_logger().warn('通知取出命令负载长度不足，需要至少4字节')
            return
        
        # 完整格式：[组号低位, 组号高位, IO状态低位, IO状态高位, 层高低位, 层高高位]
        # 我们只关心最后2个字节（层高）
        if len(payload) >= 4:
            # 取最后2个字节作为层高
            layer_low = payload[2]  # 第3个字节是层高低位
            layer_high = payload[3]  # 第4个字节是层高高位
        else:
            # 如果只有2个字节，直接使用（简化格式）
            layer_low = payload[0]
            layer_high = payload[1]
        
        original_layer = (layer_high << 8) | layer_low  # 小端序组合
        
        # 层号映射：1-41 映射到 -15 到 28
        # 映射规律：
        # 1-15 -> -15 到 -1
        # 16-41 -> 3 到 28
        if 1 <= original_layer <= 15:
            mapped_layer = original_layer - 16
        elif 16 <= original_layer <= 41:
            mapped_layer = original_layer - 13
        else:
            # 如果层号超出范围，记录警告并尝试默认映射
            self.get_logger().warn(f'原始层号超出映射范围: {original_layer}，尝试使用默认映射')
            if original_layer <= 15:
                mapped_layer = original_layer - 16
            else:
                mapped_layer = original_layer - 13
        
        # 发布到/outbound_start话题
        msg = Int8()
        msg.data = mapped_layer
        self.outbound_start_pub.publish(msg)
        
        # 记录详细信息
        self.get_logger().info(f'发布出库启动命令: 原始层{original_layer} -> 映射层{mapped_layer} (字节序列: 0x{layer_low:02X} 0x{layer_high:02X})')
        
        # 如果有额外的数据（组号和IO状态），记录但不处理
        if len(payload) > 2:
            extra_data = payload[:-2]  # 除了最后2个字节外的所有数据
            self.get_logger().info(f'忽略额外数据: {extra_data}')

    def process_end_operation(self, payload):
        """处理结束作业命令 (0x0107) - /warehouse_stop /outbound_stop"""
        # 根据表格：出入库同时停止（不管数据）
        msg = Empty()
        self.warehouse_stop_pub.publish(msg)
        self.outbound_stop_pub.publish(msg)
        self.get_logger().info('发布仓库/出库停止命令')
        # 结束作业命令，直接发布 /control_command -> stop命令
        command_str = "stop"
        control_msg = String()
        control_msg.data = command_str
        self.control_pub.publish(control_msg)
        self.get_logger().info('发布开始作业命令: 进入自动模式')

    def process_axis_jog(self, payload):
        """处理轴点动命令 (0x010D) - /jog_command"""
        # 检查负载长度：至少需要6字节（轴号2字节 + 方向2字节 + 其他数据）
        if len(payload) < 6:
            self.get_logger().warn('轴点动命令负载长度不足，需要至少6字节')
            return
        
        # 解析轴号（小端序）：负载的第1-2字节（索引0-1）
        axis_low = payload[0]  # 轴号低位字节
        axis_high = payload[1]  # 轴号高位字节
        axis_num = (axis_high << 8) | axis_low  # 小端序组合
        
        # 解析方向（小端序）：负载的第3-4字节（索引2-3）
        direction_low = payload[2]  # 方向低位字节
        direction_high = payload[3]  # 方向高位字节
        direction = (direction_high << 8) | direction_low  # 小端序组合
        
        # 解析速度（小端序）：负载的第5-6字节（索引4-5）
        speed_low = payload[4]  # 速度低位字节
        speed_high = payload[5]  # 速度高位字节
        speed = (speed_high << 8) | speed_low  # 小端序组合，单位：mm/s * 10

        # 轴号映射表：数字轴号 -> 字符串轴名
        axis_mapping = {
            1: "axis1_1",  # 轴1的第一个电机
            2: "axis1_2",  # 轴1的第二个电机
            3: "axis2_1",  # 轴2的第一个电机
            4: "axis2_2",  # 轴2的第二个电机
            5: "axis3",    # 轴3
            6: "axis4",    # 轴4
            7: "axis5"     # 轴5
        }
        
        # 将数字轴号转换为字符串轴名
        axis_name = axis_mapping.get(axis_num, f"axis{axis_num}")
        # axis_name = f"axis{axis_num}"
        
        # 方向映射
        direction_map = {
            0: "stop",
            1: "forward",  # 假设1为正转
            2: "reverse"   # 假设2为反转
        }
        
        direction_str = direction_map.get(direction, "stop")
        command_str = f"{axis_name}:{direction_str}"
        
        # 发布到/jog_command话题
        msg = String()
        msg.data = command_str
        self.jog_pub.publish(msg)
        
        # 发布到/jog_speed_command话题
        speed_command_str = f"{axis_name}:{speed}"
        speed_msg = String()
        speed_msg.data = speed_command_str
        self.jog_speed_pub.publish(speed_msg)

        # 记录详细信息
        self.get_logger().info(f'发布点动命令: {command_str}, 速度: {speed}mm/s (轴号: {axis_num}->{axis_name}, 方向: 0x{direction_high:02X}{direction_low:02X}, 速度: 0x{speed_high:02X}{speed_low:02X})')

        # 如果有额外的数据，记录但不处理
        if len(payload) > 6:
            extra_data = payload[6:]  # 第7字节及以后的数据
            self.get_logger().info(f'忽略额外数据: {extra_data}')

    def process_axis_stop(self, payload):
        """处理轴停止命令 (0x010F) - /jog_command"""
        # 检查负载长度：至少需要6字节（轴号2字节 + 方向2字节 + 其他数据）
        if len(payload) < 6:
            self.get_logger().warn('轴点动命令负载长度不足，需要至少6字节')
            return
        
        # 解析轴号（小端序）：负载的第1-2字节（索引0-1）
        axis_low = payload[0]  # 轴号低位字节
        axis_high = payload[1]  # 轴号高位字节
        axis_num = (axis_high << 8) | axis_low  # 小端序组合
        
        # 解析方向（小端序）：负载的第3-4字节（索引2-3）
        direction_low = payload[2]  # 方向低位字节
        direction_high = payload[3]  # 方向高位字节
        direction = (direction_high << 8) | direction_low  # 小端序组合
        
        # 轴号映射表：数字轴号 -> 字符串轴名
        axis_mapping = {
            1: "axis1_1",  # 轴1的第一个电机
            2: "axis1_2",  # 轴1的第二个电机
            3: "axis2_1",  # 轴2的第一个电机
            4: "axis2_2",  # 轴2的第二个电机
            5: "axis3",    # 轴3
            6: "axis4",    # 轴4
            7: "axis5"     # 轴5
        }
        
        # 将数字轴号转换为字符串轴名
        axis_name = axis_mapping.get(axis_num, f"axis{axis_num}")
        # axis_name = f"axis{axis_num}"
        
        # 方向映射
        direction_map = {
            0: "stop",
        }
        
        direction_str = direction_map.get(direction, "stop")
        command_str = f"{axis_name}:{direction_str}"
        
        # 发布到/jog_command话题
        msg = String()
        msg.data = command_str
        self.jog_pub.publish(msg)
        
        # 记录详细信息
        self.get_logger().info(f'发布点动命令: {command_str} (轴号: {axis_num}->{axis_name}, 方向: 0x{direction_high:02X}{direction_low:02X})')
        # self.get_logger().info(f'发布点动命令: {command_str} (轴号: 0x{axis_high:02X}{axis_low:02X}, 方向: 0x{direction_high:02X}{direction_low:02X})')

        # 如果有额外的数据，记录但不处理
        if len(payload) > 4:
            extra_data = payload[4:]  # 第5字节及以后的数据
            self.get_logger().info(f'忽略额外数据: {extra_data}')
        # # 根据表格：所有轴停止
        # command_str = "all:stop"
        # msg = String()
        # msg.data = command_str
        # self.jog_pub.publish(msg)
        # self.get_logger().info('发布所有轴停止命令')

    def process_clear_system_fault(self, payload):
        """处理清除系统故障和告警命令 (0x0117) - /fault_code"""
        # 根据需求：清除所有系统故障和告警
        # 通过发布 "clear_all" 到 /control_command 话题
        # 或直接调用 EthercatNode 的 clear_all_faults 方法
        command_str = "clear_all_faults"
        msg = String()
        msg.data = command_str
        self.control_pub.publish(msg)
        self.get_logger().info('发布清除系统故障和告警命令 (0x0117)')

    def process_clear_axis_fault(self, payload):
        """处理清除轴故障命令 (0x011B) - /control_command"""
        # 根据表格：清除轴故障
        command_str = "clear_fault"
        msg = String()
        msg.data = command_str
        self.control_pub.publish(msg)
        self.get_logger().info('发布清除轴故障命令')

    # 原有的处理函数保持不变
    def process_system_control(self, payload):
        """处理系统控制命令 (0x01)"""
        if len(payload) < 1:
            self.get_logger().warn('系统控制命令负载长度不足')
            return
            
        sub_command = payload[0]
        command_map = {
            0x01: "start_manual",
            0x02: "start_auto", 
            0x03: "stop",
            0x04: "clear_fault",
            0x05: "reset"
        }
        
        if sub_command in command_map:
            command_str = command_map[sub_command]
            msg = String()
            msg.data = command_str
            self.control_pub.publish(msg)
            self.get_logger().info(f'发布系统控制命令: {command_str}')
        else:
            self.get_logger().warn(f'未知系统控制子命令: 0x{sub_command:02X}')

    def process_jog_speed(self, payload):
        """处理点动速度命令 (0x03)"""
        if len(payload) < 5:  # 1字节轴ID + 4字节浮点数
            self.get_logger().warn('点动速度命令负载长度不足')
            return
            
        axis_id = payload[0]
        # 将4字节转换为浮点数 (小端序)
        speed_bytes = bytes(payload[1:5])
        try:
            speed = struct.unpack('<f', speed_bytes)[0]
            axis_name = f"axis{axis_id}"
            
            command_str = f"{axis_name}:{speed:.1f}"
            msg = String()
            msg.data = command_str
            self.jog_speed_pub.publish(msg)
            self.get_logger().info(f'发布点动速度命令: {command_str}')
        except Exception as e:
            self.get_logger().error(f'速度值解析错误: {e}')

    def process_position_control(self, payload):
        """处理位置控制命令 (0x04)"""
        if len(payload) < 5:  # 1字节轴ID + 4字节浮点数
            self.get_logger().warn('位置控制命令负载长度不足')
            return
            
        axis_id = payload[0]
        # 将4字节转换为浮点数 (小端序)
        position_bytes = bytes(payload[1:5])
        try:
            position = struct.unpack('<f', position_bytes)[0]
            axis_name = f"axis{axis_id}"
            
            # 发布到位移命令话题
            command_str = f"{axis_name}:{position:.1f}"
            msg = String()
            msg.data = command_str
            self.displacement_pub.publish(msg)
            self.get_logger().info(f'发布位置命令: {command_str}')
        except Exception as e:
            self.get_logger().error(f'位置值解析错误: {e}')

    def process_layer_command(self, payload):
        """处理层命令 (0x05)"""
        if len(payload) < 1:
            self.get_logger().warn('层命令负载长度不足')
            return
            
        layer = payload[0]
        msg = Int8()
        msg.data = layer
        self.layer_pub.publish(msg)
        self.get_logger().info(f'发布层命令: 第{layer}层')

    def process_board_width(self, payload):
        """处理板宽命令 (0x0B)"""
        if len(payload) < 4:  # 4字节浮点数
            self.get_logger().warn('板宽命令负载长度不足')
            return
            
        # 将4字节转换为浮点数 (小端序)
        width_bytes = bytes(payload[0:4])
        try:
            width = struct.unpack('<f', width_bytes)[0]
            msg = Float64()
            msg.data = float(width)
            self.board_width_pub.publish(msg)
            self.get_logger().info(f'发布板宽命令: {width:.1f}')
        except Exception as e:
            self.get_logger().error(f'板宽值解析错误: {e}')

def main(args=None):
    rclpy.init(args=args)
    parser_node = ByteMultiArrayParser()

    try:
        rclpy.spin(parser_node)
    except KeyboardInterrupt:
        if parser_node:
            print('ByteMultiArray解析器被用户中断')
    finally:
        parser_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()