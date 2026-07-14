#!/usr/bin/env python3
"""
ROS2 ByteMultiArray 发布脚本 - 支持多种指令
支持入库指令（0x0101）、出库指令（0x0103）、结束作业指令（0x0107）、
轴点动指令（0x010D）、IO控制指令（0x0115）、清除系统故障（0x0117）和清除轴故障（0x011B）
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import ByteMultiArray, MultiArrayLayout, MultiArrayDimension
import argparse
import sys

def main():
    # 解析命令行参数
    parser = argparse.ArgumentParser(description='发布ByteMultiArray消息')
    parser.add_argument('--command', type=str, required=True, 
                        choices=['start', 'warehouse', 'outbound', 'stop', 'jog', 'io', 
                                'start_result', 'clear_system_fault', 'clear_axis_fault'], 
                        help='指令类型: start(开始作业) 或 warehouse(入库) 或 outbound(出库) 或 stop(结束作业) 或 jog(轴点动) 或 io(IO控制) 或 start_result(开始结果) 或 clear_system_fault(清除系统故障) 或 clear_axis_fault(清除轴故障)')
    parser.add_argument('--layer', type=int, default=1,
                       help='层高（仅warehouse和outbound指令有效，默认1）')
    parser.add_argument('--width', type=float, default=15.0,
                    help='板宽值，单位厘米（仅start指令有效，默认15.0）')
    parser.add_argument('--axis', type=int, default=1,
                       help='轴号（仅jog指令有效，默认1）')
    parser.add_argument('--direction', type=int, choices=[0, 1, 2], default=1,
                       help='方向: 0=停止, 1=正转, 2=反转（仅jog指令有效，默认1）')
    parser.add_argument('--io_low', type=lambda x: int(x, 0), default=0x00,
                       help='IO状态低位字节（仅io指令有效，十六进制，默认0x00）')
    parser.add_argument('--io_high', type=lambda x: int(x, 0), default=0x00,
                       help='IO状态高位字节（仅io指令有效，十六进制，默认0x00）')
    
    args = parser.parse_args()
    
    rclpy.init()
    
    # 创建节点
    node = Node('byte_multiarray_publisher')
    publisher = node.create_publisher(ByteMultiArray, '/integrated_control', 10)
    
    # 等待连接建立
    rclpy.spin_once(node, timeout_sec=1.0)
    
    # 创建MultiArrayLayout
    layout = MultiArrayLayout()
    layout.data_offset = 0
    
    # 根据指令类型构造不同的消息
    if args.command == 'start':
        # 开始作业指令 (0x0105)，附带4字节板宽信息
        # 格式: [指令码低位0x05, 指令码高位0x01, 板宽数据(2字节，小端序)]
        
        # 1. 将板宽（厘米）转换为放大10倍后的整数(先不放大)
        width_integer = int(args.width * 1.0)
        
        # 2. 将整数拆分为4字节，小端序
        width_bytes = width_integer.to_bytes(2, byteorder='little', signed=False)
        
        # 3. 构造消息数据：指令码(2字节) + 板宽数据(2字节)
        msg_data = [
            bytes([0x05]),  # 指令码低位
            bytes([0x01]),  # 指令码高位 (0x0105 = 开始作业指令)
            bytes([width_bytes[0]]), # 板宽数据低位 (width_low) 转换为bytes
            bytes([width_bytes[1]]), # 板宽数据高位 (width_high) 转换为bytes
            bytes([0x00]),           # 额外字节1
            bytes([0x00]),           # 额外字节2
        ]
        # # 将2字节板宽数据依次加入
        # for b in width_bytes:
        #     msg_data.append(bytes([b]))
        
        layout.dim = [MultiArrayDimension()]
        layout.dim[0].label = 'start_command_with_width'
        layout.dim[0].size = 6  # 总字节数变为 2 + 2 = 4
        layout.dim[0].stride = 1
        
        node.get_logger().info(f'构造开始作业指令: 进入自动模式，板宽={args.width}cm (编码值={width_integer})')

    elif args.command == 'start_result':
        # 开始结果指令 (0x0106)
        # 格式: [指令码低位0x06, 指令码高位0x01] (不需要负载)

        msg_data = [
            bytes([0x06]),  # 指令码低位
            bytes([0x01]),  # 指令码高位 (0x0106 = 开始结果指令)
            bytes([0x00]),  # 填充字节0
            bytes([0x00])   # 填充字节0
        ]

        layout.dim = [MultiArrayDimension()]
        layout.dim[0].label = 'start_result_command'
        layout.dim[0].size = 4
        layout.dim[0].stride = 1

        node.get_logger().info('构造开始结果指令: 指令码 0x0106')

    elif args.command == 'warehouse':
        # 入库指令 (0x0101)
        # 格式: [指令码低位0x01, 指令码高位0x01, 组号低位0x00, 组号高位0x00, 
        #        IO状态低位0x00, IO状态高位0x00, 层高低位, 层高高位]
        
        # 将层高转换为小端序的两个字节
        layer_low = args.layer & 0xFF  # 低8位
        layer_high = (args.layer >> 8) & 0xFF  # 高8位
        
        msg_data = [
            bytes([0x01]),  # 指令码低位
            bytes([0x01]),  # 指令码高位 (0x0101 = 入库指令)
            bytes([0x00]),  # 组号低位
            bytes([0x00]),  # 组号高位
            bytes([0x00]),  # IO状态低位
            bytes([0x00]),  # IO状态高位
            bytes([layer_low]),   # 层高低位
            bytes([layer_high])   # 层高高位
        ]
        
        layout.dim = [MultiArrayDimension()]
        layout.dim[0].label = 'warehouse_command'
        layout.dim[0].size = 8
        layout.dim[0].stride = 1
        
        node.get_logger().info(f'构造入库指令: 层高={args.layer} (0x{layer_low:02X} 0x{layer_high:02X})')
        
    elif args.command == 'outbound':
        # 出库指令 (0x0103)
        # 格式: [指令码低位0x03, 指令码高位0x01, 组号低位0x00, 组号高位0x00, 
        #        IO状态低位0x00, IO状态高位0x00, 层高低位, 层高高位]
        
        # 将层高转换为小端序的两个字节
        layer_low = args.layer & 0xFF  # 低8位
        layer_high = (args.layer >> 8) & 0xFF  # 高8位
        
        msg_data = [
            bytes([0x03]),  # 指令码低位
            bytes([0x01]),  # 指令码高位 (0x0103 = 出库指令)
            bytes([0x00]),  # 组号低位
            bytes([0x00]),  # 组号高位
            bytes([0x00]),  # IO状态低位
            bytes([0x00]),  # IO状态高位
            bytes([layer_low]),   # 层高低位
            bytes([layer_high])   # 层高高位
        ]
        
        layout.dim = [MultiArrayDimension()]
        layout.dim[0].label = 'outbound_command'
        layout.dim[0].size = 8
        layout.dim[0].stride = 1
        
        node.get_logger().info(f'构造出库指令: 层高={args.layer} (0x{layer_low:02X} 0x{layer_high:02X})')
        
    elif args.command == 'stop':
        # 结束作业指令 (0x0107)
        # 格式: [指令码低位0x07, 指令码高位0x01]
        
        msg_data = [
            bytes([0x07]),  # 指令码低位
            bytes([0x01]),  # 指令码高位 (0x0107 = 结束作业指令)
        ]
        
        layout.dim = [MultiArrayDimension()]
        layout.dim[0].label = 'stop_command'
        layout.dim[0].size = 2
        layout.dim[0].stride = 1
        
        node.get_logger().info('构造结束作业指令')
        
    elif args.command == 'jog':
        # 轴点动指令 (0x010D)
        # 格式: [指令码低位0x0D, 指令码高位0x01, 轴号低位, 轴号高位, 方向低位, 方向高位, 填充0...] (共14字节)
        
        # 将轴号和方向转换为小端序的两个字节
        axis_low = args.axis & 0xFF  # 轴号低位
        axis_high = (args.axis >> 8) & 0xFF  # 轴号高位
        direction_low = args.direction & 0xFF  # 方向低位
        direction_high = (args.direction >> 8) & 0xFF  # 方向高位
        
        # 构造14字节数据
        msg_data = [
            bytes([0x0D]),  # 指令码低位
            bytes([0x01]),  # 指令码高位 (0x010D = 轴点动指令)
            bytes([axis_low]),   # 轴号低位
            bytes([axis_high]),  # 轴号高位
            bytes([direction_low]),   # 方向低位
            bytes([direction_high]), # 方向高位
        ]
        
        # 填充剩余8字节为0（共14字节）
        for i in range(8):
            msg_data.append(bytes([0x00]))
        
        layout.dim = [MultiArrayDimension()]
        layout.dim[0].label = 'jog_command'
        layout.dim[0].size = 14
        layout.dim[0].stride = 1
        
        direction_map = {0: "停止", 1: "正转", 2: "反转"}
        direction_str = direction_map.get(args.direction, "未知")
        node.get_logger().info(f'构造轴点动指令: 轴号={args.axis}, 方向={direction_str}')
        
    elif args.command == 'io':
        # IO控制指令 (0x0115)
        # 格式: [指令码低位0x15, 指令码高位0x01, 组号低位0x00, 组号高位0x00, 
        #        IO状态低位, IO状态高位]
        
        msg_data = [
            bytes([0x15]),  # 指令码低位
            bytes([0x01]),  # 指令码高位 (0x0115 = IO控制指令)
            bytes([0x00]),  # 组号低位
            bytes([0x00]),  # 组号高位
            bytes([args.io_low]),   # IO状态低位
            bytes([args.io_high])   # IO状态高位
        ]
        
        layout.dim = [MultiArrayDimension()]
        layout.dim[0].label = 'io_command'
        layout.dim[0].size = 6
        layout.dim[0].stride = 1
        
        node.get_logger().info(f'构造IO控制指令: IO状态=0x{args.io_high:02X}{args.io_low:02X}')
    
    # +++ 新增：清除系统故障指令 (0x0117) +++
    elif args.command == 'clear_system_fault':
        # 清除系统故障和告警指令 (0x0117)
        # 格式: [指令码低位0x17, 指令码高位0x01]
        
        msg_data = [
            bytes([0x17]),  # 指令码低位
            bytes([0x01]),  # 指令码高位 (0x0117 = 清除系统故障和告警指令)
        ]
        
        layout.dim = [MultiArrayDimension()]
        layout.dim[0].label = 'clear_system_fault_command'
        layout.dim[0].size = 2
        layout.dim[0].stride = 1
        
        node.get_logger().info('构造清除系统故障和告警指令: 指令码 0x0117')
    
    # +++ 新增：清除轴故障指令 (0x011B) +++
    elif args.command == 'clear_axis_fault':
        # 清除轴故障指令 (0x011B)
        # 格式: [指令码低位0x1B, 指令码高位0x01]
        
        msg_data = [
            bytes([0x1B]),  # 指令码低位
            bytes([0x01]),  # 指令码高位 (0x011B = 清除轴故障指令)
        ]
        
        layout.dim = [MultiArrayDimension()]
        layout.dim[0].label = 'clear_axis_fault_command'
        layout.dim[0].size = 2
        layout.dim[0].stride = 1
        
        node.get_logger().info('构造清除轴故障指令: 指令码 0x011B')
    
    # 创建ByteMultiArray消息
    msg = ByteMultiArray()
    msg.layout = layout
    msg.data = msg_data
    
    try:
        # 发送消息
        publisher.publish(msg)
        
        # 日志输出
        data_hex = ' '.join([f'0x{b.hex().upper()}' for b in msg.data])
        node.get_logger().info(f'发布ByteMultiArray消息: [{data_hex}]')
        node.get_logger().info('消息已发送，程序退出')
        
    except Exception as e:
        node.get_logger().error(f'发生错误: {e}')
        import traceback
        traceback.print_exc()
    finally:
        # 短暂等待确保消息发送
        rclpy.spin_once(node, timeout_sec=0.1)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()