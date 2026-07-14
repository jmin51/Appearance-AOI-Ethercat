#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray, Int8, Empty
import threading
import select
import sys
import tty
import termios
import time
import math

class ControlCommander(Node):
    # 轴列表 - 增减轴只需修改此处
    AXES = ['axis1', 'axis2', 'axis3', 'axis4', 'axis5', 'axis6', 'axis7', 'axis8']
    # 轴计算模式 - 与 axes_config.yaml 中 calc_mode 对应
    AXIS_CALC_MODES = {
        'axis1': 'ANGLE', 'axis2': 'ANGLE', 'axis3': 'ANGLE',
        'axis4': 'DISPLACEMENT', 'axis5': 'DISPLACEMENT',
        'axis6': 'ANGLE', 'axis7': 'ANGLE', 'axis8': 'ANGLE',
    }
    
    def __init__(self):
        # 1. 降低ROS2日志级别，减少控制台输出
        import rclpy.logging
        rclpy.logging.set_logger_level('control_commander', rclpy.logging.LoggingSeverity.WARN)
        
        super().__init__('control_commander')
        
        # 生成点动命令映射: 01=axis1正转, 02=axis1反转, 03=axis2正转, ...
        self._jog_commands = {}
        for i, axis in enumerate(self.AXES):
            idx = i * 2 + 1
            self._jog_commands[f'{idx:02d}'] = f'{axis}:forward'
            self._jog_commands[f'{idx+1:02d}'] = f'{axis}:reverse'
        
        # 创建发布器
        self.control_pub = self.create_publisher(String, '/control_command', 10)
        self.displacement_pub = self.create_publisher(String, '/displacement_command', 10)
        self.angle_pub = self.create_publisher(String, '/angle_command', 10)
        self.jog_pub = self.create_publisher(String, '/jog_command', 10)
        # 新增IO控制发布器
        self.io_control_pub = self.create_publisher(String, '/do_control', 10)
        # 新增仓库控制发布器
        self.warehouse_start_pub = self.create_publisher(Int8, '/warehouse_start', 10)
        self.warehouse_stop_pub = self.create_publisher(Empty, '/warehouse_stop', 10)
        # 新增出库控制发布器
        self.outbound_start_pub = self.create_publisher(Int8, '/outbound_start', 10)
        self.outbound_stop_pub = self.create_publisher(Empty, '/outbound_stop', 10)
        # 新增层控制发布器
        self.layer_pub = self.create_publisher(Int8, '/layer_command', 10)
        
        # 设置非阻塞输入
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())
        
        # 菜单状态变量
        self.current_menu = "main"  # 当前菜单状态：main, manual_mode, auto_mode
        self.manual_mode_active = False
        self.auto_mode_active = False
        
        # 位移控制参数 - 统一用 dict 跟踪各轴位置
        self.axis_positions = {axis: 0.0 for axis in self.AXES}
        self.position_step = 10.000  # 默认步长10mm
        
        # 按键缓冲区
        self.key_buffer = ""
        
        # 简化输出标志
        self.simplified_output = True
        
        # IO状态跟踪
        self.io_status = {
            '800': 0,  # 启动按钮灯
            '801': 0,  # 复位按钮灯
            '802': 0,  # 暂停按钮灯
            '803': 0,  # 蜂鸣器
            '804': 0,  # 三色红灯
            '805': 0,  # 三色黄灯
            '806': 0,  # 三色绿灯
            '810': 0,  # 顶升气缸下降
            '811': 0,  # 齿轮对接气缸伸出
            '812': 0,  # 皮带正转启动
            '813': 0,  # 皮带反转启动
        }
        
        self.get_logger().info('控制命令节点已启动')
        self.print_main_menu()
    
    def print_main_menu(self):
            """打印主菜单 - 减少换行符使用"""
            menu = (
                "\n" + "=" * 40 + "\n" +
                "ROS2 机械臂控制主菜单\n" +
                "=" * 40 + "\n" +
                "请选择操作模式或功能：\n" +
                "  01: 进入手动模式     02: 进入自动模式\n" +
                "  03: 停止所有轴       04: 清除故障\n" +
                "  05: 重置轴           06: 发送轨迹动作\n" +
                "  q : 退出程序\n" +
                "=" * 40 + "\n" +
                "请输入命令代码 (例如: 01 或 q): "
            )
            print(menu, end='', flush=True)
    
    def print_manual_menu(self):
            """打印手动控制菜单 - 减少换行符使用"""
            # 1. 构建轴控制区块字符串 - 从AXES列表自动生成
            jog_lines = []
            for i, axis in enumerate(self.AXES):
                idx = i * 2 + 1
                num = axis.replace('axis', '')
                jog_lines.append(f"  {idx:02d}:轴{num}正转点动   {idx+1:02d}:轴{num}反转点动")
            axis_ctrl_block = "[轴控制 - 点动]\n" + "\n".join(jog_lines) + "\n"
            
            # 2. 构建IO控制区块字符串
            # IO命令起始编号 = 轴数*2 + 1
            io_start = len(self.AXES) * 2 + 1
            io_lines = []
            io_list = [
                (f"{io_start:02d}:M800启动按钮灯", '800'), (f"{io_start+1:02d}:M801复位按钮灯", '801'),
                (f"{io_start+2:02d}:M802暂停按钮灯", '802'), (f"{io_start+3:02d}:M803蜂鸣器", '803'),
                (f"{io_start+4:02d}:M804三色红灯", '804'), (f"{io_start+5:02d}:M805三色黄灯", '805'),
                (f"{io_start+6:02d}:M806三色绿灯", '806'), (f"{io_start+7:02d}:M810顶升气缸下降", '810'),
                (f"{io_start+8:02d}:M811齿轮对接气缸伸出", '811'), (f"{io_start+9:02d}:M812皮带正转启动", '812'),
                (f"{io_start+10:02d}:M813皮带反转启动", '813')
            ]
            self._io_commands = {item[0].split(':')[0]: item[1] for item in io_list}
            for i in range(0, len(io_list), 4):
                line_parts = []
                for j in range(4):
                    idx = i + j
                    if idx < len(io_list):
                        name, addr = io_list[idx]
                        status = "ON" if self.io_status[addr] else "OFF"
                        line_parts.append(f"  {name}[{status}]")
                io_lines.append("".join(line_parts))
            io_ctrl_block = "[IO控制] (当前状态)\n" + "\n".join(io_lines) + "\n"
            
            # 3. 构建其他控制区块字符串
            other_ctrl_block = (
                "\n[其他控制]\n" +
                "  s:停止所有轴点动  r:复位所有IO  b:返回主菜单\n"
            )
            
            # 4. 拼接最终菜单字符串并打印
            menu = (
                "\n" + "=" * 50 + "\n" +
                "手动模式 - 轴控制与IO控制\n" +
                "=" * 50 + "\n" +
                axis_ctrl_block + "\n" +
                io_ctrl_block +
                other_ctrl_block +
                "=" * 50 + "\n" +
                "请输入命令代码: "
            )
            print(menu, end='', flush=True)
    
    def print_auto_menu(self):
            """打印自动控制菜单"""
            # 1. 点动控制区块 - 从AXES列表自动生成
            jog_parts = []
            for i, axis in enumerate(self.AXES):
                idx = i * 2 + 1
                num = axis.replace('axis', '')
                jog_parts.append(f"  {idx:02d}:轴{num}正转  {idx+1:02d}:轴{num}反转")
            jog_ctrl_block = "[点动控制]\n" + "\n".join(jog_parts) + "\n"
            
            # 2. 轴位置控制区块 - 自动生成 (命令编号从40开始，避免与点动01-16/IO17-27冲突)
            pos_lines = []
            self._pos_commands = {}  # 位置控制命令映射
            cmd_idx = 40
            for axis in self.AXES:
                num = axis.replace('axis', '')
                mode = self.AXIS_CALC_MODES.get(axis, 'ANGLE')
                if axis in ('axis4', 'axis5'):
                    # 位移轴: 提供 预设位移 + 位移输入 + 角度输入
                    pos = self.axis_positions.get(axis, 0.0)
                    self._pos_commands[f'{cmd_idx:02d}'] = (axis, 'disp_origin')
                    self._pos_commands[f'{cmd_idx+1:02d}'] = (axis, 'disp_preset')
                    self._pos_commands[f'{cmd_idx+2:02d}'] = (axis, 'disp_forward')
                    self._pos_commands[f'{cmd_idx+3:02d}'] = (axis, 'disp_reverse')
                    self._pos_commands[f'{cmd_idx+4:02d}'] = (axis, 'disp_input')
                    self._pos_commands[f'{cmd_idx+5:02d}'] = (axis, 'angle_input')
                    pos_lines.append(
                        f"  轴{num} (位移, 当前:{pos:.1f}mm): "
                        f"{cmd_idx:02d}:回原点  {cmd_idx+1:02d}:→10mm  "
                        f"{cmd_idx+2:02d}:+{self.position_step:.0f}mm  {cmd_idx+3:02d}:-{self.position_step:.0f}mm  "
                        f"{cmd_idx+4:02d}:位移输入  {cmd_idx+5:02d}:角度输入"
                    )
                    cmd_idx += 6
                elif mode == 'ANGLE':
                    # 角度轴: 提供 角度输入 + 位移输入
                    self._pos_commands[f'{cmd_idx:02d}'] = (axis, 'angle_input')
                    self._pos_commands[f'{cmd_idx+1:02d}'] = (axis, 'disp_input')
                    pos_lines.append(
                        f"  轴{num} (角度): {cmd_idx:02d}:角度输入(°)  {cmd_idx+1:02d}:位移输入(mm)"
                    )
                    cmd_idx += 2
                else:
                    # 纯位移轴
                    self._pos_commands[f'{cmd_idx:02d}'] = (axis, 'disp_input')
                    pos_lines.append(
                        f"  轴{num} (位移): {cmd_idx:02d}:位移输入(mm)"
                    )
                    cmd_idx += 1
            pos_ctrl_block = "[轴位置控制]\n" + "\n".join(pos_lines) + "\n"
            
            # 3. 双轴位移控制已移除
            # 4. IO控制区块
            io_start = len(self.AXES) * 2 + 1
            io_lines = []
            io_list = [
                (f"{io_start:02d}:M800启动按钮灯", '800'), (f"{io_start+1:02d}:M801复位按钮灯", '801'),
                (f"{io_start+2:02d}:M802暂停按钮灯", '802'), (f"{io_start+3:02d}:M803蜂鸣器", '803'),
                (f"{io_start+4:02d}:M804三色红灯", '804'), (f"{io_start+5:02d}:M805三色黄灯", '805'),
                (f"{io_start+6:02d}:M806三色绿灯", '806'), (f"{io_start+7:02d}:M810顶升气缸下降", '810'),
                (f"{io_start+8:02d}:M811齿轮对接气缸伸出", '811'), (f"{io_start+9:02d}:M812皮带正转启动", '812'),
                (f"{io_start+10:02d}:M813皮带反转启动", '813')
            ]
            self._auto_io_commands = {item[0].split(':')[0]: item[1] for item in io_list}
            for i in range(0, len(io_list), 4):
                line_parts = []
                for j in range(4):
                    idx = i + j
                    if idx < len(io_list):
                        name, addr = io_list[idx]
                        status = "ON" if self.io_status[addr] else "OFF"
                        line_parts.append(f"  {name}[{status}]")
                io_lines.append("".join(line_parts))
            io_ctrl_block = "\n[IO控制] (当前状态)" + "\t".join(io_lines) + "\t"
            
            # 5. 仓库控制区块
            warehouse_block = (
                "\n[仓库控制]" +
                "  30:启动入库  31:停止入库  32:启动出库  33:停止出库\t"
            )
            
            # 6. 层控制区块
            layer_block = (
                "\t[层控制]" +
                "  34:移动到指定层（请输入层号: -20~30）\t"
            )
            
            # 7. 其他控制区块
            other_ctrl_block = (
                "\t[其他控制]" +
                "  s:停止所有轴(含点动和位移)  r:复位所有IO  b:返回主菜单\t"
            )
            
            # 8. 拼接最终菜单
            menu = (
                "\n" + "=" * 60 + "\n" +
                "自动模式 - 位移控制与IO控制\n" +
                "=" * 60 + "\n" +
                jog_ctrl_block + "\n" +
                pos_ctrl_block + "\n" +
                io_ctrl_block +
                warehouse_block +
                layer_block +
                other_ctrl_block +
                "=" * 60 + "\n" +
                "请输入命令代码: "
            )
            print(menu, end='', flush=True)
    
    def send_control_command(self, command_data):
        """发送控制命令"""
        msg = String()
        msg.data = command_data
        self.control_pub.publish(msg)
        if not self.simplified_output:
            self.get_logger().info('已发送控制命令: {}'.format(command_data))
    
    def send_displacement_command(self, axis_name, position):
        """发送位移命令 - 格式: 'axis4:0.0' 或 'axis5:10.0'"""
        msg = String()
        msg.data = '{}:{:.3f}'.format(axis_name, position)
        self.displacement_pub.publish(msg)
        if not self.simplified_output:
            self.get_logger().info('已发送位移命令: {}'.format(msg.data))
    
    def send_jog_command(self, command):
        """发送点动命令"""
        msg = String()
        msg.data = command
        self.jog_pub.publish(msg)
        if not self.simplified_output:
            self.get_logger().info('已发送点动命令: {}'.format(command))
    
    def send_angle_command(self, axis_name, angle_rad):
        """发送角度命令 - 格式: 'axis6:1.5708' (单位: rad)"""
        msg = String()
        msg.data = '{}:{:.6f}'.format(axis_name, angle_rad)
        self.angle_pub.publish(msg)
        if not self.simplified_output:
            self.get_logger().info('已发送角度命令: {}'.format(msg.data))
    
    def send_io_control(self, io_address, value):
        """发送IO控制命令 - 格式: '801:1'"""
        msg = String()
        msg.data = '{}:{}'.format(io_address, value)
        self.io_control_pub.publish(msg)
        if not self.simplified_output:
            self.get_logger().info('已发送IO控制命令: {}'.format(msg.data))
        return msg.data
    
    def toggle_io(self, io_address):
        """切换IO状态并发送控制命令"""
        # 切换状态 (0->1, 1->0)
        self.io_status[io_address] = 1 - self.io_status[io_address]
        # 发送控制命令
        command = self.send_io_control(io_address, self.io_status[io_address])
        # 显示状态
        io_names = {
            '800': '启动按钮灯',
            '801': '复位按钮灯', 
            '802': '暂停按钮灯',
            '803': '蜂鸣器',
            '804': '三色红灯',
            '805': '三色黄灯',
            '806': '三色绿灯',
            '810': '顶升气缸下降',
            '811': '齿轮对接气缸伸出',
            '812': '皮带正转启动',
            '813': '皮带反转启动'
        }
        status = "开启" if self.io_status[io_address] else "关闭"
        print("{} {} ({})".format(io_names.get(io_address, 'IO'), status, command))
        # 刷新菜单显示新状态
        if self.current_menu == "manual_mode" or self.current_menu == "auto_mode":
            if self.current_menu == "manual_mode":
                self.print_manual_menu()
            else:
                self.print_auto_menu()
    
    def reset_all_io(self):
        """复位所有IO状态"""
        for io_address in self.io_status:
            if self.io_status[io_address] == 1:  # 只关闭已开启的IO
                self.io_status[io_address] = 0
                self.send_io_control(io_address, 0)
        print("所有IO已复位")
        # 刷新菜单
        if self.current_menu == "manual_mode" or self.current_menu == "auto_mode":
            if self.current_menu == "manual_mode":
                self.print_manual_menu()
            else:
                self.print_auto_menu()
    
    def send_warehouse_start(self):
        """发送入库启动命令"""
        msg = Int8()
        msg.data = 24
        self.warehouse_start_pub.publish(msg)
        print("启动入库")
    
    def send_warehouse_stop(self):
        """发送入库停止命令"""
        msg = Empty()
        self.warehouse_stop_pub.publish(msg)
        print("停止入库")
    
    def send_outbound_start(self):
        """发送出库启动命令"""
        msg = Int8()
        msg.data = 24
        self.outbound_start_pub.publish(msg)
        print("启动出库")
    
    def send_outbound_stop(self):
        """发送出库停止命令"""
        msg = Empty()
        self.outbound_stop_pub.publish(msg)
        print("停止出库")
    
    def stop_all_jog(self):
        """停止所有轴的点动"""
        for axis in self.AXES:
            msg = String()
            msg.data = '{}:stop'.format(axis)
            self.jog_pub.publish(msg)
        print("停止所有轴")
    
    def enter_manual_mode(self):
        """进入手动模式"""
        self.send_control_command('start_manual')
        time.sleep(0.1)
        self.current_menu = "manual_mode"
        self.manual_mode_active = True
        self.auto_mode_active = False
        self.print_manual_menu()
    
    def enter_auto_mode(self):
        """进入自动模式"""
        self.send_control_command('start_auto')
        time.sleep(0.1)
        self.current_menu = "auto_mode"
        self.auto_mode_active = True
        self.manual_mode_active = False
        self.print_auto_menu()
    
    def return_to_main_menu(self):
        """返回主菜单 - 不发送任何话题"""
        self.current_menu = "main"
        self.manual_mode_active = False
        self.auto_mode_active = False
        self.stop_all_jog()  # 返回主菜单时停止所有轴
        self.send_control_command('stop')  # 发送停止命令以确保所有运动停止
        self.print_main_menu()
    
    def handle_manual_mode_command(self, key):
        """处理手动模式下的命令"""
        if key in self._jog_commands:
            self.send_jog_command(self._jog_commands[key])
            axis_num = self._jog_commands[key].split(':')[0].replace('axis', '')
            direction = '正转' if 'forward' in self._jog_commands[key] else '反转'
            print(f"轴{axis_num}{direction}执行")
        elif key in self._io_commands:
            self.toggle_io(self._io_commands[key])
        elif key == 's':
            self.stop_all_jog()
        elif key == 'r':
            self.reset_all_io()
        elif key == 'b':
            self.return_to_main_menu()
        else:
            print("未知命令: {}".format(key))
            self.print_manual_menu()
    
    def handle_auto_mode_command(self, key):
        """处理自动模式下的命令"""
        if key in self._jog_commands:
            self.send_jog_command(self._jog_commands[key])
            axis_num = self._jog_commands[key].split(':')[0].replace('axis', '')
            direction = '正转' if 'forward' in self._jog_commands[key] else '反转'
            print(f"轴{axis_num}{direction}执行")
        elif hasattr(self, '_pos_commands') and key in self._pos_commands:
            # 通用轴位置控制处理
            axis_name, action = self._pos_commands[key]
            num = axis_name.replace('axis', '')
            step = self.position_step
            if action == 'disp_origin':
                self.axis_positions[axis_name] = 0.0
                self.send_displacement_command(axis_name, 0.0)
                print(f"轴{num}移动到原点")
            elif action == 'disp_preset':
                self.axis_positions[axis_name] = 10.0
                self.send_displacement_command(axis_name, 10.0)
                print(f"轴{num}移动到10mm")
            elif action == 'disp_forward':
                self.axis_positions[axis_name] += step
                self.send_displacement_command(axis_name, self.axis_positions[axis_name])
                print(f"轴{num}正向移动{step:.0f}mm")
            elif action == 'disp_reverse':
                self.axis_positions[axis_name] -= step
                self.send_displacement_command(axis_name, self.axis_positions[axis_name])
                print(f"轴{num}反向移动{step:.0f}mm")
            elif action == 'disp_input':
                self.input_axis_displacement(axis_name)
            elif action == 'angle_input':
                self.input_axis_angle(axis_name)
            self.print_auto_menu()
        elif hasattr(self, '_auto_io_commands') and key in self._auto_io_commands:
            self.toggle_io(self._auto_io_commands[key])
        elif key == 's':
            self.stop_all_jog()
        elif key == 'r':  # 复位所有IO
            self.reset_all_io()
        elif key == 'b':
            self.return_to_main_menu()
        # 仓库控制命令
        elif key == '30':
            self.send_warehouse_start()
        elif key == '31':
            self.send_warehouse_stop()
        elif key == '32':
            self.send_outbound_start()
        elif key == '33':
            self.send_outbound_stop()
        # 层控制命令
        elif key == '34':
            print("请输入要移动到的层号（-20~30），按回车确认: ", end='', flush=True)
            import termios
            import tty
            import sys
            old_settings = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
            try:
                layer_input = ""
                while True:
                    char = sys.stdin.read(1)
                    if char == '\n' or char == '\r':
                        break
                    elif char == '\x7f' or char == '\b':
                        if layer_input:
                            layer_input = layer_input[:-1]
                            sys.stdout.write('\b \b')
                            sys.stdout.flush()
                    else:
                        layer_input += char
                        sys.stdout.write(char)
                        sys.stdout.flush()
                sys.stdout.write('\n')
                sys.stdout.flush()
                if layer_input:
                    try:
                        layer_num = int(layer_input)
                        if -20 <= layer_num <= 30:
                            msg = Int8()
                            msg.data = layer_num
                            self.layer_pub.publish(msg)
                            print(f"已发送移动至层 {layer_num} 的命令")
                        else:
                            print(f"错误：层号 {layer_num} 超出范围（-20~30）")
                    except ValueError:
                        print(f"错误：'{layer_input}' 不是有效的整数")
            finally:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
                tty.setraw(sys.stdin.fileno())
            self.print_auto_menu()
        else:
            print("未知命令: {}".format(key))
            self.print_auto_menu()
    
    def handle_main_menu_command(self, key):
        """处理主菜单命令"""
        main_commands = {
            '01': lambda: self.enter_manual_mode(),
            '02': lambda: self.enter_auto_mode(),
            '03': lambda: (self.send_control_command('stop'), print("停止所有轴")),
            '04': lambda: (self.send_control_command('clear_fault'), print("清除故障")),
            '05': lambda: (self.send_control_command('reset'), print("重置轴")),
            '06': lambda: self.send_trajectory_action(),
        }
        
        if key in main_commands:
            main_commands[key]()
        elif key == 'q':
            # 恢复终端设置
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
            # 退出程序
            print("退出程序")
            sys.exit(0)
        else:
            print("未知命令: {}".format(key))
            self.print_main_menu()
    
    def send_trajectory_action(self):
        """发送轨迹动作"""
        print("轨迹动作需要单独执行action命令")
        print("请手动执行: ros2 action send_goal /arm_2zhou_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory")

    def input_axis_angle(self, axis_name):
        """输入轴角度值(°)并发送角度命令"""
        num = axis_name.replace('axis', '')
        print(f"\n========== 轴{num} 角度输入 ==========")
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
        try:
            val_str = input(f"请输入轴{num}角度值(°):").strip()
            angle_deg = float(val_str)
            angle_rad = math.radians(angle_deg)  # 度 → 弧度
            self.send_angle_command(axis_name, angle_rad)
            self.axis_positions[axis_name] = angle_deg  # 记录度数值，供菜单显示
            print(f"已发送: 轴{num} 角度={angle_deg:.2f}° ({angle_rad:.4f} rad)")
        except ValueError:
            print("输入格式错误，请输入数字！")
        except Exception as e:
            print(f"输入异常：{e}")
        finally:
            tty.setraw(sys.stdin.fileno())
            print("==================================\n")
    
    def input_axis_displacement(self, axis_name):
        """输入轴位移值(mm)并发送位移命令"""
        num = axis_name.replace('axis', '')
        print(f"\n========== 轴{num} 位移输入 ==========")
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
        try:
            val_str = input(f"请输入轴{num}位移值(mm):").strip()
            disp_val = float(val_str)
            self.send_displacement_command(axis_name, disp_val)
            self.axis_positions[axis_name] = disp_val
            print(f"已发送: 轴{num} 位移={disp_val:.3f} mm")
        except ValueError:
            print("输入格式错误，请输入数字！")
        except Exception as e:
            print(f"输入异常：{e}")
        finally:
            tty.setraw(sys.stdin.fileno())
            print("==================================\n")

    def get_key(self):
        """获取按键输入 - 必须输入两位数字或单字母"""
        if select.select([sys.stdin], [], [], 0.1)[0]:
            char = sys.stdin.read(1)
            
            # 如果是字母，直接返回
            if char.isalpha():
                return char
            
            # 如果是数字，必须读取第二位数字
            if char.isdigit():
                # 等待第二位数字 延迟1s内需完成输入
                if select.select([sys.stdin], [], [], 1.0)[0]:
                    char2 = sys.stdin.read(1)
                    if char2.isdigit():
                        return char + char2
                    else:
                        # 如果第二个字符不是数字，忽略输入
                        print("请输入两位数字命令")
                        return None
                else:
                    # 如果超时，忽略输入
                    print("请输入两位数字命令")
                    return None
            
            # 其他字符忽略
            return None
        
        return None
    
    def run(self):
        """主运行循环"""
        try:
            while rclpy.ok():
                key = self.get_key()
                if key:
                    # 根据当前菜单状态处理按键
                    if self.current_menu == "main":
                        self.handle_main_menu_command(key)
                    elif self.current_menu == "manual_mode":
                        self.handle_manual_mode_command(key)
                    elif self.current_menu == "auto_mode":
                        self.handle_auto_mode_command(key)
                
                time.sleep(0.1)
                
        finally:
            # 恢复终端设置
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
            self.get_logger().info('控制命令节点已关闭')

def main(args=None):
    rclpy.init(args=args)
    
    commander = ControlCommander()
    
    # 在单独的线程中运行ROS2 spinning
    spin_thread = threading.Thread(target=rclpy.spin, args=(commander,), daemon=True)
    spin_thread.start()
    
    try:
        commander.run()
    except KeyboardInterrupt:
        print("\n程序被用户中断")
    finally:
        commander.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
