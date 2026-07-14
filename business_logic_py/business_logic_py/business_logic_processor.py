#!/usr/bin/env python3
"""
业务逻辑处理器 - 框架空壳

可复用的ROS2节点骨架，提供与ethercat_controller通信的标准接口。
具体业务逻辑（如入库/出库流程）应在此类基础上扩展实现。

标准通信接口：
  发布话题:
    /jog_command          (String)  → 点动控制
    /displacement_command (String)  → 位移控制
    /do_control           (String)  → DO控制

  订阅话题:
    /py_io_status         (String)  ← IO状态
    /system_status        (String)  ← 系统状态
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class BusinessLogicProcessor(Node):
    """业务逻辑处理器基类 - 框架模式"""

    def __init__(self):
        super().__init__('business_logic_processor')

        # === 标准接口：发布器 ===
        self.jog_pub = self.create_publisher(String, '/jog_command', 10)
        self.displacement_pub = self.create_publisher(String, '/displacement_command', 10)
        self.do_control_pub = self.create_publisher(String, '/do_control', 10)

        # === 标准接口：订阅器 ===
        self.io_status_sub = self.create_subscription(
            String, '/py_io_status', self.io_status_callback, 10)
        self.system_status_sub = self.create_subscription(
            String, '/system_status', self.system_status_callback, 10)

        # === 业务逻辑定时器 ===
        self.timer = self.create_timer(0.1, self.process_logic)

        self.get_logger().info('业务逻辑处理器已启动（框架模式）')

    def io_status_callback(self, msg):
        """IO状态回调 - 子类覆写实现具体逻辑"""
        pass

    def system_status_callback(self, msg):
        """系统状态回调 - 子类覆写实现具体逻辑"""
        pass

    def process_logic(self):
        """主处理逻辑 - 子类覆写实现具体逻辑"""
        pass


def main(args=None):
    rclpy.init(args=args)
    processor = BusinessLogicProcessor()
    try:
        rclpy.spin(processor)
    except KeyboardInterrupt:
        pass
    finally:
        processor.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
