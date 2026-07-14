from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 业务逻辑处理器节点
        Node(
            package='business_logic_py',
            executable='business_logic_node',
            name='business_logic_processor',
            output='screen',
            parameters=[{
                'use_unified_protocol': False  # 参数控制是否使用统一协议
            }]
        ),
        
        # ByteMultiArray解析器节点
        Node(
            package='business_logic_py',
            executable='byte_multiarray_parser',
            name='byte_multiarray_parser',
            output='screen'
        ),
    ])