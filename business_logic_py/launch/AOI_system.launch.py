from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 定义启动参数
    use_unified_protocol_arg = DeclareLaunchArgument(
        'use_unified_protocol',
        default_value='False',
        description='控制是否使用统一协议'
    )
    
    return LaunchDescription([
        use_unified_protocol_arg,
        
        # 业务逻辑处理器节点 (Python节点)
        Node(
            package='business_logic_py',
            executable='business_logic_node',
            name='business_logic_processor',
            output='screen',
            parameters=[{
                'use_unified_protocol': LaunchConfiguration('use_unified_protocol')
            }]
        ),
        
        # ByteMultiArray解析器节点 (Python节点)
        Node(
            package='business_logic_py',
            executable='byte_multiarray_parser',
            name='byte_multiarray_parser',
            output='screen'
        ),
        
        # EtherCAT控制器节点 (C++节点)
        Node(
            package='ethercat_controller',
            executable='ethercat_controller',
            name='ethercat_controller',
            output='screen',
            emulate_tty=True,  # 确保C++节点的输出能正确显示
            parameters=[],  # 可以添加必要的参数文件
            on_exit=Shutdown()
        ),
    ])