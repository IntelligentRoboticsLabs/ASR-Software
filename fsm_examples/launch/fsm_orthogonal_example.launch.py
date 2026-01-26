"""
Launch file para fsm_orthogonal_example con remappings configurables
Incluye simulador de batería con tiempo de drenaje configurable
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Argumentos de lanzamiento
    cmd_vel_topic_arg = DeclareLaunchArgument(
        'cmd_vel_topic',
        default_value='/cmd_vel',
        description='Topic para publicar comandos de velocidad'
    )
    
    scan_topic_arg = DeclareLaunchArgument(
        'scan_topic',
        default_value='/scan_raw',
        description='Topic para recibir datos del láser'
    )
    
    battery_topic_arg = DeclareLaunchArgument(
        'battery_topic',
        default_value='/battery_state',
        description='Topic para recibir estado de batería'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Usar tiempo de simulación'
    )
    
    drain_time_arg = DeclareLaunchArgument(
        'drain_time',
        default_value='60.0',
        description='Tiempo en segundos para drenar la batería de 100% a 0%'
    )
    
    # Configuración
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    scan_topic = LaunchConfiguration('scan_topic')
    battery_topic = LaunchConfiguration('battery_topic')
    use_sim_time = LaunchConfiguration('use_sim_time')
    drain_time = LaunchConfiguration('drain_time')
    
    # Simulador de batería
    battery_simulator_node = Node(
        package='fsm_examples',
        executable='battery_simulator',
        name='battery_simulator',
        output='screen',
        remappings=[
            ('/battery_state', battery_topic),
        ],
        parameters=[{
            'use_sim_time': use_sim_time,
            'drain_time': drain_time,
        }]
    )
    
    # Nodo del ejemplo ortogonal con remappings
    orthogonal_fsm_node = Node(
        package='fsm_examples',
        executable='fsm_orthogonal_example',
        name='orthogonal_robot',
        output='screen',
        remappings=[
            ('/cmd_vel', cmd_vel_topic),
            ('/scan', scan_topic),
            ('/battery_state', battery_topic),
        ],
        parameters=[{
            'use_sim_time': use_sim_time,
        }]
    )
    
    return LaunchDescription([
        cmd_vel_topic_arg,
        scan_topic_arg,
        battery_topic_arg,
        use_sim_time_arg,
        drain_time_arg,
        battery_simulator_node,
        orthogonal_fsm_node,
    ])
