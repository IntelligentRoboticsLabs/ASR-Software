"""
Launch file para fsm_oop_example con remappings configurables
Incluye nodo de simulación de entrada de usuario
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
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Usar tiempo de simulación'
    )
    
    # Configuración
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    scan_topic = LaunchConfiguration('scan_topic')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Nodo del ejemplo OOP con remappings
    oop_fsm_node = Node(
        package='fsm_examples',
        executable='fsm_oop_example',
        name='oop_fsm_robot',
        output='screen',
        remappings=[
            ('/cmd_vel', cmd_vel_topic),
            ('/scan', scan_topic),
        ],
        parameters=[{
            'use_sim_time': use_sim_time,
        }]
    )
    
    return LaunchDescription([
        cmd_vel_topic_arg,
        scan_topic_arg,
        use_sim_time_arg,
        oop_fsm_node,
    ])
