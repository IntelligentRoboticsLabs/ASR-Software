from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('bt_examples')
    
    # Argumentos de lanzamiento
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    charger_x_arg = DeclareLaunchArgument(
        'charger_x',
        default_value='0.0',
        description='Charger X coordinate'
    )
    
    charger_y_arg = DeclareLaunchArgument(
        'charger_y',
        default_value='0.0',
        description='Charger Y coordinate'
    )
    
    # Nodo del behavior tree
    bumpandgo_node = Node(
        package='bt_examples',
        executable='bumpandgo_bt_example',
        name='bumpandgo_bt',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'charger_x': LaunchConfiguration('charger_x'),
            'charger_y': LaunchConfiguration('charger_y'),
        }],
        remappings=[
            ('/scan', '/scan_raw')
        ]
    )
    
    battery_sim_node = Node(
        package='bt_examples',
        executable='battery_simulator',
        name='battery_simulator',
        output='screen',
        parameters=[{'drain_time': 15.0}],
    )

    return LaunchDescription([
        use_sim_time_arg,
        charger_x_arg,
        charger_y_arg,
        battery_sim_node,
        bumpandgo_node
    ])
