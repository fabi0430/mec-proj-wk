from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ros_gz_bridge.actions import RosGzBridge

def generate_launch_description():
    # Nodo RosGzBridge con configuración YAML
    bridge_node = RosGzBridge(
        bridge_name="ros_gz_bridge",
        config_file="/home/oswaldo/Ros2WS/src/robot/robot/topic_bridge.yaml"  # Ruta al YAML
    )

    # Nodo cinemática
    cinematica_node = Node(
        package='robot',
        executable='cinematica',
        name='cinematica',
        output='screen'
    )

    # Nodo gazebo
    datos_simG_node = Node(
        package='robot',
        executable='datos_simG',
        name='datos_simG',
        output='screen'
    )

    execute_process_node = ExecuteProcess(
        cmd=['gnome-terminal', '--', 'ros2', 'run', 'robot', 'manipuladorTeclado'],
        output='screen'
    )

    gazebo_node = ExecuteProcess(
        cmd=['gnome-terminal', '--', 'bash', '-c', 'cd ~/Ros2WS/src/robot/robot && gz sim building_robot.sdf'],
        output='screen'
    )

    # Crear la LaunchDescription con ambos nodos
    ld = LaunchDescription()
    ld.add_action(bridge_node)
    ld.add_action(cinematica_node)
    ld.add_action(datos_simG_node)
    ld.add_action(gazebo_node)
    ld.add_action(execute_process_node)

    return ld

