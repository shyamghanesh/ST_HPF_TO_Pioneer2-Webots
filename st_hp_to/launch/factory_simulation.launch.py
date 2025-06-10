from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    # Path to the world file
    world_file = "/home/shyam/ros2_ws/src/st_hp_to/worlds/factory_floor.world"

    # Start Gazebo Ionic with the world
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-v', '4', '-r', world_file],
        output='screen'
    )

    # Spawn TurtleBot3 robots
    robot_positions = [
        (1.0, -3.3, 0.2),
        (1.0, 4.7, 0.2),
        (5.0, -3.3, 0.2)
    ]
    spawn_nodes = []
    for i, (x, y, z) in enumerate(robot_positions, 1):
        spawn_node = Node(
            package='gz_sim',
            executable='spawn',
            arguments=[
                '-entity', f'robot{i}',
                '-file', '/home/shyam/ros2_ws/install/turtlebot3_description/share/turtlebot3_description/urdf/turtlebot3_burger.urdf',
                '-x', str(x),
                '-y', str(y),
                '-z', str(z)
            ],
            output='screen'
        )
        spawn_nodes.append(spawn_node)

    return LaunchDescription([gazebo] + spawn_nodes)
