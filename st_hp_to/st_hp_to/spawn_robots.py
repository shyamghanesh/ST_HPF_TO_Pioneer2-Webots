import rclpy
from rclpy.node import Node
from gz_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose

class RobotSpawner(Node):
    def __init__(self):
        super().__init__('robot_spawner')
        self.client = self.create_client(SpawnEntity, '/world/factory_floor/spawn_entity')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Spawn service not available, waiting...')
        self.spawn_robots()

    def spawn_robots(self):
        positions = [
            (1.0, -3.3, 0.2),
            (1.0, 4.7, 0.2),
            (5.0, -3.3, 0.2)
        ]
        for i, (x, y, z) in enumerate(positions, 1):
            request = SpawnEntity.Request()
            request.name = f'robot{i}'
            request.xml = open('/home/shyam/ros2_ws/install/turtlebot3_description/share/turtlebot3_description/urdf/turtlebot3_burger.urdf', 'r').read()
            request.initial_pose = Pose()
            request.initial_pose.position.x = x
            request.initial_pose.position.y = y
            request.initial_pose.position.z = z
            self.client.call_async(request)
            self.get_logger().info(f'Spawned robot{i} at ({x}, {y}, {z})')

def main():
    rclpy.init()
    node = RobotSpawner()
    rclpy.spin_once(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
