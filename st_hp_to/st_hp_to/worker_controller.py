import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from gz_msgs.srv import SetEntityState  # Updated import

class WorkerController(Node):
    def __init__(self):
        super().__init__('worker_controller')
        self.client = self.create_client(SetEntityState, '/world/factory_floor/set_entity_state')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Gazebo service not available, waiting...')
        self.timer = self.create_timer(0.032, self.move_worker)
        self.pos_x = 5.0
        self.pos_y = 0.7
        self.vel_x = 0.1
        self.vel_y = 0.1
        self.x_min, self.x_max = -10.0, 10.0
        self.y_min, self.y_max = -12.5, 3.9

    def move_worker(self):
        # Update position
        self.pos_x += self.vel_x * 0.032
        self.pos_y += self.vel_y * 0.032

        # Boundary checks
        if self.pos_x <= self.x_min or self.pos_x >= self.x_max:
            self.vel_x = -self.vel_x
            self.pos_x = max(self.x_min, min(self.x_max, self.pos_x))
        if self.pos_y <= self.y_min or self.pos_y >= self.y_max:
            self.vel_y = -self.vel_y
            self.pos_y = max(self.y_min, min(self.y_max, self.pos_y))

        # Set new position
        request = SetEntityState.Request()
        request.entity.name = 'worker'
        request.state.pose = Pose()
        request.state.pose.position.x = self.pos_x
        request.state.pose.position.y = self.pos_y
        request.state.pose.position.z = 0.25
        self.client.call_async(request)

def main():
    rclpy.init()
    node = WorkerController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
