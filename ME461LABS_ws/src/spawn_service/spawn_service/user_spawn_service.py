import rclpy
import os 
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
from ament_index_python.packages import get_package_share_directory

class UserSpawnService(Node):
    def __init__(self):
        super().__init__('user_spawn_service')
        self.declare_parameter('x_position', 0.0)
        self.declare_parameter('y_position', 0.0)
        self.declare_parameter('z_position', 0.0)
        self.declare_parameter('object_name', 'Donut')
        self.model_name = self.get_parameter('object_name').value
        self.x_position = self.get_parameter('x_position').value
        self.y_position = self.get_parameter('y_position').value
        self.z_position = self.get_parameter('z_position').value
        self.spawn_client = self.create_client(SpawnEntity, 'spawn_entity')
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.get_logger().info('service is available')
        self.spawn_target()

    def spawn_target(self):
        pkg_path = os.path.join(get_package_share_directory('sokoban'))
        model_dir = os.path.join(pkg_path,'models')
        name = self.model_name
        if name == 'Donut':
            model_file = os.path.join(model_dir, 'Donut', 'model.sdf')
        elif name == 'Infinity':
            model_file = os.path.join(model_dir, 'Infinity', 'model.sdf')
        elif name == 'Kare':
            model_file = os.path.join(model_dir, 'Kare', 'model.sdf')
        elif name == 'b3gen':
            model_file = os.path.join(model_dir, 'b3gen', 'model.sdf')
        else:
            self.get_logger().info('Invalid model name')
            self.get_logger().info('Resorting to Default Model')
            model_file = os.path.join(model_dir, 'Donut', 'model.sdf')

        req = SpawnEntity.Request()
        with open(model_file, 'r') as file:
            req.xml = file.read()
        req.robot_namespace = name
        req.name = name
        req.initial_pose.position.x = self.x_position
        req.initial_pose.position.y = self.y_position
        req.initial_pose.position.z = self.z_position
        req.initial_pose.orientation.x = 0.0
        req.initial_pose.orientation.y = 0.0
        req.initial_pose.orientation.z = 0.0
        req.initial_pose.orientation.w = 1.0
        req.reference_frame = 'world'
        future = self.spawn_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(
                f'Spawned {name} at x:{self.x_position}, y:{self.y_position}, z:{self.z_position}')
        else:
            self.get_logger().info('Service call failed')

def main(args=None):
    rclpy.init(args=args)
    user_spawn_service = UserSpawnService()
    rclpy.spin(user_spawn_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()


