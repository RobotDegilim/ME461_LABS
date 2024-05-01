'''
This script is used to spawn targets in the gazebo environment. The targets are spawned at specific positions in the environment.
THe script read the Json file of the targets and spawns the targets at the specified positions in the world.
'''
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
from .definition import Target 
from ament_index_python.packages import get_package_share_directory
import os
import json

class SpawnService(Node):
    def __init__(self):
        super().__init__('spawn_service')
        self.spawn_client = self.create_client(SpawnEntity, 'spawn_entity')
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.get_logger().info('Service is available')
        self.futures = []
        self.load_and_spawn_targets()

    def handle_spawn_response(self, future, name):
        if future.result() is not None:
            self.get_logger().info(f'Spawned {name} target successfully')
        else:
            self.get_logger().info(f'Failed to spawn {name} target')

    def spawn_targets(self, model_file, name, x, y, z):
        unique_id = f"{name}_{x}_{y}_{z}"
        req = SpawnEntity.Request()
        try:
            with open(model_file, 'r') as file:
                req.xml = file.read()
        except IOError as e:
            self.get_logger().error(f"Failed to read model file {model_file}: {str(e)}")
            return
        req.robot_namespace = name
        req.name = unique_id
        req.initial_pose.position.x = x
        req.initial_pose.position.y = y
        req.initial_pose.position.z = z
        req.initial_pose.orientation.x = 0.0
        req.initial_pose.orientation.y = 0.0
        req.initial_pose.orientation.z = 0.0
        req.initial_pose.orientation.w = 1.0
        req.reference_frame = 'world'
        future = self.spawn_client.call_async(req)
        future.add_done_callback(lambda future: self.handle_spawn_response(future, name))
        self.futures.append(future)
        self.get_logger().info(f'Spawning {unique_id} target...')

    def load_and_spawn_targets(self):
        json_file_path = os.path.join(get_package_share_directory('spawn_service'), 'params', 'targets.json')
        with open(json_file_path, 'r') as file:
            targets_data = json.load(file)
        
        for target_info in targets_data['targets']:
            target = Target(target_info['name'], target_info['x'], target_info['y'], target_info['z'])
            model_file = target.get_model_file()
            if model_file:
                self.spawn_targets(model_file, target.name, target.x, target.y, target.z)

def main(args=None):
    rclpy.init(args=args)
    spawn_service = SpawnService()
    rclpy.spin(spawn_service)
    spawn_service.get_logger().info('All requests sent, waiting for responses...')
    rclpy.shutdown()

if __name__ == '__main__':
    main()
