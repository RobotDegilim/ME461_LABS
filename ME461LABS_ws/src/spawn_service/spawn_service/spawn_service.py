'''
This script is used to spawn boxes in the gazebo world. It uses the SpawnEntity service to spawn the boxes in the world.
The script reads the xacro file and spawns the boxes in random positions in the world.
'''
import rclpy
import os
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
from ament_index_python.packages import get_package_share_directory
import xacro
import random

class SpawnService(Node):
    def __init__(self):
        super().__init__('spawn_service')
        self.declare_parameter('object_name', 'box')
        self.declare_parameter('number_of_boxes', 10)
        object_name = self.get_parameter('object_name').value
        number_of_boxes = self.get_parameter('number_of_boxes').value

        self.spawn_client = self.create_client(SpawnEntity, 'spawn_entity')
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.get_logger().info('service is available')
        self.spawn(object_name, number_of_boxes)


    def spawn(self, name, number):
        pkg_path = os.path.join(get_package_share_directory('sokoban'))
        xacro_file = os.path.join(pkg_path,'urdf','box_spawn.urdf.xacro')
        robot_description_config = xacro.process_file(xacro_file)

        futures = []
        for i in range(number):
            req = SpawnEntity.Request()
            req.xml = robot_description_config.toxml()
            req.robot_namespace = name
            req.name = name + str(i)
            req.initial_pose.position.x = random.uniform(-2, 2)
            req.initial_pose.position.y = random.uniform(-2, 2)
            req.initial_pose.position.z = 0.0
            req.initial_pose.orientation.x = 0.0
            req.initial_pose.orientation.y = 0.0
            req.initial_pose.orientation.z = 0.0
            req.initial_pose.orientation.w = 1.0
            req.reference_frame = 'world'
            future = self.spawn_client.call_async(req)
            futures.append((future, req.name))

        for future, req_name in futures:
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                self.get_logger().info(f'Successfully spawned {req_name}')
            else:
                self.get_logger().info(f'Failed to spawn {req_name}')
        

def main(args=None):
    rclpy.init(args=args)
    spawn_service = SpawnService()
    rclpy.spin(spawn_service)
    spawn_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
        
