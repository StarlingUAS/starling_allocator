

import rclpy
from rclpy.node import Node

# from starling_allocator_msgs.msg import Allocation
from starling_allocator_msgs.srv import AllocateTrajectories

from simple_offboard_msgs.srv import SubmitTrajectory

from trajectory_msgs.msg import JointTrajectoryPoint
from trajectory_msgs.msg import JointTrajectory


class Allocator(Node):

    def __init__(self):
        super().__init__('starling_allocator')
        self.submit_trajectories_srv = self.create_service(AllocateTrajectories, 'submit_trajectories', self.submit_trajectories_cb)

    def submit_trajectories_cb(self, req, res):
        self.get_logger().info("Trajectory Submit Request Received")

        if len(req.trajectories) == 0:
            res.success = False
            res.message = "No trajectories submitted"
            self.get_logger().warn(res.messge)
            return res

        current_namespaces = self.__get_current_vehicle_namespaces()

        # Do something with allocation method, currently just allocate by order

        for name, traj in zip(current_namespace, req.trajectories):
            cli = self.create_client(SubmitTrajectory, f'{name}/submit_trajectory')
            sreq = SubmitTrajectory.Request()
            sreq.trajectory = traj
            sreq.type = 'position'
            sreq.auto_arm = True

            while not cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('service not available, waiting again...')
            cli.call_async(sreq)
            self.get_logger().info(f"Sent trajectory request to vehicle {name}")


    def __get_current_vehicle_namespaces(self):
        topic_list = self.get_topic_names_and_types()
        namespaces = set()
        self.get_logger().info('Found the following topics:')
        for topic_name, _ in topic_list:
            self.get_logger().info(topic_name)
            if 'mavros' in topic_name:
                namespaces.add(topic_name.split('/')[1])
        self.get_logger().info(f'Found {len(namespaces)} namespaces: {",".join(namespaces)}')
        return namespaces



def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(Allocator())
    rclpy.shutdown()
