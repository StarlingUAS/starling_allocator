

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
        self.get_logger().info("Initialised")

    def submit_trajectories_cb(self, req, res):
        self.get_logger().info("Trajectory Submit Request Received")

        if len(req.trajectories) == 0:
            res.success = False
            res.message = "No trajectories submitted"
            self.get_logger().warn(res.messge)
            return res

        current_namespaces = self.__get_current_vehicle_namespaces()

        # Do something with allocation method, currently just allocate by order

        for name, traj in zip(current_namespaces, req.trajectories):
            srv_name = f'{name}/submit_trajectory'
            cli = self.create_client(SubmitTrajectory, srv_name)
            sreq = SubmitTrajectory.Request()
            sreq.trajectory = traj
            sreq.type = 'position'
            sreq.interpolation_method = 'cubic'
            sreq.auto_arm = True
            sreq.auto_takeoff = True

            while not cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'service "{srv_name}" not available, waiting again...')
            cli.call_async(sreq)
            self.get_logger().info(f"Sent trajectory request to vehicle '{name}' via {srv_name}\n{sreq}")

        return res


    def __get_current_vehicle_namespaces(self):
        topic_list = self.get_topic_names_and_types()
        namespaces = set()
        self.get_logger().info('Found the following topics:')
        for topic_name, _ in topic_list:
            self.get_logger().info(topic_name)
            if 'mavros' in topic_name:
                name = topic_name.split('/')[1]
                if name == 'mavros':
                    name = ''
                namespaces.add(name)
        self.get_logger().info(f'Found {len(namespaces)} namespaces: {",".join(namespaces)}')
        return namespaces



def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(Allocator())
    rclpy.shutdown()
