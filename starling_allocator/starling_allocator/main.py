import random

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

# from starling_allocator_msgs.msg import Allocation
from starling_allocator_msgs.srv import AllocateTrajectories

from simple_offboard_msgs.srv import SubmitTrajectory

from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectoryPoint
from trajectory_msgs.msg import JointTrajectory


class Allocator(Node):

    def __init__(self):
        super().__init__('starling_allocator')
        self.submit_trajectories_srv = self.create_service(AllocateTrajectories, 'submit_trajectories', self.submit_trajectories_cb)
        self.method_mapping = {
            'nearest': self.create_namespace_trajectory_mapping_nearest,
            'random': self.create_namespace_trajectory_mapping_random,
            'manual': self.create_namespace_trajectory_mapping_manual,
        }
        self.callback_group =
        self.get_logger().info("Initialised")

    def submit_trajectories_cb(self, req, res):
        self.get_logger().info("Trajectory Submit Request Received")

        if len(req.trajectories) == 0:
            res.success = False
            res.message = "No trajectories submitted"
            self.get_logger().warn(res.messge)
            return res

        if len(req.trajectory_types) == 0:
            res.success = False
            res.message = "Please specify 'trajectory_types' for each trajectory"
            self.get_logger().warn(res.messge)
            return res

        if len(req.trajectories) != len(req.trajectory_types):
            res.success = False
            res.message = "The number of trajectories do not match the number of trajectory_types given"
            self.get_logger().warn(res.messge)
            return res

        # Set Defaults
        if not req.interpolation_method:
           req.interpolation_method = 'cubic'
        if not req.auto_arm:
            req.auto_arm = True
        if not req.auto_takeoff:
            req.auto_takeoff = True
        if not req.method:
            req.method = "nearest"

        # Check if method is valid
        if req.method not in self.method_mapping:
            res.success = False
            res.message = "Method not recognised, currently supported methods include: " + ','.join(self.method_mapping.keys())
            self.get_logger().warn(res.messge)
            return res

        # generate trajectory tuple
        traj_tuple = [(i, t, ttyp) for i, (t, ttyp) in enumerate(zip(req.trajectories, req.trajectory_types))]

        # Generate mapping between vehicle and trajectory
        ns_traj_map = self.method_mapping[req.method](req, traj_tuple)

        try:
            # Call sending service
            self.call_trajectory_service(ns_traj_map, req)
        except Exception as e:
            res.success = False
            res.message = "Error occured when calling trajectory service: " + e
            self.get_logger().error(res.messge)
            return res

        res.success = True
        res.message = "Successfully allocated trajectories to available vehicles"
        res.allocation = []
        for name, (traj_idx, traj, _) in namespace_trajectory_mapping_dict.items():
            all = Allocation()
            all.vehicle = name
            all.trajectory_index = traj_idx
            all.trajectory = traj
            res.allocation.append(all)
        return res

    def create_namespace_trajectory_mapping_manual(self, req, traj_tuple):
        if not req.manual_allocation_targets:
            raise RuntimeError('Manual allocation method specified but no manual_allocation_targets given')

        if len(req.manual_allocation_targets) != len(trajectories):
            raise RuntimeError('The number of manual allocation targets does not match the number of trajectories')

        current_namespaces = self.__get_current_vehicle_namespaces()

        # Check if current namespaces are legitimate
        unrecognised_ns = [k for k in req.manual_allocation_targets if k not in current_namespaces]
        if len(unrecognised_ns) != 0:
            raise RuntimeError(f"The following namespaces are not currently active or available, will not proceed: {unrecognised_ns}. Current available namespaces: {current_namespaces}")

        return dict(zip(req.manual_allocation_targets, traj_tuple))

    def create_namespace_trajectory_mapping_nearest(self, req, traj_tuple):
        current_namespaces = self.__get_current_vehicle_namespaces()
        current_locations = {cn: None for cn in current_namespaces}
        current_namespace_pose_subs = [
            self.create_subscription(PoseStamped, f'{cn}/mavros/local_position/pose', lambda x: current_locations[cn] = x)
            for cn in current_namespaces
        ]

        while any([cv == None for cv in current_locations.values()]):
            self.get_logger().info(f"Waiting for pose information: {current_locations}")



    def create_namespace_trajectory_mapping_random(self, req, traj_tuple):
        current_namespaces = self.__get_current_vehicle_namespaces()
        random.shuffle(current_namespaces)
        random.shuffle(traj_tuple)
        return dict(zip(current_namespaces, traj_tuple))


    def call_trajectory_service(namespace_trajectory_mapping_dict, req):
        # parallelise in the future?
        for name, (_, traj, traj_type) in namespace_trajectory_mapping_dict.items():
            srv_name = f'{name}/submit_trajectory'
            cli = self.create_client(SubmitTrajectory, srv_name)
            sreq = SubmitTrajectory.Request()
            sreq.trajectory = traj
            sreq.type = traj_type
            sreq.interpolation_method = req.interpolation_method
            sreq.auto_arm = req.auto_arm
            sreq.auto_takeoff = req.auto_takeoff

            while not cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'service "{srv_name}" not available, waiting again...')
            cli.call_async(sreq)
            self.get_logger().info(f"Sent trajectory request to vehicle '{name}' via {srv_name}\n{sreq}")

        return True

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
