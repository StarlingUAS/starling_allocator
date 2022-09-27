import random
import time
import numpy as np
from functools import partial

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import qos_profile_system_default

from starling_allocator_msgs.msg import Allocation, Allocations
from starling_allocator_msgs.srv import AllocateTrajectories

from simple_offboard_msgs.srv import SubmitTrajectory

from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectoryPoint
from trajectory_msgs.msg import JointTrajectory


class Allocator(Node):

    def __init__(self):
        super().__init__('starling_allocator')
        self.callback_group = ReentrantCallbackGroup()
        # self.srv_callback_group = ReentrantCallbackGroup()
        self.submit_trajectories_srv = self.create_service(AllocateTrajectories, 'submit_trajectories', self.submit_trajectories_cb)
        self.method_mapping = {
            'nearest': self.create_namespace_trajectory_mapping_nearest,
            'random': self.create_namespace_trajectory_mapping_random,
            'manual': self.create_namespace_trajectory_mapping_manual,
        }
        self.current_locations = {}


        self.allocation_pub = self.create_publisher(Allocations, 'current_allocated_trajectory', 10)
        self.current_allocation = Allocations()
        self.create_timer(5, self.publish_allocation)

        self.get_logger().info("Initialised")
    def publish_allocation(self):
        self.allocation_pub.publish(self.current_allocation)

    def submit_trajectories_cb(self, req, res):
        self.get_logger().info("Trajectory Submit Request Received")

        if len(req.trajectories) == 0:
            res.success = False
            res.message = "No trajectories submitted"
            self.get_logger().warn(res.message)
            return res

        if not req.trajectory_types or len(req.trajectory_types) == 0:
            res.success = False
            res.message = "Please specify 'trajectory_types' for each trajectory"
            self.get_logger().warn(res.message)
            return res

        if len(req.trajectories) != len(req.trajectory_types):
            res.success = False
            res.message = "The number of trajectories do not match the number of trajectory_types given"
            self.get_logger().warn(res.message)
            return res

        # Set Defaults
        if not req.interpolation_method:
           req.interpolation_method = 'linear'
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
            self.get_logger().warn(res.message)
            return res

        # generate trajectory tuple
        traj_tuple = [(i, t, ttyp) for i, (t, ttyp) in enumerate(zip(req.trajectories, req.trajectory_types))]

        # Generate mapping between vehicle and trajectory
        self.get_logger().info(f"Generating trajectory map of type {req.method}")
        ns_traj_map = self.method_mapping[req.method](req, traj_tuple)
        v = {k: kid for k, (kid, _, _) in ns_traj_map.items()}
        self.get_logger().info(f"Generated trajectory map: {v}")

        try:
            # Call sending service
            self.call_trajectory_service(ns_traj_map, req)
        except Exception as e:
            res.success = False
            res.message = "Error occured when calling trajectory service: " + str(e)
            self.get_logger().error(res.message)
            return res

        res.success = True
        res.message = "Successfully allocated trajectories to available vehicles"
        res.allocation = []
        for name, (traj_idx, traj, _) in ns_traj_map.items():
            all = Allocation()
            all.vehicle = name
            all.trajectory_index = traj_idx
            all.trajectory = traj
            res.allocation.append(all)
        # self.get_logger().info(f"Final Allocation from request is {res.allocation}")
        self.current_allocation.allocation = res.allocation
        return res

    def create_namespace_trajectory_mapping_manual(self, req, traj_tuple):
        if not req.manual_allocation_targets:
            raise RuntimeError('Manual allocation method specified but no manual_allocation_targets given')

        if len(req.manual_allocation_targets) != len(req.trajectories):
            raise RuntimeError('The number of manual allocation targets does not match the number of trajectories')

        current_namespaces = self.__get_current_vehicle_namespaces()

        # Check if current namespaces are legitimate
        unrecognised_ns = [k for k in req.manual_allocation_targets if k not in current_namespaces]
        if len(unrecognised_ns) != 0:
            raise RuntimeError(f"The following namespaces are not currently active or available, will not proceed: {unrecognised_ns}. Current available namespaces: {current_namespaces}")

        return dict(zip(req.manual_allocation_targets, traj_tuple))

    def __assign_current_loc(self, cn, msg):
        # self.get_logger().info(f'Assigning {cn} to {msg}')
        self.current_locations[cn] = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]

    def create_namespace_trajectory_mapping_nearest(self, req, traj_tuple):
        current_namespaces = self.__get_current_vehicle_namespaces()
        self.current_locations = {cn: None for cn in current_namespaces}

        if len(traj_tuple[0][1].points[0].positions) == 0:
            self.get_logger().info("No positions found in trajectory, cannot assign by nearest, assigning by order")
            return dict(zip(current_namespaces, traj_tuple))

        current_namespace_pose_subs = [
            self.create_subscription(PoseStamped, f'{cn}/mavros/local_position/pose',
                                     callback=partial(self.__assign_current_loc, cn),
                                     qos_profile=10, callback_group=self.callback_group)
            for cn in current_namespaces]

        self.get_logger().info(f"Polling for 2 seconds")
        rate = self.create_rate(0.5)
        rate.sleep()
        rate.destroy()

        self.current_locations = {cn: cv for cn, cv in self.current_locations.items() if cv != None}
        self.get_logger().info(f"Got local positions: {self.current_locations}")

        # Close the subscribers once current locations have been accessed
        for sub in current_namespace_pose_subs:
            sub.destroy()

        traj_locations = {k: t.points[0].positions[:3] for (k, t, _) in traj_tuple}

        assigned = {}
        assigned_traj_idx = set()
        for cn, cl in self.current_locations.items():
            self.get_logger().info(f"Assigning {cn}")
            cassin = None
            min_dist = 100000000000
            for k, position in traj_locations.items():
                if k in assigned_traj_idx:
                    self.get_logger().info(f"{k} already assigned")
                    continue
                dist = np.linalg.norm(np.array(cl) - np.array(position))
                self.get_logger().info(f"Checking traj {k} at dist {dist} from vehicle position {cl} to init traj {position}")
                if dist < min_dist:
                    min_dist = dist
                    cassin = k
                    self.get_logger().info(f"Maybe Assinging traj {k} to {cn}")
            self.get_logger().info(f"Assigned vehicle {cn} to traj {cassin}")
            assigned[cn] = traj_tuple[cassin]
            assigned_traj_idx.add(cassin)

            blah = {cn: ttp[0] for cn, ttp in assigned.items()}
            self.get_logger().info(f"Assigned: {blah}")

        return assigned

    def create_namespace_trajectory_mapping_random(self, req, traj_tuple):
        current_namespaces = self.__get_current_vehicle_namespaces()
        random.shuffle(current_namespaces)
        random.shuffle(traj_tuple)
        return dict(zip(current_namespaces, traj_tuple))

    def call_trajectory_service(self, namespace_trajectory_mapping_dict, req):
        # parallelise in the future?
        for name, (_, traj, traj_type) in namespace_trajectory_mapping_dict.items():
            srv_name = f'{name}/submit_trajectory'
            cli = self.create_client(SubmitTrajectory, srv_name)
            sreq = SubmitTrajectory.Request()
            sreq.trajectory = traj
            sreq.type = 'position' if traj_type in ['position', 'velocity'] else 'attitude'
            sreq.interpolation_method = req.interpolation_method
            sreq.auto_arm = req.auto_arm
            sreq.auto_takeoff = req.auto_takeoff
            sreq.frame_id = 'map' if traj_type == 'position' else f'{name}/body'

            if not cli.wait_for_service(timeout_sec=10.0):
                self.get_logger().info(f'Service "{srv_name}" timed out not available. Request to vehicle {name} failed.')
                continue

            cli.call_async(sreq)
            self.get_logger().info(f"Sent trajectory request to vehicle '{name}' via {srv_name}")
            self.get_logger().debug(f"Req: {sreq}")

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
    try:
        alloc = Allocator()
        executor = MultiThreadedExecutor()
        executor.add_node(alloc)
        try:
            executor.spin()
        finally:
            executor.shutdown()
            alloc.destroy_node()
    finally:
        rclpy.shutdown()
