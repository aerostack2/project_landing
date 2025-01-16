#!/usr/bin/env python3

# Copyright 2024 Universidad Politécnica de Madrid
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Universidad Politécnica de Madrid nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Simple mission for a single drone."""

__authors__ = 'Rafael Perez-Segui'
__copyright__ = 'Copyright (c) 2024 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

import argparse
from time import sleep

from as2_python_api.drone_interface import DroneInterfaceBase
from as2_python_api.modules.follow_path_module import FollowPathModule
from as2_python_api.modules.go_to_module import GoToModule
from as2_python_api.modules.land_module import LandModule
from as2_python_api.modules.takeoff_module import TakeoffModule
from as2_python_api.modules.trajectory_generation_module import TrajectoryGenerationModule 
from as2_msgs.action import GeneratePolynomialTrajectory
from as2_msgs.msg import BehaviorStatus, PoseWithID, PoseStampedWithID, PoseStampedWithIDArray, YawMode

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import rclpy.time
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped 

TAKE_OFF_HEIGHT = 3.0  # Height in meters
TAKE_OFF_SPEED = 1.0  # Max speed in m/s
SLEEP_TIME = 0.5  # Sleep time between behaviors in seconds

FRAME_ID='vessel'
REF_FRAME_ID='earth'

UPDATE_VESSEL_POSITION_RATE = 30.0  # Rate in Hz
UPDATE_VESSEL_POSITION_THRESHOLD = 0.1  # Threshold in meters

# Aproach maneuver
AP_X = -1.5  # X position in meters relative to the reference
AP_Y = 0.0  # Y position in meters relative to the reference
AP_Z = 3.0  # Z position in meters relative to the reference
AP_SPEED = 2.0  # Max speed in m/s
AP_DISTANCE = 0.5  # Distance to goal in meters

# Landing maneuver
LD_X = -1.0  # X position in meters relative to the reference
LD_Y = 0.0  # Y position in meters relative to the reference
LD_Z = 0.0  # Z position in meters relative to the reference
LD_SPEED_XY = 1.0  # Max speed in m/s in xy plane
LD_SPEED_Z = 0.5  # Max speed in m/s in z


LAND_SPEED = 0.5  # Max speed in m/s


class DroneInterface(DroneInterfaceBase):
    def __init__(
            self, drone_id: str, use_sim_time: bool = True,
            verbose: bool = False):
        super().__init__(
            drone_id=drone_id,
            use_sim_time=use_sim_time,
            verbose=verbose,
            spin_rate=200.0)
        self.takeoff = TakeoffModule(drone=self)
        # self.go_to = GoToModule(drone=self)
        # self.follow_path = FollowPathModule(drone=self)
        self.land = LandModule(drone=self)
        self.trajectory_generation = TrajectoryGenerationModule(drone=self)

        # Create a tf
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            buffer=self.tf_buffer,
            node=self,
            spin_thread=True)

        # Modify publishers
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.modify_pub = self.create_publisher(
            PoseStampedWithIDArray,
            'motion_reference/modify_waypoint',
            qos_profile)
        
        self.last_vessel_position = None
        
    def get_vessel_position(self) -> TransformStamped:
        """
        Get the vessel position.
        """
        try:
            # Attempt to get the latest transform between the source and target frames
            current_time = self.get_clock().now()
            timeout = rclpy.time.Duration(seconds=0.1)
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                REF_FRAME_ID,  # Target frame
                FRAME_ID,  # Source frame
                current_time, # Time of the request
                timeout
            )
            return transform

        except Exception as ex:
            # Log a warning if the transformation is unavailable
            self.get_logger().warn(f'Could not get transform: {ex}')
            return None
        
    def publish_modify_waypoint(self):
        """
        Publish a modify waypoint message.
        """
        pose_with_id = PoseStampedWithID()
        pose_with_id.id = FRAME_ID
        pose_with_id.pose.header.frame_id = REF_FRAME_ID
        pose_with_id.pose.header.stamp = self.get_clock().now().to_msg()
        pose_with_id.pose.pose.position.x = self.last_vessel_position.transform.translation.x + self.x_offset
        pose_with_id.pose.pose.position.y = self.last_vessel_position.transform.translation.y + self.y_offset
        pose_with_id.pose.pose.position.z = self.last_vessel_position.transform.translation.z + self.z_offset

        pose_with_id_array = PoseStampedWithIDArray()
        pose_with_id_array.poses.append(pose_with_id)
        self.modify_pub.publish(pose_with_id_array)
    
    def update_vessel_position(self):
        """
        Update the vessel position.
        """
        if self.x_offset is None or self.y_offset is None or self.z_offset is None:
            return

        transform = self.get_vessel_position()
        if transform is None:
            return
        if self.last_vessel_position is None:
            self.last_vessel_position = transform
            self.publish_modify_waypoint()
            return
        
        # Check if the vessel has moved
        dx = transform.transform.translation.x - self.last_vessel_position.transform.translation.x
        dy = transform.transform.translation.y - self.last_vessel_position.transform.translation.y
        dz = transform.transform.translation.z - self.last_vessel_position.transform.translation.z
        distance = (dx**2 + dy**2 + dz**2)**0.5
        if distance > UPDATE_VESSEL_POSITION_THRESHOLD:
            self.last_vessel_position = transform
            self.publish_modify_waypoint()
    
    def send_trajectory_generation(self, x_offset, y_offset, z_offset, speed):
        """
        Send a trajectory generation behavior.
        """
        self.x_offset = x_offset
        self.y_offset = y_offset
        self.z_offset = z_offset

        approach_pose = PoseWithID()
        approach_pose.id = 'vessel'
        approach_pose.pose.position.x = x_offset
        approach_pose.pose.position.y = y_offset
        approach_pose.pose.position.z = z_offset

        yaw_mode = YawMode()
        yaw_mode.mode = YawMode.KEEP_YAW

        self.trajectory_generation(
            path=[approach_pose],
            speed=speed,
            yaw_mode=yaw_mode.mode,
            yaw_angle=0.0,
            frame_id=FRAME_ID,
            wait=False)


def drone_start(drone_interface: DroneInterface) -> bool:
    """
    Take off the drone.

    :param drone_interface: DroneInterface object
    :return: Bool indicating if the take off was successful
    """
    print('Start mission')

    # Arm
    print('Arm')
    success = drone_interface.arm()
    print(f'Arm success: {success}')

    # Offboard
    print('Offboard')
    success = drone_interface.offboard()
    print(f'Offboard success: {success}')

    # Take Off
    print('Take Off')
    success = drone_interface.takeoff(height=TAKE_OFF_HEIGHT, speed=TAKE_OFF_SPEED)
    print(f'Take Off success: {success}')

    return success


def drone_run(drone_interface: DroneInterface) -> bool:
    """
    Run the mission for a single drone.

    :param drone_interface: DroneInterface object
    :return: Bool indicating if the mission was successful
    """
    print('Run mission')

    # Approach
    drone_interface.send_trajectory_generation(AP_X, AP_Y, AP_Z, speed=AP_SPEED)
    sleep(SLEEP_TIME)
    while drone_interface.trajectory_generation.status == BehaviorStatus.RUNNING:
        drone_interface.update_vessel_position()
        sleep(0.1)

    # Land
    drone_interface.send_trajectory_generation(LD_X, LD_Y, LD_Z, speed=LD_SPEED_XY)
    sleep(SLEEP_TIME)
    while drone_interface.trajectory_generation.status == BehaviorStatus.RUNNING:
        drone_interface.update_vessel_position()
        sleep(0.1)

    return False

def drone_end(drone_interface: DroneInterface) -> bool:
    """
    End the mission for a single drone.

    :param drone_interface: DroneInterface object
    :return: Bool indicating if the land was successful
    """
    print('End mission')

    # # Land
    # print('Land')
    # success = drone_interface.land(speed=LAND_SPEED)
    # print(f'Land success: {success}')
    # if not success:
    #     return success
    print('Disarm')
    success = drone_interface.disarm()
    print(f'Disarm success: {success}')

    # Manual
    print('Manual')
    success = drone_interface.manual()
    print(f'Manual success: {success}')

    return success


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Single drone mission')

    parser.add_argument('-n', '--namespace',
                        type=str,
                        default='drone0',
                        help='ID of the drone to be used in the mission')
    parser.add_argument('-v', '--verbose',
                        action='store_true',
                        default=False,
                        help='Enable verbose output')
    parser.add_argument('-s', '--use_sim_time',
                        action='store_true',
                        default=True,
                        help='Use simulation time')

    args = parser.parse_args()
    drone_namespace = args.namespace
    verbosity = args.verbose
    use_sim_time = args.use_sim_time

    print(f'Running mission for drone {drone_namespace}')
    print(f'Verbose: {verbosity}')
    print(f'Use simulation time: {use_sim_time}')

    rclpy.init()

    uav = DroneInterface(
        drone_id=drone_namespace,
        use_sim_time=use_sim_time,
        verbose=verbosity)
    
    # Wait for user confirmation
    print('Start mission? y/n')
    if input() != 'y':
        uav.shutdown()
        rclpy.shutdown()
        print('Exit')
        exit(0)

    success = drone_start(uav)
    if success:
        success = drone_run(uav)
    success = drone_end(uav)

    uav.shutdown()
    rclpy.shutdown()
    print('Clean exit')
    exit(0)
