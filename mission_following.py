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
from as2_python_api.modules.follow_reference_module import FollowReferenceModule
from as2_msgs.action import FollowReference
from as2_msgs.msg import BehaviorStatus
import rclpy

TAKE_OFF_HEIGHT = 3.0  # Height in meters
TAKE_OFF_SPEED = 1.0  # Max speed in m/s
SLEEP_TIME = 0.5  # Sleep time between behaviors in seconds

FRAME_ID='vessel'

# Aproach maneuver
AP_X = -1.5  # X position in meters relative to the reference
AP_Y = 0.0  # Y position in meters relative to the reference
AP_Z = 3.0  # Z position in meters relative to the reference
AP_SPEED_XY = 2.0  # Max speed in m/s in xy plane
AP_SPEED_Z = 2.0  # Max speed in m/s in z
AP_DISTANCE = 0.5  # Distance to goal in meters

# Landing maneuver
LD_X = -1.0  # X position in meters relative to the reference
LD_Y = 0.0  # Y position in meters relative to the reference
LD_Z = 0.0  # Z position in meters relative to the reference
LD_SPEED_XY = 1.0  # Max speed in m/s in xy plane
LD_SPEED_Z = 0.5  # Max speed in m/s in z
LD_DISTANCE = 0.9  # Distance to goal in meters


LAND_SPEED = 0.5  # Max speed in m/s


class DroneInterface(DroneInterfaceBase):
    def __init__(self, drone_id: str, use_sim_time: bool = False, verbose: bool = False):
        super().__init__(
            drone_id=drone_id,
            use_sim_time=use_sim_time,
            verbose=verbose,
            spin_rate=200.0)
        self.takeoff = TakeoffModule(drone=self)
        # self.go_to = GoToModule(drone=self)
        # self.follow_path = FollowPathModule(drone=self)
        self.land = LandModule(drone=self)
        self.follow_reference = FollowReferenceModule(drone=self)


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

    # Follow reference
    # drone_interface.follow_reference_with_reference_facing(
    drone_interface.follow_reference.follow_reference(
        x=AP_X, y=AP_Y, z=AP_Z, frame_id=FRAME_ID,
        speed_x=AP_SPEED_XY, speed_y=AP_SPEED_XY, speed_z=AP_SPEED_Z)
    sleep(SLEEP_TIME)

    # Wait for the drone to reach the goal
    while drone_interface.follow_reference.status == BehaviorStatus.RUNNING:
        fb: FollowReference.Feedback = drone_interface.follow_reference.feedback
        print(f'Feedback: {fb}')
        print(f'Actual distance to goal: {fb.actual_distance_to_goal}')
        if fb.actual_distance_to_goal < AP_DISTANCE:
            break

    # Land
    drone_interface.follow_reference.follow_reference(
        x=LD_X, y=LD_Y, z=LD_Z, frame_id=FRAME_ID,
        speed_x=LD_SPEED_XY, speed_y=LD_SPEED_XY, speed_z=LD_SPEED_Z)
    sleep(SLEEP_TIME)

    # Wait for the drone to reach the goal
    while drone_interface.follow_reference.status == BehaviorStatus.RUNNING:
        fb: FollowReference.Feedback = drone_interface.follow_reference.feedback
        print(f'Actual distance to goal: {fb.actual_distance_to_goal}')
        if fb.actual_distance_to_goal < LD_DISTANCE:
            return True
    return False

def drone_end(drone_interface: DroneInterface) -> bool:
    """
    End the mission for a single drone.

    :param drone_interface: DroneInterface object
    :return: Bool indicating if the land was successful
    """
    print('End mission')

    # Land
    print('Land')
    success = drone_interface.land(speed=LAND_SPEED)
    print(f'Land success: {success}')
    if not success:
        return success

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
