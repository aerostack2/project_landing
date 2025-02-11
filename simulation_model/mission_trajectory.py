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
from typing import List

from as2_python_api.drone_interface import DroneInterface
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path


# Takeoff params
TAKE_OFF_HEIGHT = 1.5  # Height in meters
TAKE_OFF_SPEED = 1.0  # Max speed in m/s
SLEEP_TIME = 0.5  # Sleep time between behaviors in seconds

# Fly params
FRAME_ID = 'earth'  # Frame ID for the path
SPEED = 1.5  # Max speed in m/s
HEIGHT = 1.5  # Height in meters
INGORE_YAW = True  # If true, keep the current yaw
VARY_HEIGHT = False # If true, follows path with varying height

NUM_LOOPS = 0  # Number of times to loop the path
DIM = 1.0
PATH = [
    [DIM, DIM, HEIGHT],
    [DIM, -DIM, HEIGHT],
    [-DIM, DIM, HEIGHT],
    [-DIM, -DIM, HEIGHT],
    [0.0, 0.0, HEIGHT],
]

PATH_HEIGHT = [
    [0.0, DIM, HEIGHT-0.5],
    [0.0, DIM, HEIGHT+0.5],
    [0.0, -DIM, HEIGHT-0.5],
    [0.0, -DIM, HEIGHT+0.5],
    [0.0, 0.0, HEIGHT],
]

PATH_HEIGHT_YAW = [
    [DIM, DIM, 1.5*HEIGHT],
    [DIM, -DIM, HEIGHT],
    [-DIM, DIM, 0.75*HEIGHT],
    [-DIM, -DIM, 1.25*HEIGHT],
    [0.0, 0.0, HEIGHT],
]

# Land params
LAND_SPEED = 0.5  # Max speed in m/s


def confirm(msg: str = 'Continue') -> bool:
    """ Ask for confirmation """
    confirmation = input(f'{msg}? (y/n): ')
    if confirmation == 'y':
        return True
    return False


def clean_exit(drone_interface: DroneInterface):
    """
    Clean exit for the mission.

    :param drone_interface: DroneInterface object
    """
    print('Clean exit')
    drone_interface.shutdown()
    rclpy.shutdown()
    exit(0)


def convert_list_to_path(path_list: List) -> Path:
    """ Convert a list of points to a Path message """
    path = Path()
    path.header.frame_id = 'earth'

    # Variable to keep track of the last added point
    last_point = None
    
    for point in path_list:
        # Skip the point if it's the same as the last added one
        if last_point is not None and point == last_point:
            continue
        
        # Create and add a new PoseStamped
        pose = PoseStamped()
        pose.pose.position.x = point[0]
        pose.pose.position.y = point[1]
        pose.pose.position.z = point[2]
        pose.pose.orientation.w = 1.0
        path.poses.append(pose)
        
        # Update last_point to the current point
        last_point = point
    
    return path


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
    sleep(SLEEP_TIME)

    # Offboard
    print('Offboard')
    success = drone_interface.offboard()
    print(f'Offboard success: {success}')
    sleep(SLEEP_TIME)

    # Take Off
    print('Take Off')
    success = drone_interface.takeoff(height=TAKE_OFF_HEIGHT, speed=TAKE_OFF_SPEED)
    print(f'Take Off success: {success}')
    sleep(SLEEP_TIME)

    return success


def drone_run(drone_interface: DroneInterface, path: Path) -> bool:
    """
    Run the mission for a single drone.

    :param drone_interface: DroneInterface object
    :return: Bool indicating if the mission was successful
    """
    print('Run mission')
    if INGORE_YAW:
        print('Following path with keep yaw')
        success = drone_interface.follow_path.follow_path_with_keep_yaw(
            path, speed=SPEED, frame_id=FRAME_ID)
    else:
        print('Following path with path facing')
        success = drone_interface.follow_path.follow_path_with_path_facing(
            path, speed=SPEED, frame_id=FRAME_ID)
    print(f'Path success: {success}')

    return success

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
    sleep(SLEEP_TIME)
    
    # print('Disarm')
    # success = drone_interface.disarm()
    # print(f'Disarm success: {success}')
    # sleep(SLEEP_TIME)
    
    # # Manual
    # print('Manual')
    # success = drone_interface.manual()
    # print(f'Manual success: {success}')
    # sleep(SLEEP_TIME)
    
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
                        default=False,
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
    
    # Take off
    if confirm('Take off? y/n'):
        success = drone_start(uav)
        if not success:
            if confirm('Land? y/n'):
                success = drone_end(uav)
            if confirm('Clean exit? y/n'):
                clean_exit(uav)

    path_squared = PATH

    if VARY_HEIGHT:
        if INGORE_YAW:
            path_squared = PATH_HEIGHT
        else:
            path_squared = PATH_HEIGHT_YAW

    for i in range(NUM_LOOPS):
        print('Adding loop')
        path_squared += path_squared
    path = convert_list_to_path(path_squared)
    
    # Fly
    while confirm('Do a run? y/n'): 
        success = drone_run(uav, path=path)
    
    # Land
    print('Land? y/n')
    if input() == 'y':
        success = drone_end(uav)

    uav.shutdown()
    rclpy.shutdown()
    print('Clean exit')
    exit(0)
