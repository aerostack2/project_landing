#!/bin/bash

gz topic -t /model/boat_beacon/joint/left_engine_propeller_joint/cmd_thrust -m gz.msgs.Double -p 'data: 40.00'

gz topic -t /model/boat_beacon/joint/right_engine_propeller_joint/cmd_thrust -m gz.msgs.Double -p 'data: 60.00'