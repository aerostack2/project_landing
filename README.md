# project_landing
Project for the landing experiments on a moving platform that emulates the motion of a ship

## USAGE

Clone this repository and compile maritime simulation plugins by running

```
cd plugin_ws
colcon build --merge-install
```

### Ship simulation

To launch the simulation with sea and buoyant ship run

```
./launch_as2.bash
```

To move the ship use the next command

```
gz topic -t /model/vessel/joint/engine_propeller_joint/cmd_thrust -m gz.msgs.Double -p 'data: 1000.00'
```

### ACRO mode

To launch a test world with a drone controlled in ACRO mode run

```
./launch_as2.bash -c
```



