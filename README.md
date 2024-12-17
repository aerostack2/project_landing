# project_landing
Project for the landing experiments on a moving platform that emulates the motion of a ship

## USAGE

Clone this repository and compile maritime simulation plugins by running

```
cd gz_resources/plugin_ws
colcon build --merge-install
```

### Follow reference landing

Launch aerostack2 onboard nodes

```
./launch_as2_follow_reference.bash
```

Launch ground station

```
./launch_ground_station.bash
```

Confirm start mission in the ground station

### Trajectory generation landing

Launch aerostack2 onboard nodes

```
./launch_as2.bash
```

Launch ground station

```
./launch_ground_station.bash
```

Confirm start mission in the ground station
