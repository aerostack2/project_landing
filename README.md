# project_landing
Project for the landing experiments on a moving platform that emulates the motion of a ship

## USAGE

Clone this repository and compile maritime simulation plugins by running

```
cd plugin
colcon build --merge-install
```

### Ship simulation

To launch the simulation with sea and buoyant ship run

```
./launch_as2.bash
```

### ACRO mode

To launch a test world with a drone controlled in ACRO mode run

```
./launch_as2.bash -c
```

