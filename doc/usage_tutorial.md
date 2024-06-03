# How to use

## Configuration

To configurure parameters of the simulator:
- [perception.yaml](../config/perception.yaml) - configuration of cone models
- [sensors.yaml](../config/sensors.yaml) - configuration of imu, wheel encoders and other sensors
- [vehicleModel.yaml](../config/vehicleModel.yaml) - configuration of vehicle model
- [mainConfig.yaml](../config/mainConfig.yaml) - configuration of event timeouts and other competition features

## Running the simulator

The simulator is inside a docker container, which by running it, the simulator will also be launched. To do this, run the following command on your terminal:

```bash
docker compose up
```

In case you want to do some work inside the container, by using the Dev Containers extension in VSCode the process becomes very simple, since the system is already setup for that.

Inside the devcontainer, in order to run the simulator, simply use one of the launch files present in the [launch folder](../launch/):

```sh
colcon build # if you haven't compiled yet
source ./install/setup.bash # if you haven't already
ros2 launch pacsim example.launch.py
```

## Visualization

For visualization, either Rviz or Foxglove can be used, the later needing to run the foxglove bridge in order to work.

## Track

The simulator already has comes with some tracks, used in different events of the competitions (acceleration, skidpad...).

### Track generator

Checkout the track generator in the [track editor folder](../track_editor/).


### Other tools

[Scrips folder](../scripts/) contains a tool for track convertion from fssim format to this one, as well as a report evaluation tool.