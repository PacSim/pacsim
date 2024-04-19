# How to use

## Configuration

To configurure parameters of the simulator:
- [perception.yaml](../config/perception.yaml) - configuration of cone models
- [sensors.yaml](../config/sensors.yaml) - configuration of imu, wheel encoders and other sensors
- [vehicleModel.yaml](../config/vehicleModel.yaml) - configuration of vehicle model
- [mainConfig.yaml](../config/mainConfig.yaml) - configuration of event timeouts and other competition features

## Running the simulator

To run the simulator, simply use one of the launch files present in the [launch folder](../launch/):

```sh
colcon build # if you haven't compiled yet
source ./install/setup.bash # if you haven't already
ros2 launch pacsim example.launch.py
```

## Visualization

Use Foxglove (or Rviz) for visualization purposes. TODO: create Foxglove dashboard for pacsim

## Track Generation

Checkout the track generator in the [track editor folder](../track_editor/).

## Other tools

[Scrips folder](../scripts/) contains a tool for track convertion from fssim format to this one, as well as a report evaluation tool.