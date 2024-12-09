# Getting started with PacSim

To use PacSim, you need to create a message converter node to match your own interfaces with the simulator. You will need to advertise and publish to some topics. Note that some topics are not directly used in the simulator.

| TOPIC                        | MESSAGE TYPE               | DESCRIPTION                                                                           | USED?  |
|------------------------------|----------------------------|---------------------------------------------------------------------------------------|---|
| /pacsim/steering_setpoint    | pacsim::msg::StampedScalar | Target steering angle at the steering wheel/sensor (rad)                              | YES |
| /pacsim/torques_max          | pacsim::msg::Wheels        | Upper bound torque value at each wheel's motor (Nm)                                   | YES |
| /pacsim/powerground_setpoint | pacsim::msg::StampedScalar | Powered ground multiplier. 0 for no powered aero, 1 for full capacity (Dimensionless) | YES |
| /pacsim/torques_min          | pacsim::msg::Wheels        | Lower bound torque value at each wheel's motor (Nm)                                   | NO |
| /pacsim/wheelspeed_setpoints | pacsim::msg::Wheels        | Target wheel speeds at each wheel's motor (RPM)                                       | NO |

This simulator does not model the interface of a motor controller. It is highly recommended you implement your own internal control loop that controls the amount of torque applied to each wheel to reach a given target wheel speed. Moreover, `torques_max` can be seen more as the "current torque request sent", rather than the upper bound torque value at each wheel's motor.

The example launch file (example.launch.py) shows an example of how to start the simulator node and the robot_state_publisher for the 3D visualisation of the car.

The sensors and vehicle model are configured using config files. Examples are provided in the config folder. Things such as the discipline or the path of the track file or config files are defined using ROS2 parameters.

Future improvements that you may like to consider include, but are not limited to:
- Implement a more accurate vehicle model in the future via the `IVehicleModel` class.
- Improve sensor modelling (e.g. realistic false positive and false negative cone detections)
- Generate ground truth reference trajectory from track YAML files
- External clock control (ability to pause/play at will for debugging purposes)

### Do you have any ideas? Please feel free to contribute by creating an issue on the repository! 
