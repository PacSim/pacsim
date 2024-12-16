# Getting started with PacSim

## Interfaces 
To use PacSim, you need to create a message converter node to match your own interfaces with the simulator. You will need to advertise and publish to some topics. Note that some topics are not directly used in the simulator with the default vehicle model provided.

### Inputs

| TOPIC                        | MESSAGE TYPE               | DESCRIPTION                                                                           | USED WITH DEFAULT MODEL?  |
|------------------------------|----------------------------|---------------------------------------------------------------------------------------|---|
| /pacsim/steering_setpoint    | pacsim::msg::StampedScalar | Target steering angle at the steering wheel/sensor (rad)                              | YES |
| /pacsim/powerground_setpoint | pacsim::msg::StampedScalar | Powered ground multiplier. 0 for no powered aero, 1 for full capacity (Dimensionless) | YES |
| /pacsim/wheelspeed_setpoints | pacsim::msg::Wheels        | Target wheel speeds at each wheel's motor (RPM)                                       | NO |
| /pacsim/torques_min          | pacsim::msg::Wheels        | Lower bound torque value at each wheel's motor (Nm)                                   | NO |
| /pacsim/torques_max          | pacsim::msg::Wheels        | Upper bound torque value at each wheel's motor (Nm)                                   | YES |

### Outputs

TBD


## Other notes

The simulation's interfaces are provided so that motor controllers with an internal control loop for the wheel speeds (e.g. AMK or Electrophorus Engineering) can be fully modelled in the simulator. The default model does not contain this control loop. The values provided for `torques_max` are the actual torques applied to the wheels.

The example launch file (example.launch.py) shows an example of how to start the simulator node and the robot_state_publisher for the 3D visualisation of the car.

The sensors and vehicle model are configured using config files. Examples are provided in the config folder. Things such as the discipline or the path of the track file or config files are defined using ROS2 parameters.

The default vehicle model provided is rather simple and just meant to be a starting point. You are encouraged to integrate your own vehicle model by implementing the `IVehicleModel` class.
