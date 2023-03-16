# Charged-Up-2023
Introducing Team 6925's code for Charged UP
For the first time, we have programmed completely in command base

The program consists of a SDS Swerve Drivetrain with CAN Spark Maxes that uses SDS's swerve library and runs in field-oriented mode.

The intake consists of two CAN Spark Maxes programmed to the driver controller.

The arm consists of two Talon FXs programmed to many different positions and is activated using the manipulator controller.

The elevator consists of a single CAN Spark Max that is programmed to the manipulator controller.

The autonomous system uses a WPILib's odometry system as well as Path Planner to create an auto path that allows us to score on the charge station. The auto system will also score a cube.

Finally, there is also a Limelight programmed to tell the driver when a target in within view.
