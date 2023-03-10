// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.TimedElevatorCommand;
import frc.robot.commands.ElevatorCommand;





/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  
  // The robot's subsystems and commands are defined here...
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  private final IntakeSubsystem intake = new IntakeSubsystem();
  private final ElevatorSubsystem elevator = new ElevatorSubsystem();
  private final ArmSubsystem arm = new ArmSubsystem();


  
  private final XboxController m_controller = new XboxController(0);
  public final Joystick operator = new Joystick(1); 

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation
    m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
            m_drivetrainSubsystem,
            () -> -modifyAxis(m_controller.getLeftY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(m_controller.getLeftX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(m_controller.getRightX()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
    ));



    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_controller, XboxController.Button.kY.value).onTrue(m_drivetrainSubsystem.zeroGyroCommand());
    new JoystickButton(m_controller, XboxController.Button.kRightBumper.value).whileTrue(new IntakeCommand(intake, 0.5));
    new JoystickButton(m_controller, XboxController.Button.kLeftBumper.value).whileTrue(new IntakeCommand(intake, -0.5));
    new JoystickButton(operator, 1).whileTrue(new IntakeCommand(intake, -0.1));
    
    new JoystickButton(operator, 6).whileTrue(new ElevatorCommand(elevator, 0.6));
    new JoystickButton(operator, 4).whileTrue(new ElevatorCommand(elevator, -0.6));
    new JoystickButton(m_controller, XboxController.Button.kA.value).onTrue(new TimedElevatorCommand(elevator, -0.1));


    new JoystickButton(operator, 7).onTrue(arm.scoreLowCubeF());
    new JoystickButton(operator, 8).onTrue(arm.scoreLowCubeB());
    new JoystickButton(operator, 3).onTrue(arm.scoreHighF());
    new JoystickButton(operator, 5).onTrue(arm.scoreHighB());
    new JoystickButton(operator, 12).onTrue(arm.scoreLowConeF());
    new JoystickButton(operator, 11).onTrue(arm.scoreLowConeB());
    new JoystickButton(operator, 2).onTrue(arm.runToBasePostion());
    new JoystickButton(operator, 10).onTrue(arm.groundF());
    new JoystickButton(operator, 9).onTrue(arm.groundB());
    new JoystickButton(m_controller, XboxController.Button.kX.value).onTrue(arm.rampForward());
    new JoystickButton(m_controller, XboxController.Button.kB.value).onTrue(arm.rampBackward());




  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new InstantCommand();
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.05);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }
}
