// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.rmi.registry.RegistryHandler;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.components.Elevator;
import frc.robot.components.Swinging;
import frc.robot.components.Intake;;
/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private XboxController drive;
  private Joystick operator;
  private RobotContainer m_robotContainer;
  private Swinging armSwing;
  private Elevator elevator;
  private Intake intake;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    //Arm
    TalonFX leftMotor = new TalonFX(1);
    TalonFX rightMotor = new TalonFX(2);
    this.armSwing = new Swinging(leftMotor, rightMotor);

    CANSparkMax elevatorMotor = new CANSparkMax(21, MotorType.kBrushless);
    this.elevator = new Elevator(elevatorMotor);

    CANSparkMax intakeMotorRight = new CANSparkMax(22, MotorType.kBrushless);
    CANSparkMax intakeMotorLeft = new CANSparkMax(23, MotorType.kBrushless);
    this.intake = new Intake(intakeMotorRight, intakeMotorLeft);

    drive = new XboxController(0);
    operator = new Joystick(1);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    armSwing.zeroPosition();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    SmartDashboard.putNumber("Arm Position", armSwing.getPosition());
    if(operator.getRawButton(7)){
      armSwing.scoreLowCubeF();
    }
    if(operator.getRawButton(8)){
      armSwing.scoreLowCubeB();
    }
    if(operator.getRawButton(9)){
      armSwing.scoreHighF();
    }
    if(operator.getRawButton(10)){
      armSwing.scoreHighB();
    } 
    if(operator.getRawButton(12)){
      armSwing.runToBasePostion();
    }

    if(operator.getRawButton(5)){
      armSwing.runArmF();
    }else if(operator.getRawButton(3)){
      armSwing.runArmB();
    }else{
      armSwing.SwingArmOff();
    }

    SmartDashboard.putNumber("Elevator Position", elevator.getElevatorPosition());
    if(operator.getRawButton(6)){
      elevator.elevatorUp();
    }else if (operator.getRawButton(4)){ //borked
      elevator.elevatorDown();
    }else{
      elevator.elevatorOff();
    }

    if (operator.getRawButton(1)){ // also borked
      intake.intakeForward();
      
    }else if(operator.getRawButton(2)){
      intake.intakeBackward();
    }else if(drive.getAButton()){
      intake.holdIntake();
    }else{
      intake.intakeOff();
    }

    }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
