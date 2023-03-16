// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import static frc.robot.Constants.*;

import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;



public class DrivetrainSubsystem extends SubsystemBase {
  public static final double MAX_VOLTAGE = 12.0;
  private final SwerveDriveOdometry m_odometry;
  
  public static final double MAX_VELOCITY_METERS_PER_SECOND = 5880.0 / 60.0 *
          SdsModuleConfigurations.MK4I_L3.getDriveReduction() *
          SdsModuleConfigurations.MK4I_L3.getWheelDiameter() * Math.PI;
  
  public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
          Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

  public final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
          // Front left
          new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Front right
          new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Back left
          new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Back right
          new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0)
  );

  
  private final Pigeon2 m_pigeon = new Pigeon2(DRIVETRAIN_PIGEON_ID);
  
 
  private final SwerveModule m_frontLeftModule;
  private final SwerveModule m_frontRightModule;
  private final SwerveModule m_backLeftModule;
  private final SwerveModule m_backRightModule;

  private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

  public DrivetrainSubsystem() {
    ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

    
    m_frontLeftModule = Mk4iSwerveModuleHelper.createNeo(
            
            tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(0, 0),
            Mk4iSwerveModuleHelper.GearRatio.L3,
            FRONT_LEFT_MODULE_DRIVE_MOTOR,
            FRONT_LEFT_MODULE_STEER_MOTOR,
            FRONT_LEFT_MODULE_STEER_ENCODER,
            FRONT_LEFT_MODULE_STEER_OFFSET
    );

    m_frontRightModule = Mk4iSwerveModuleHelper.createNeo(
            tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(2, 0),
            Mk4iSwerveModuleHelper.GearRatio.L3,
            FRONT_RIGHT_MODULE_DRIVE_MOTOR,
            FRONT_RIGHT_MODULE_STEER_MOTOR,
            FRONT_RIGHT_MODULE_STEER_ENCODER,
            FRONT_RIGHT_MODULE_STEER_OFFSET
    );

    m_backLeftModule = Mk4iSwerveModuleHelper.createNeo(
            tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(4, 0),
            Mk4iSwerveModuleHelper.GearRatio.L3,
            BACK_LEFT_MODULE_DRIVE_MOTOR,
            BACK_LEFT_MODULE_STEER_MOTOR,
            BACK_LEFT_MODULE_STEER_ENCODER,
            BACK_LEFT_MODULE_STEER_OFFSET
    );

    m_backRightModule = Mk4iSwerveModuleHelper.createNeo(
            tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(6, 0),
            Mk4iSwerveModuleHelper.GearRatio.L3,
            BACK_RIGHT_MODULE_DRIVE_MOTOR,
            BACK_RIGHT_MODULE_STEER_MOTOR,
            BACK_RIGHT_MODULE_STEER_ENCODER,
            BACK_RIGHT_MODULE_STEER_OFFSET
    );

    m_odometry = new SwerveDriveOdometry(m_kinematics, this.getGyroscopeRotation(), new SwerveModulePosition[] {
        this.getPositionFromWheel(m_frontLeftModule),
        this.getPositionFromWheel(m_frontRightModule),
        this.getPositionFromWheel(m_backLeftModule),
        this.getPositionFromWheel(m_backRightModule)
}, new Pose2d(0.0,0.0, new Rotation2d(0.0)));
  }


public Command zeroGyroCommand() {
        return runOnce(this::zeroGyro).withName("ZeroGyroCommand");
      }
    
private void zeroGyro() {
        m_pigeon.setYaw(0.0);
      }

      public Pose2d getPose() {
        return m_odometry.getPoseMeters();
      }
public void resetOdometry(Pose2d pose) {
        m_odometry.resetPosition(getGyroscopeRotation(), new SwerveModulePosition[] {
                this.getPositionFromWheel(m_frontLeftModule),
                this.getPositionFromWheel(m_frontRightModule),
                this.getPositionFromWheel(m_backLeftModule),
                this.getPositionFromWheel(m_backRightModule)
        },pose);
      }


  public Rotation2d getGyroscopeRotation() {
    return Rotation2d.fromDegrees(m_pigeon.getYaw());

}

  public void drive(ChassisSpeeds chassisSpeeds) {
    m_chassisSpeeds = chassisSpeeds;
  }

  private SwerveModulePosition getPositionFromWheel(SwerveModule module) {
        return new SwerveModulePosition(module.getDriveVelocity(), new Rotation2d(module.getSteerAngle()));
}

SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);


public void setModuleStates(SwerveModuleState[] states){
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

        m_frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                        states[0].angle.getRadians());
        m_frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                        states[1].angle.getRadians());
        m_backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                        states[2].angle.getRadians());
        m_backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                        states[3].angle.getRadians());
}

  @Override
  public void periodic() {
    
        SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

        m_frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                        states[0].angle.getRadians());
        m_frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                        states[1].angle.getRadians());
        m_backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                        states[2].angle.getRadians());
        m_backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                        states[3].angle.getRadians());

        m_odometry.update(
                this.getGyroscopeRotation(),
                new SwerveModulePosition[] {
                        this.getPositionFromWheel(m_frontLeftModule),
                        this.getPositionFromWheel(m_frontRightModule),
                        this.getPositionFromWheel(m_backLeftModule),
                        this.getPositionFromWheel(m_backRightModule)
                }
        );

        SmartDashboard.putNumber("X", m_odometry.getPoseMeters().getTranslation().getX());
        SmartDashboard.putNumber("Y", m_odometry.getPoseMeters().getTranslation().getY());

  }
}
