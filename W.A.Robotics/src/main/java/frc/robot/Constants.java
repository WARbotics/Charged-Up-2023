// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final double DRIVETRAIN_TRACKWIDTH_METERS = .58; 
    
    public static final double DRIVETRAIN_WHEELBASE_METERS = .40; 

    public static final int DRIVETRAIN_PIGEON_ID = 1; 

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 20;
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 1; 
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 2; 
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(144.58); 

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 16; 
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 17; 
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 3; 
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(39.550);

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 3; 
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 2; 
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 1; 
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(313.593); 

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 19; 
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 18; 
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 4; 
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(44.121); 

    public static final double MAX_ACCEL_METERS_PER_SECOND_SQUARED = 1.0;
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 1.0;

    public static final double X_CONTROLLER_KP = 0.0;
    public static final double Y_CONTROLLER_KP = 0.0;
    public static final double THETA_CONTROLLER_KP = 0.0;

}
