package frc.robot.commands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ElevatorCommand;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj.Timer;

public class AutoCommand extends SequentialCommandGroup{
    public DrivetrainSubsystem drive;
    private PathPlannerTrajectory traj;
    public IntakeSubsystem intake;
    public ArmSubsystem arm; 
    public Timer timer;

    public AutoCommand(DrivetrainSubsystem drive, ArmSubsystem arm, IntakeSubsystem intake){
        this.drive = drive;
        this.arm = arm;
        this.intake = intake;
        traj = PathPlanner.loadPath("New Path", Constants.MAX_VELOCITY_METERS_PER_SECOND, Constants.MAX_ACCEL_METERS_PER_SECOND_SQUARED);

        PPSwerveControllerCommand testPath = 
        new PPSwerveControllerCommand(traj, drive::getPose, drive.m_kinematics, 
                new PIDController(Constants.X_CONTROLLER_KP, 0, 0), 
                new PIDController(Constants.Y_CONTROLLER_KP, 0, 0), 
                new PIDController(Constants.THETA_CONTROLLER_KP, 0, 0), 
                drive::setModuleStates, 
                drive);
        
        IntakeCommand runIntake = new IntakeCommand(intake, 0.5);

        addCommands(new InstantCommand(() -> drive.resetOdometry(traj.getInitialHolonomicPose())),
         arm.scoreLowCubeB(),
         runIntake.withTimeout(2),
         arm.runToBasePostion(),
        testPath
        );
    };
}