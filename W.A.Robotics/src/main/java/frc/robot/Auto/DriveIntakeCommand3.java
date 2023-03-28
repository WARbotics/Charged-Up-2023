package frc.robot.Auto;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants;
import frc.robot.commands.IntakeCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj.Timer;

public class DriveIntakeCommand3 extends ParallelRaceGroup{
    public IntakeSubsystem intake;
    public DrivetrainSubsystem drive;
    private PathPlannerTrajectory driveForward;

    public DriveIntakeCommand3(IntakeSubsystem intake, DrivetrainSubsystem drive){
        this.intake = intake;
        this.drive = drive;

        driveForward = PathPlanner.loadPath("Drive Forward 3", 1.00, 1.00);

        PPSwerveControllerCommand driveForwardCommand = 
        new PPSwerveControllerCommand(driveForward, drive::getPose, drive.m_kinematics, 
                new PIDController(Constants.X_CONTROLLER_KP, 0, 0), 
                new PIDController(Constants.Y_CONTROLLER_KP, 0, 0), 
                new PIDController(Constants.THETA_CONTROLLER_KP, 0, 0), 
                drive::setModuleStates, 
                drive);
      
                IntakeCommand runIntakeIn = new IntakeCommand(intake, -0.3);
                drive.zeroGyroCommand();

                addCommands(new InstantCommand(() -> drive.resetOdometry(driveForward.getInitialHolonomicPose())),
                runIntakeIn,
                driveForwardCommand
                );
    }
}

