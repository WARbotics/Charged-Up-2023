package frc.robot.Auto;

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

public class ScoreWithTaxiAutoRightSlow extends SequentialCommandGroup{
    public IntakeSubsystem intake;
    public ArmSubsystem arm;
    public DrivetrainSubsystem drive;
    private PathPlannerTrajectory driveForward;
    private PathPlannerTrajectory inchForward;


    public ScoreWithTaxiAutoRightSlow(IntakeSubsystem intake, ArmSubsystem arm, DrivetrainSubsystem drive){
        driveForward = PathPlanner.loadPath("Drive Forward 3", 1.00, 1.00);
        inchForward = PathPlanner.loadPath("Inch Forward", 1.00, 1.00);

        PPSwerveControllerCommand inchForwardCommand = 
        new PPSwerveControllerCommand(inchForward, drive::getPose, drive.m_kinematics, 
            new PIDController(Constants.X_CONTROLLER_KP, 0, 0), 
            new PIDController(Constants.Y_CONTROLLER_KP, 0, 0), 
            new PIDController(Constants.THETA_CONTROLLER_KP, 0, 0), 
            drive::setModuleStates, 
            drive);

        PPSwerveControllerCommand driveForwardCommand = 
            new PPSwerveControllerCommand(driveForward, drive::getPose, drive.m_kinematics, 
            new PIDController(Constants.X_CONTROLLER_KP, 0, 0), 
            new PIDController(Constants.Y_CONTROLLER_KP, 0, 0), 
            new PIDController(Constants.THETA_CONTROLLER_KP, 0, 0), 
            drive::setModuleStates, 
            drive);
        
        IntakeCommand runIntakeOut = new IntakeCommand(intake, 1.0);                
        IntakeCommand runIntakeIn = new IntakeCommand(intake, -0.3);

        drive.zeroGyroCommand();

        addCommands(new InstantCommand(() -> drive.resetOdometry(driveForward.getInitialHolonomicPose())),
            arm.scoreLowCubeB(),
            new WaitCommand(1.0),
            runIntakeOut.withTimeout(1),
            arm.runToBasePostion(),   
            driveForwardCommand,
            arm.groundB(),
            new WaitCommand(1),
            inchForwardCommand,
            new WaitCommand(0.5),
            runIntakeIn.withTimeout(1.5),
            arm.runToBasePostion()
        );
    }
}

