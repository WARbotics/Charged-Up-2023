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

public class ScoreWithBalanceAuto extends SequentialCommandGroup{
    public DrivetrainSubsystem drive;
    private PathPlannerTrajectory park;
    public IntakeSubsystem intake;
    public ArmSubsystem arm; 

    public ScoreWithBalanceAuto(DrivetrainSubsystem drive, ArmSubsystem arm, IntakeSubsystem intake){
        this.drive = drive;
        this.arm = arm;
        this.intake = intake;
        park = PathPlanner.loadPath("Park on Charge Station", Constants.MAX_VELOCITY_METERS_PER_SECOND, Constants.MAX_ACCEL_METERS_PER_SECOND_SQUARED);

        PPSwerveControllerCommand parkCommand = 
        new PPSwerveControllerCommand(park, drive::getPose, drive.m_kinematics, 
                new PIDController(Constants.X_CONTROLLER_KP, 0, 0), 
                new PIDController(Constants.Y_CONTROLLER_KP, 0, 0), 
                new PIDController(Constants.THETA_CONTROLLER_KP, 0, 0), 
                drive::setModuleStates, 
                drive);
        
        IntakeCommand runIntakeOut = new IntakeCommand(intake, 0.7);
        IntakeCommand runIntakeIn = new IntakeCommand(intake, -0.1);
        drive.zeroGyroCommand();


        addCommands(new InstantCommand(() -> drive.resetOdometry(park.getInitialHolonomicPose())),
        runIntakeIn.withTimeout(2), 
        arm.scoreLowCubeB(),
         new WaitCommand(1.0),
         runIntakeOut.withTimeout(2),
         arm.runToBasePostion(),
        parkCommand
        );
    };
}