package frc.robot.Auto;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants;
import frc.robot.commands.IntakeCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class ScoreTwoWithBalanceAuto1 extends SequentialCommandGroup{
    public IntakeSubsystem intake;
    public ArmSubsystem arm;
    public DrivetrainSubsystem drive;
    private PathPlannerTrajectory driveBackward;
    private PathPlannerTrajectory parkChargeStation;

    public ScoreTwoWithBalanceAuto1(IntakeSubsystem intake, ArmSubsystem arm, DrivetrainSubsystem drive){
        this.intake = intake;
        this.arm = arm;
        this.drive = drive;

        parkChargeStation = PathPlanner.loadPath("Balance on Charge Station 1", 1.00, 1.00);
        driveBackward = PathPlanner.loadPath("Drive Backward 1", 1.00, 1.00);


        PPSwerveControllerCommand balancCommand = 
        new PPSwerveControllerCommand(parkChargeStation, drive::getPose, drive.m_kinematics, 
                new PIDController(Constants.X_CONTROLLER_KP, 0, 0), 
                new PIDController(Constants.Y_CONTROLLER_KP, 0, 0), 
                new PIDController(Constants.THETA_CONTROLLER_KP, 0, 0), 
                drive::setModuleStates, 
                drive);

        PPSwerveControllerCommand driveBackwardCommand = 
        new PPSwerveControllerCommand(driveBackward, drive::getPose, drive.m_kinematics, 
                 new PIDController(Constants.X_CONTROLLER_KP, 0, 0), 
                 new PIDController(Constants.Y_CONTROLLER_KP, 0, 0), 
                new PIDController(Constants.THETA_CONTROLLER_KP, 0, 0), 
                    drive::setModuleStates, 
                drive);
        
        IntakeCommand runIntakeOut = new IntakeCommand(intake, 0.7);
        DriveIntakeCommand1 driveWithIntake = new DriveIntakeCommand1(intake, drive);

        addCommands(
            arm.groundB(),
            driveWithIntake,
            arm.runToBasePostion(),
            driveBackwardCommand,
            arm.scoreLowCubeB(),
            new WaitCommand(1.0),
            runIntakeOut.withTimeout(2),
            arm.runToBasePostion(),
            balancCommand
            );
    }
}
