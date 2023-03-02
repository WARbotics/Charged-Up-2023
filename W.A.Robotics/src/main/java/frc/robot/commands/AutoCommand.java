package frc.robot.commands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ElevatorCommand;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;
import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoCommand {
    public DrivetrainSubsystem drive;
    private final SendableChooser<Command> dropDown;

    public AutoCommand(DrivetrainSubsystem drive){
        this.drive = drive;
        dropDown = new SendableChooser<>();
    dropDown.addOption("Example", run(() -> {
      getCommand("New Path", true);
    }));

    SmartDashboard.putData("Auto Selection", dropDown);
    }

    public Command getSelectedCommand() {
        return dropDown.getSelected();
      }

    private Command getCommand(String pathName, boolean isFirstPath){
        PathPlannerTrajectory traj = PathPlanner.loadPath(
      pathName,
      DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
      Constants.MAX_ACCEL_METERS_PER_SECOND_SQUARED);

      return sequence(
        run(() -> {
            if (isFirstPath){
                drive.resetOdometry(traj.getInitialHolonomicPose());
            }
        },drive),
        new PPSwerveControllerCommand(traj, drive::getPose, drive.m_kinematics, 
            new PIDController(Constants.X_CONTROLLER_KP, 0, 0), 
            new PIDController(Constants.Y_CONTROLLER_KP, 0, 0), 
            new PIDController(Constants.THETA_CONTROLLER_KP, 0, 0), 
            drive::setModuleStates, 
            drive)
        );
    } 
}
