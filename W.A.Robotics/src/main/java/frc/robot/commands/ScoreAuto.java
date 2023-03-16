package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;


public class ScoreAuto extends SequentialCommandGroup{
    public IntakeSubsystem intake;
    public ArmSubsystem arm; 

    public ScoreAuto( IntakeSubsystem intake, ArmSubsystem arm){
        this.arm = arm;
        this.intake = intake;

        IntakeCommand runIntakeOut = new IntakeCommand(intake, 0.7);
        IntakeCommand runIntakeIn = new IntakeCommand(intake, -0.1);

        addCommands(
        runIntakeIn.withTimeout(2),
        arm.scoreLowCubeB(),
        new WaitCommand(1.0),
        runIntakeOut.withTimeout(2),
        arm.runToBasePostion());
    }
}
