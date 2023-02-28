package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends CommandBase{
    private IntakeSubsystem intakeSubsystem;
    private  double speed;

    public IntakeCommand(IntakeSubsystem intake, double speed){
        intakeSubsystem = intake;
        this.speed = speed;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize(){
    }

    @Override
    public void execute(){
        intakeSubsystem.setMotor(speed);
    }

    @Override
    public void end(boolean interrupted){
        intakeSubsystem.intakeOff();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}