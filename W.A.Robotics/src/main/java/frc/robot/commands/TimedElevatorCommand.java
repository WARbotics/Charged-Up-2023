package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj.Timer;

public class TimedElevatorCommand extends CommandBase{
    private ElevatorSubsystem elevatorSubsystem;
   
    private double initTime;
    private double speed;

    public TimedElevatorCommand(ElevatorSubsystem elevator, double speed){
        elevatorSubsystem = elevator;
        this.speed = speed;
        initTime = Timer.getFPGATimestamp();
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute(){
        while (Timer.getFPGATimestamp() - initTime <= 500){
            elevatorSubsystem.setMotor(speed);
    }
}
    
    @Override
    public void end(boolean interrupted){
        elevatorSubsystem.elevatorOff();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
