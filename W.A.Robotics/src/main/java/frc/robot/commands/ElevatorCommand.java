package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorCommand extends CommandBase{
    private ElevatorSubsystem elevatorSubsystem;
   
    private double speed;
    
    public ElevatorCommand(ElevatorSubsystem elevator, double speed){
        elevatorSubsystem = elevator;
        this.speed = speed;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute(){
        elevatorSubsystem.setMotor(speed);

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
