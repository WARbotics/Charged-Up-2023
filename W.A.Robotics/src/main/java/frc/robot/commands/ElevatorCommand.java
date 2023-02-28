package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj.Timer;

public class ElevatorCommand extends CommandBase{
    private ElevatorSubsystem elevatorSubsystem;
   
    private double initTime;
    
    public ElevatorCommand(ElevatorSubsystem elevator){
        elevatorSubsystem = elevator;
        initTime = Timer.getFPGATimestamp();
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        while(Timer.getFPGATimestamp() - initTime <= 20000){
            elevatorSubsystem.elevatorUp();
        }
        }
    
    @Override
    public void end(boolean interrupted){
        elevatorSubsystem.elevatorOff();
    }

    @Override
    public boolean isFinished(){
        elevatorSubsystem.elevatorOff();
        return false;
    }
}
