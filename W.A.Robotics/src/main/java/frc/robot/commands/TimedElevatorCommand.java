package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj.Timer;

public class TimedElevatorCommand extends CommandBase{
    private ElevatorSubsystem elevatorSubsystem;
   
    private double initTime;
    private double speed;
    private Timer timer;

    public TimedElevatorCommand(ElevatorSubsystem elevator, double speed, Timer timer){
        elevatorSubsystem = elevator;
        this.speed = speed;
        this.timer = timer;
        initTime = timer.get();
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        timer.start();
    }

    @Override
    public void execute(){
        while (timer.get() - initTime <= 0.5){
            elevatorSubsystem.setMotor(speed);
    }
}
    
    @Override
    public void end(boolean interrupted){
        timer.stop();
        timer.reset();
        elevatorSubsystem.elevatorOff();
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
