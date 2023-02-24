package frc.robot.components;

import com.revrobotics.CANSparkMax;

public class Elevator {

    private CANSparkMax elevatorMotor;

    public Elevator (CANSparkMax elevatorMotor){
        this.elevatorMotor = elevatorMotor;
    }

    public void elevatorUp(){
        this.elevatorMotor.set(0.6);
    }

    public void elevatorDown(){
        this.elevatorMotor.set(-0.6);
    }

    public void elevatorOff(){
        this.elevatorMotor.set(0);
    }

    public double getElevatorPosition(){
        double elevatorPosition = elevatorMotor.get();
        return elevatorPosition;
    }
}