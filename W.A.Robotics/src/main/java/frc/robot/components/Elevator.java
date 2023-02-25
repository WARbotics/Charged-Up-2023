package frc.robot.components;

import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

public class Elevator {

    private CANSparkMax elevatorMotor;
    private RelativeEncoder elevatorEncoder;
    private SparkMaxPIDController elevatorPIDController;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
    

    public Elevator (CANSparkMax elevatorMotor){
        this.elevatorMotor = elevatorMotor;
        this.elevatorEncoder = elevatorMotor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
        this.elevatorPIDController = elevatorMotor.getPIDController();
        this.elevatorPIDController.setFeedbackDevice(elevatorEncoder);

        kP = 0.05; 
        kI = 0;
        kD = 0.05; 
        kIz = 0; 
        kFF = 0; 
        kMaxOutput = 0.3; 
        kMinOutput = -0.3;

        elevatorPIDController.setP(kP);
        elevatorPIDController.setI(kI);
        elevatorPIDController.setD(kD);
        elevatorPIDController.setIZone(kIz);
        elevatorPIDController.setFF(kFF);
        elevatorPIDController.setOutputRange(kMinOutput, kMaxOutput);

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

    public void zeroElevator(){
        this.elevatorEncoder.setPosition(0);
    }

    public void elevatorTest(){
        this.elevatorPIDController.setReference(30, CANSparkMax.ControlType.kPosition);
    }


    public double getElevatorPosition(){
        double elevatorPosition = elevatorEncoder.getPosition();
        return elevatorPosition;
    }
}