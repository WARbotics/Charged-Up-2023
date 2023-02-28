package frc.robot.components;

import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;


public class Elevator {

    private CANSparkMax elevatorMotor;
    private RelativeEncoder elevatorEncoder;
    private SparkMaxPIDController elevatorPIDController;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
    private double initTime;
    

    public Elevator (CANSparkMax elevatorMotor){
        this.elevatorMotor = elevatorMotor;
        initTime = Timer.getFPGATimestamp();
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

    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);
    SmartDashboard.putNumber("Set Rotations", 30);

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
        double p = SmartDashboard.getNumber("P Gain", 0);
        double i = SmartDashboard.getNumber("I Gain", 0);
        double d = SmartDashboard.getNumber("D Gain", 0);
        double iz = SmartDashboard.getNumber("I Zone", 0);
        double ff = SmartDashboard.getNumber("Feed Forward", 0);
        double max = SmartDashboard.getNumber("Max Output", 0);
        double min = SmartDashboard.getNumber("Min Output", 0);
        double rotations = SmartDashboard.getNumber("Set Rotations", 0);

        if((p != kP)){elevatorPIDController.setP(p); kP = p; }
        if((i != kI)) { elevatorPIDController.setI(i); kI = i; }
        if((d != kD)) { elevatorPIDController.setD(d); kD = d; }
        if((iz != kIz)) { elevatorPIDController.setIZone(iz); kIz = iz; }
        if((ff != kFF)) { elevatorPIDController.setFF(ff); kFF = ff; }
        if((max != kMaxOutput) || (min != kMinOutput)) { 
        elevatorPIDController.setOutputRange(min, max); 
        kMinOutput = min; kMaxOutput = max; 
        }

        elevatorPIDController.setReference(rotations, CANSparkMax.ControlType.kPosition);
        SmartDashboard.putNumber("SetPoint", rotations);
    }

    public void testElevatorOne(){
        while(Timer.getFPGATimestamp() - initTime <= 20000){
            elevatorMotor.set(0.3);
        }
    }

    
    public double getElevatorPosition(){
        double elevatorPosition = elevatorEncoder.getPosition();
        return elevatorPosition;
    }
}