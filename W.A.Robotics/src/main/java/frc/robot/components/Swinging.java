package frc.robot.components;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class Swinging extends CommandBase {

    private TalonFX leftMotor;
    private TalonFX rightMotor;

    private double kF = 0;
    private double kP = .014;
    private double kI = 0.0;
    private double kD = 0.04;


    public Swinging(TalonFX leftMotor, TalonFX rightMotor){
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        this.rightMotor.follow(leftMotor);
        this.rightMotor.setInverted(true);
        this.rightMotor.setNeutralMode(NeutralMode.Brake);
        this.leftMotor.setNeutralMode(NeutralMode.Brake);

        this.leftMotor.configFactoryDefault();

        this.leftMotor.config_kF(0, kF, 30);
        this.leftMotor.config_kP(0, kP, 30);
        this.leftMotor.config_kI(0, kI, 30);
        this.leftMotor.config_kD(0, kD, 30);
        
    }

    public double getPosition(){
        double armPosition = leftMotor.getSelectedSensorPosition();
        return armPosition;

    }
    
    public void zeroPosition(){
        this.leftMotor.setSelectedSensorPosition(0);
    }
    
    public void scoreLowCubeF(){
        leftMotor.set(ControlMode.Position, -25000);
        leftMotor.configVoltageCompSaturation(11);
        leftMotor.enableVoltageCompensation(true);
    }

    public void scoreLowCubeB(){
        leftMotor.set(ControlMode.Position, 25000);
        leftMotor.configVoltageCompSaturation(11);
        leftMotor.enableVoltageCompensation(true);
    }

    public void scoreHighF(){
        leftMotor.set(ControlMode.Position, 23300);
        leftMotor.configVoltageCompSaturation(11);
        leftMotor.enableVoltageCompensation(true);
    }

    public void scoreHighB(){
        leftMotor.set(ControlMode.Position, -23300);
        leftMotor.configVoltageCompSaturation(11);
        leftMotor.enableVoltageCompensation(true);
    }

    public void scoreLowConeF(){
        leftMotor.set(ControlMode.Position, 27000);
        leftMotor.configVoltageCompSaturation(11);
        leftMotor.enableVoltageCompensation(true);
    }

    public void scoreLowConeB(){
        leftMotor.set(ControlMode.Position, -27000);
        leftMotor.configVoltageCompSaturation(11);
        leftMotor.enableVoltageCompensation(true);
    }

    public void groundF(){
        leftMotor.set(ControlMode.Position, 47000);
        leftMotor.configVoltageCompSaturation(11);
        leftMotor.enableVoltageCompensation(true);
    }

    public void groundB(){
        leftMotor.set(ControlMode.Position, -47000);
        leftMotor.configVoltageCompSaturation(11);
        leftMotor.enableVoltageCompensation(true);
    }

    public void runArmF(){
        leftMotor.set(ControlMode.PercentOutput, 0.1);
        leftMotor.configVoltageCompSaturation(11);
        leftMotor.enableVoltageCompensation(true);
    }

    public void runArmB(){
        leftMotor.set(ControlMode.PercentOutput, 0.1);
        leftMotor.configVoltageCompSaturation(11);
        leftMotor.enableVoltageCompensation(true);
    }

    public void runToBasePostion(){
        leftMotor.set(ControlMode.PercentOutput, -0.1);
        leftMotor.configVoltageCompSaturation(11);
        leftMotor.enableVoltageCompensation(true);
    } 

    public void SwingArmOff(){
        leftMotor.set(ControlMode.PercentOutput, 0);
    }
    
    //ground position 47000
    // high cone 23300
    //low cone 27000
}
