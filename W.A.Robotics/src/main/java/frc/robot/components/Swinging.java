package frc.robot.components;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class Swinging extends CommandBase {

    private TalonFX leftMotor;


    public Swinging(TalonFX leftMotor){
        this.leftMotor = leftMotor;
        this.leftMotor.setNeutralMode(NeutralMode.Brake);
        
    }

    public void SwingArmForward(){
        leftMotor.set(ControlMode.PercentOutput, .5);
        leftMotor.configVoltageCompSaturation(11);
        leftMotor.enableVoltageCompensation(true);
    }

    public void SwingArmBackward(){
        leftMotor.set(ControlMode.PercentOutput, -.5);
        leftMotor.configVoltageCompSaturation(11);
        leftMotor.enableVoltageCompensation(true);
    } 

    public void SwingArmOff(){
        leftMotor.set(ControlMode.PercentOutput, 0);
    }
    
}
