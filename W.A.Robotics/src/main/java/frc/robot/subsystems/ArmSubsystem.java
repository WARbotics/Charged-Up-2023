package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class ArmSubsystem extends SubsystemBase {
    private final TalonFX armMortorRight = new TalonFX(1);
    private final TalonFX armMotorleft = new TalonFX(2);

    private final double kF = 0;
    private final double kP = .008;
    private final double kI = 0.0;
    private final double kD = 0.04;

    public ArmSubsystem(){
        armMortorRight.setInverted(true);
        armMortorRight.follow(armMotorleft);
        armMortorRight.setNeutralMode(NeutralMode.Brake);
        armMotorleft.setNeutralMode(NeutralMode.Brake);

        armMotorleft.configFactoryDefault();

        armMotorleft.config_kF(0, kF, 30);
        armMotorleft.config_kP(0, kP, 30);
        armMotorleft.config_kI(0, kI, 30);
        armMotorleft.config_kD(0, kD, 30);

        armMotorleft.setSelectedSensorPosition(0);
    }

    public CommandBase scoreLowCubeF(){
        return this.runOnce(() -> armMotorleft.set(ControlMode.Position, -25000));
    }

    public CommandBase scoreLowCubeB(){
        return this.runOnce(() -> armMotorleft.set(ControlMode.Position, 25000));
    }

    public CommandBase scoreHighF(){
        return this.runOnce(() -> armMotorleft.set(ControlMode.Position, 16000));
    }

    public CommandBase scoreHighB(){
        return this.runOnce(() -> armMotorleft.set(ControlMode.Position, -16000));

    }

    public CommandBase scoreLowConeF(){
        return this.runOnce(() -> armMotorleft.set(ControlMode.Position, 27000));

    }

    public CommandBase scoreLowConeB(){
        return this.runOnce(() -> armMotorleft.set(ControlMode.Position, -27000));

    }

    public CommandBase groundF(){
        return this.runOnce(() -> armMotorleft.set(ControlMode.Position, 40000));

    }

    public CommandBase groundB(){
        return this.runOnce(() -> armMotorleft.set(ControlMode.Position, -40000));

    }

    public void runArmF(){
        armMotorleft.set(ControlMode.PercentOutput, 0.1);
        armMotorleft.configVoltageCompSaturation(11);
        armMotorleft.enableVoltageCompensation(true);
    }

    public void runArmB(){
        armMotorleft.set(ControlMode.PercentOutput, -0.1);
        armMotorleft.configVoltageCompSaturation(11);
        armMotorleft.enableVoltageCompensation(true);
    }

    public CommandBase runToBasePostion(){
        return this.runOnce(() -> armMotorleft.set(ControlMode.Position, 0));

    } 

    public void SwingArmOff(){
        armMotorleft.set(ControlMode.PercentOutput, 0);
    }
    
    public void zeroPosition(){
        armMotorleft.setSelectedSensorPosition(0);
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Arm Position", armMotorleft.getSelectedSensorPosition());
    }
}
