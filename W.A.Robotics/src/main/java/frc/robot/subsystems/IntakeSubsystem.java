package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.CommandBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class IntakeSubsystem extends SubsystemBase {
    private final CANSparkMax intakeMotorRight = new CANSparkMax(22, MotorType.kBrushless);
    private final CANSparkMax intakeMotorLeft = new CANSparkMax(23, MotorType.kBrushless);
    
    public IntakeSubsystem(){
        intakeMotorRight.setInverted(true);
    }

    public void setMotor(double speed){
        intakeMotorLeft.set(speed);
        intakeMotorRight.set(speed);
    }
    
    public void intakeForward(){
        intakeMotorRight.set(.05);
        intakeMotorLeft.set(.05);
    }

    public void intakeBackward(){
        intakeMotorRight.set(-0.5);
        intakeMotorLeft.set(-0.5);
    }

    public void intakeHold(){
        intakeMotorRight.set(.01);
        intakeMotorLeft.set(.01);
    }

    public void intakeOff(){
        intakeMotorRight.set(0);
        intakeMotorLeft.set(0);
    }

    @Override
    public void periodic(){
    }

    
}
