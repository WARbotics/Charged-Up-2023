package frc.robot.components;

import com.revrobotics.CANSparkMax;



public class Intake {
    private CANSparkMax intakeMotorRight;
    private CANSparkMax intakeMotorLeft;
  

    public Intake(CANSparkMax intakeMotorRight, CANSparkMax intakeMotorLeft){
        this.intakeMotorRight = intakeMotorRight;
        this.intakeMotorRight.setInverted(true);
        this.intakeMotorLeft = intakeMotorLeft;
    }

    public void intakeForward(){
        this.intakeMotorRight.set(0.5);
        this.intakeMotorLeft.set(0.5);;
    }

    public void holdIntake(){
        this.intakeMotorLeft.set(-0.1);
        this.intakeMotorRight.set(-0.1);
    }

    public void intakeBackward(){
        this.intakeMotorRight.set(-0.5);
        this.intakeMotorLeft.set(-0.5);
    }

    public void intakeOff(){
        this.intakeMotorRight.set(0);
        this.intakeMotorLeft.set(0);

    }
}
