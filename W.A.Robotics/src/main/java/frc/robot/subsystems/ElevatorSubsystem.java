package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ElevatorSubsystem extends SubsystemBase{
    private final CANSparkMax elevatorMotor = new CANSparkMax(21, MotorType.kBrushless);

    public void setMotor(double speed){
        elevatorMotor.set(speed);
    }

    public void elevatorOff(){
        elevatorMotor.set(0);
    }
    
    @Override
    public void periodic(){ 
    }
}
