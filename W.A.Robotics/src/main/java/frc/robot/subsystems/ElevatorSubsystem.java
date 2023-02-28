package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.CommandBase;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ElevatorSubsystem extends SubsystemBase{
    private final CANSparkMax elevatorMotor = new CANSparkMax(21, MotorType.kBrushless);

    
    public void elevatorUp(){
        elevatorMotor.set(-0.3);
    }

    public void elevatorDown(){
        elevatorMotor.set(0.3);
    }

    public void elevatorOff(){
        elevatorMotor.set(0);
    }
    
    @Override
    public void periodic(){ 
    }
}
