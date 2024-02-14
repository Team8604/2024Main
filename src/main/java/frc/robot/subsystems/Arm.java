package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
    
    //Initialize motors
    public final CANSparkMax rightArm = new CANSparkMax(ArmConstants.kRightArm, MotorType.kBrushless);
    public final CANSparkMax leftArm = new CANSparkMax(ArmConstants.kLeftArm, MotorType.kBrushless);
    public final DutyCycleEncoder armEncoder = new DutyCycleEncoder(ArmConstants.kArmEncoder); 

}
