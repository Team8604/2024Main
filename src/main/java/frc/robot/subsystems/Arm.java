package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {
    
    //Initialize motors
    private final CANSparkMax rightArm = new CANSparkMax(ArmConstants.kRightArm, MotorType.kBrushless);
    private final CANSparkMax leftArm = new CANSparkMax(ArmConstants.kLeftArm, MotorType.kBrushless);
    private final DutyCycleEncoder armEncoder = new DutyCycleEncoder(ArmConstants.kArmEncoder); 

    public Arm() {
        rightArm.restoreFactoryDefaults();
        leftArm.restoreFactoryDefaults();
        leftArm.follow(rightArm, true);

    }

    public double getAngle() {
        return armEncoder.getAbsolutePosition();
    }

    public void setSpeed(double speed) {
        rightArm.set(MathUtil.clamp(speed, -1 * ArmConstants.kMaxSpeed, ArmConstants.kMaxSpeed));
    }

    public void periodic() {
        SmartDashboard.putNumber("Arm Angle", armEncoder.getAbsolutePosition());
    }

}
