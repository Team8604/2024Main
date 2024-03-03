package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.arm.*;


public class Arm extends SubsystemBase {
    
    //Initialize motors, encoder, and limit switches
    private final CANSparkMax rightArm = new CANSparkMax(ArmConstants.kRightArm, MotorType.kBrushless);
    private final CANSparkMax leftArm = new CANSparkMax(ArmConstants.kLeftArm, MotorType.kBrushless);
    private final DutyCycleEncoder armEncoder = new DutyCycleEncoder(ArmConstants.kArmEncoder); 
    public final SparkLimitSwitch rightFowardLimitSwitch = rightArm.getForwardLimitSwitch(ArmConstants.kArmLimitSwitchType);
    public final SparkLimitSwitch leftFowardLimitSwitch = leftArm.getForwardLimitSwitch(ArmConstants.kArmLimitSwitchType);
    public final SparkLimitSwitch rightBackwardLimitSwitch = rightArm.getReverseLimitSwitch(ArmConstants.kArmLimitSwitchType);
    public final SparkLimitSwitch leftBackwardLimitSwitch = leftArm.getReverseLimitSwitch(ArmConstants.kArmLimitSwitchType);
    


    public Arm() {
        rightArm.restoreFactoryDefaults();
        leftArm.restoreFactoryDefaults();
        rightArm.follow(leftArm, true);
        
        rightFowardLimitSwitch.enableLimitSwitch(true);
        leftFowardLimitSwitch.enableLimitSwitch(true);
        rightBackwardLimitSwitch.enableLimitSwitch(true);
        leftBackwardLimitSwitch.enableLimitSwitch(true);

    }

    public double getAngle() {
        return armEncoder.getAbsolutePosition();
    }

    public void setSpeed(double speed) {
        leftArm.set(MathUtil.clamp(speed, -1 * ArmConstants.kMaxSpeed, ArmConstants.kMaxSpeed));
    }


    @Override
    public void periodic() {
    // This method will be called once per scheduler run

        SmartDashboard.putBoolean("Right Foward Arm Encoder", rightFowardLimitSwitch.isPressed());
        SmartDashboard.putBoolean("Left Foward Arm Angle", leftFowardLimitSwitch.isPressed());
        SmartDashboard.putBoolean("Right Backward Arm Angle", rightBackwardLimitSwitch.isPressed());
        SmartDashboard.putBoolean("Left Backward Arm Angle", leftBackwardLimitSwitch.isPressed());
        SmartDashboard.putNumber("Arm Angle", armEncoder.getAbsolutePosition());
        SmartDashboard.putNumber("Arm speed", RunArm.adjustArm);

        
    }

}
