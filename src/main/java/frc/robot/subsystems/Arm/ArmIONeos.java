package frc.robot.subsystems.Arm;

import com.revrobotics.CANSparkLowLevel.MotorType;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants.ArmConstants;

public class ArmIONeos implements ArmIO {
    private final CANSparkMax rightArm = new CANSparkMax(ArmConstants.kRightArm, MotorType.kBrushless);
    private final CANSparkMax leftArm = new CANSparkMax(ArmConstants.kLeftArm, MotorType.kBrushless);
    private final DutyCycleEncoder armEncoder = new DutyCycleEncoder(ArmConstants.kArmEncoder); 

    private final SparkLimitSwitch rightFowardLimitSwitch = rightArm.getForwardLimitSwitch(ArmConstants.kArmLimitSwitchType);
    private final SparkLimitSwitch leftFowardLimitSwitch = leftArm.getForwardLimitSwitch(ArmConstants.kArmLimitSwitchType);
    private final SparkLimitSwitch rightBackwardLimitSwitch = rightArm.getReverseLimitSwitch(ArmConstants.kArmLimitSwitchType);
    private final SparkLimitSwitch leftBackwardLimitSwitch = leftArm.getReverseLimitSwitch(ArmConstants.kArmLimitSwitchType);

    private final RelativeEncoder leftMotorEncoder = leftArm.getEncoder();
    private final RelativeEncoder rightMotorEncoder = rightArm.getEncoder();

    public ArmIONeos() {
        rightArm.restoreFactoryDefaults();
        leftArm.restoreFactoryDefaults();
        rightArm.follow(leftArm, true);

        rightFowardLimitSwitch.enableLimitSwitch(true);
        leftFowardLimitSwitch.enableLimitSwitch(true);
        rightBackwardLimitSwitch.enableLimitSwitch(true);
        leftBackwardLimitSwitch.enableLimitSwitch(true);
    }

    public void updateInputs(ArmInputs inputs) {
        inputs.absoluteEncoderAngle = (armEncoder.getAbsolutePosition() * 360 + ArmConstants.kArmEncoderOffset);

        inputs.rightMotorEncoderAngle = rightMotorEncoder.getPosition();
        inputs.leftMotorEncoderAngle = leftMotorEncoder.getPosition();

        inputs.motorLeftAmpsOutput = leftArm.getOutputCurrent();
        inputs.motorRightAmpsOutput = rightArm.getOutputCurrent();

        inputs.rightFowardArmLimitSwitchEnabled = rightFowardLimitSwitch.isLimitSwitchEnabled();
        inputs.leftFowardArmLimitSwitchEnabled = leftFowardLimitSwitch.isLimitSwitchEnabled();
        inputs.rightBackwardArmLimitSwitchEnabled = rightBackwardLimitSwitch.isLimitSwitchEnabled();
        inputs.leftBackwardArmLimitSwitchEnabled = leftBackwardLimitSwitch.isLimitSwitchEnabled();

        inputs.rightFowardArmLimitSwitchPressed = rightFowardLimitSwitch.isPressed();
        inputs.leftFowardArmLimitSwitchPressed = leftFowardLimitSwitch.isPressed();
        inputs.rightBackwardArmLimitSwitchPressed = rightBackwardLimitSwitch.isPressed();
        inputs.leftBackwardArmLimitSwitchPressed = leftBackwardLimitSwitch.isPressed();

    }

    public void setVoltage(double volts) {
        Logger.recordOutput("Arm/OutputVoltage", volts);
        leftArm.setVoltage(volts);
    }
}
