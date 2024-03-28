package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase{
    // Initialize motors
    private final TalonFX rightClimber = new TalonFX(ClimberConstants.kRightClimber, ClimberConstants.CANBUS_NAME);
    private final TalonFX leftClimber = new TalonFX(ClimberConstants.kLeftClimber, ClimberConstants.CANBUS_NAME);

    public double rightClimberPosition;    
    public double leftClimberPosition;   
    
    public double leftEncoderStartPosition = leftClimber.getPosition().getValueAsDouble();
    public double rightEncoderStartPosition = rightClimber.getPosition().getValueAsDouble();

    public Climber(){
        rightClimber.setSafetyEnabled(true);
        leftClimber.setSafetyEnabled(true);
    }

    public void setLeftVoltage(double volts){
        if (getLeftInRange(volts)) {
            if (leftSlow(volts)) {
                volts *= 0.5;
            } 
        } else if (RobotContainer.climbOverideButton.getAsBoolean()) {
            volts *= 0.5;
        } else {
            volts = 0;
        }
        leftClimber.setVoltage(MathUtil.clamp(volts, -12, 12));
    }
    
    public void setRightVoltage(double volts){
        if (getRightInRange(volts)) {
            if (rightSlow(volts)) {
                volts *= ClimberConstants.slowModeSpeed;
            } 
        } else if (RobotContainer.climbOverideButton.getAsBoolean()) {
            volts *= 0.5;
        } else {
            volts = 0;
        }
        rightClimber.setVoltage(MathUtil.clamp(volts, -12, 12));
    }

    public boolean getLeftInRange(double volts){
        if (volts < 0 && leftClimberPosition > ClimberConstants.leftUpPosition + leftEncoderStartPosition || volts > 0 && leftClimberPosition < ClimberConstants.leftDownPosition + leftEncoderStartPosition){
            return true;
        }
        return false;
    }

    public boolean leftSlow(double volts) {
        // used to slow down arm right before limit
        if (volts < 0 && leftClimberPosition < ClimberConstants.leftUpPosition + leftEncoderStartPosition + ClimberConstants.toleranceToStartSlow || volts > 0 && leftClimberPosition > ClimberConstants.leftDownPosition + leftEncoderStartPosition - ClimberConstants.toleranceToStartSlow) {
            return true;
        }
        else {return false;}
    }

    public boolean getRightInRange(double volts){
        if (volts < 0 && rightClimberPosition > ClimberConstants.rightUpPosition + rightEncoderStartPosition || volts > 0 && rightClimberPosition < ClimberConstants.rightDownPosition + rightEncoderStartPosition){
            return true;
        }
        return false;
    }

    public boolean rightSlow(double volts) {
        // used to slow down arm right before limit
        if (volts < 0 && rightClimberPosition < ClimberConstants.rightUpPosition + rightEncoderStartPosition + ClimberConstants.toleranceToStartSlow || volts > 0 && rightClimberPosition > ClimberConstants.rightDownPosition + rightEncoderStartPosition - ClimberConstants.toleranceToStartSlow) {
            return true;
        }
        else {return false;}
    }

    public void fullBreak(){
        rightClimber.setNeutralMode(NeutralModeValue.Brake);
        leftClimber.setNeutralMode(NeutralModeValue.Brake);
    }
    
    @Override
    public void periodic(){
        leftClimberPosition = leftClimber.getPosition().getValueAsDouble();
        rightClimberPosition = rightClimber.getPosition().getValueAsDouble();

        SmartDashboard.putNumber("Left Climber Motor Position", leftClimberPosition);
        SmartDashboard.putNumber("Right Climber Motor Position", rightClimberPosition);
        SmartDashboard.putNumber("Left Climber Motor Voltage", leftClimber.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Right Climber Motor Voltage", rightClimber.getMotorVoltage().getValueAsDouble());

        SmartDashboard.putNumber("Left Encoder Start Position", leftEncoderStartPosition);
        SmartDashboard.putNumber("Right Encoder Start Position", rightEncoderStartPosition);
    }
}