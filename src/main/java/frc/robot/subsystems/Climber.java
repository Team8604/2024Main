
package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

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
            if (!leftSlow()) {
                leftClimber.setVoltage(MathUtil.clamp(volts, -12, 12));
            } else {
                leftClimber.setVoltage(MathUtil.clamp(volts/2, -12, 12));
            }
        } else if (RobotContainer.climbOverideButton.getAsBoolean()) {
            leftClimber.setVoltage(MathUtil.clamp(volts/2, -12, 12));
        }
    }
    
    public void setRightVoltage(double volts){
        if (getRightInRange(volts)) {
            if (!leftSlow()) {
                rightClimber.setVoltage(MathUtil.clamp(volts, -12, 12));
            } else {
                rightClimber.setVoltage(MathUtil.clamp(volts/2, -12, 12));
            }
        } else if (RobotContainer.climbOverideButton.getAsBoolean()) {
            rightClimber.setVoltage(MathUtil.clamp(volts/2, -12, 12));
        }
    }

    public boolean getLeftInRange(double volts){
        if (volts < 0 && leftClimberPosition > ClimberConstants.upPosition + leftEncoderStartPosition || volts > 0 && leftClimberPosition < ClimberConstants.downPosition + leftEncoderStartPosition){
            return true;
        }
        return false;
    }

    public boolean leftSlow() {
        if (leftClimberPosition < ClimberConstants.upPosition + leftEncoderStartPosition + ClimberConstants.toleranceToStartSlow || leftClimberPosition > ClimberConstants.downPosition + leftEncoderStartPosition - ClimberConstants.toleranceToStartSlow) {
            return true;
        }
        else {return false;}
    }

    public boolean getRightInRange(double volts){
        if (volts < 0 && rightClimberPosition > ClimberConstants.upPosition + rightEncoderStartPosition || volts > 0 && rightClimberPosition < ClimberConstants.downPosition + rightEncoderStartPosition){
            return true;
        }
        return false;
    }

    public boolean rightSlow() {
        if (rightClimberPosition < ClimberConstants.upPosition + rightEncoderStartPosition + ClimberConstants.toleranceToStartSlow || rightClimberPosition > ClimberConstants.downPosition + rightEncoderStartPosition - ClimberConstants.toleranceToStartSlow) {
            return true;
        }
        else {return false;}
    }

    
    @Override
    public void periodic(){
        leftClimberPosition = leftClimber.getPosition().getValueAsDouble();
        rightClimberPosition = rightClimber.getPosition().getValueAsDouble();

        SmartDashboard.putNumber("Left Climber Motor", leftClimber.get());
        SmartDashboard.putNumber("Right Climber Motor", rightClimber.get());
        SmartDashboard.putNumber("Left Climber Motor Position", leftClimberPosition);
        SmartDashboard.putNumber("Right Climber Motor Position", rightClimberPosition);
        SmartDashboard.putNumber("Left Climber Motor Voltage", leftClimber.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Right Climber Motor Voltage", rightClimber.getMotorVoltage().getValueAsDouble());

        SmartDashboard.putNumber("Left Encoder Start Position", leftEncoderStartPosition);
        SmartDashboard.putNumber("Right Encoder Start Position", rightEncoderStartPosition);
    }
}
