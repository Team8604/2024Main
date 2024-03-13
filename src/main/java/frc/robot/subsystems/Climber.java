
package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase{
    // Initialize motors
    private final TalonFX rightClimber = new TalonFX(ClimberConstants.kRightClimber, ClimberConstants.CANBUS_NAME);
    private final TalonFX leftClimber = new TalonFX(ClimberConstants.kLeftClimber, ClimberConstants.CANBUS_NAME);

    public Climber(){
        rightClimber.setSafetyEnabled(true);
        leftClimber.setSafetyEnabled(true);
    }

    public void setLeftVoltage(double volts){
        leftClimber.setVoltage(MathUtil.clamp(volts, -12, 12));
    }
    
    public void setRightVoltage(double volts){
        rightClimber.setVoltage(MathUtil.clamp(volts, -12, 12));
    }
    
    @Override
    public void periodic(){
        SmartDashboard.putNumber("Left Climber Motor", leftClimber.get());
        SmartDashboard.putNumber("Right Climber Motor", rightClimber.get());
    }
}
