package frc.robot.subsystems.Intake;  

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
    // Initialize motor
    private final CANSparkMax intakeMotor = new CANSparkMax(IntakeConstants.kIntake, MotorType.kBrushless);

    public Intake() {
        intakeMotor.restoreFactoryDefaults();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake speed", intakeMotor.get());
    }

    public void setSpeed(double speed) {
        intakeMotor.set(MathUtil.clamp(speed, -1 * IntakeConstants.kMaxSpeed, IntakeConstants.kMaxSpeed));
    }

    
}
