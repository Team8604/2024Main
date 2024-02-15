package frc.robot.subsystems;  

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants.IntakeConstants;

public class Intake {
    //initialize motor
    private final CANSparkMax intakeMotor = new CANSparkMax(IntakeConstants.kIntake, MotorType.kBrushless);

    public Intake() {
        intakeMotor.restoreFactoryDefaults();
    }

    public void setSpeed(double speed) {
        intakeMotor.set(MathUtil.clamp(speed, -1 * IntakeConstants.kMaxSpeed, IntakeConstants.kMaxSpeed));
    }
}
