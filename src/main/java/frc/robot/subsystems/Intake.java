package frc.robot.subsystems;  

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.Rev2mDistanceSensor;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants.IntakeConstants;

public class Intake {
    //initialize motor
    private final CANSparkMax intakeMotor = new CANSparkMax(IntakeConstants.kIntake, MotorType.kBrushless);
    private final Rev2mDistanceSensor intakeSensor = new Rev2mDistanceSensor(null);

    public Intake() {
        intakeMotor.restoreFactoryDefaults();
    }

    public void setSpeed(double speed) {
        intakeMotor.set(MathUtil.clamp(speed, -1 * IntakeConstants.kMaxSpeed, IntakeConstants.kMaxSpeed));
    }


    public double getIntakeSensorDistance(){
        intakeSensor.setAutomaticMode(true);

        return intakeSensor.getRange();
    }
}
