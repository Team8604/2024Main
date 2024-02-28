package frc.robot.subsystems;  

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase{
    //initialize motor
    private final CANSparkMax intakeMotor = new CANSparkMax(IntakeConstants.kIntake, MotorType.kBrushless);
    private final Rev2mDistanceSensor intakeSensor = new Rev2mDistanceSensor(Port.kOnboard);

    public Intake() {
        intakeMotor.restoreFactoryDefaults();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Distance Sensor", getIntakeSensorDistance());
    }

    public void setSpeed(double speed) {
        intakeMotor.set(IntakeConstants.kMaxSpeed * MathUtil.clamp(speed, -1, 1));
    }


    public double getIntakeSensorDistance(){
        intakeSensor.setAutomaticMode(true);

        return intakeSensor.getRange();
    }

    public boolean isNote() {
        if (intakeSensor.GetRange() < IntakeConstants.kNoteDistance) {
            return true;
        }

        return false;
    }
}
