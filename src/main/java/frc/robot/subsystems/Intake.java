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
    private final Rev2mDistanceSensor intakeSensorOnBoard = new Rev2mDistanceSensor(Port.kOnboard);
    private final Rev2mDistanceSensor intakeSensorMXP = new Rev2mDistanceSensor(Port.kMXP);

    public Intake() {
        intakeMotor.restoreFactoryDefaults();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        
        SmartDashboard.putNumber("Distance Sensor", getIntakeSensorDistance());
        //RobotContainer.operatorA.onTrue(new RunIntake());
    }

    public void setSpeed(double speed) {
        intakeMotor.set(IntakeConstants.kMaxSpeed * MathUtil.clamp(speed, -1, 1));
    }


    public double getIntakeSensorDistance(){
        intakeSensorOnBoard.setAutomaticMode(true);

        return intakeSensorMXP.getRange();
    }

    public boolean isNote() {
        if (getIntakeSensorDistance() <= IntakeConstants.kNoteDistance) {
            SmartDashboard.putBoolean("Note Collected", isNote());
            return true;
        }

        return false;
    }

}