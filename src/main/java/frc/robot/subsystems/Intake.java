package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

public class Intake {
    //initialize motor
    private final CANSparkMax intakeMotor = new CANSparkMax(IntakeConstants.kIntake, MotorType.kBrushless);
}
