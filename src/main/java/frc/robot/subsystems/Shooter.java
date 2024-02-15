package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import frc.robot.Constants.ShooterConstants;

public class Shooter {
    //initialize motor
    private final CANSparkMax shooterMotor = new CANSparkMax(ShooterConstants.kShooter, MotorType.kBrushless);
}
