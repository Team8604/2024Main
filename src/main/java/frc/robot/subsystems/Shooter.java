package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase{
    //initialize motor
    private final CANSparkMax shooterMotor = new CANSparkMax(ShooterConstants.kShooter, MotorType.kBrushless);

    public Shooter() {
        shooterMotor.restoreFactoryDefaults();
    }

    @Override
    public void periodic() {}

    public void setSpeed(double speed) {
        shooterMotor.set(ShooterConstants.kMaxSpeed * MathUtil.clamp(speed, -1, 1));
    }
}

