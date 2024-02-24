package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import frc.robot.Constants.ShooterConstants;

public class ShooterIONeo implements ShooterIO{
    private final CANSparkMax shooterMotor = new CANSparkMax(ShooterConstants.kShooter, MotorType.kBrushless);

    public ShooterIONeo(){
        shooterMotor.restoreFactoryDefaults();

    }
    public void updateInputs(ShooterInputs inputs) {
        inputs.ShooterMotorOutput = shooterMotor.get();

    }

    public void set(double speed) {
        Logger.recordOutput("Arm/OutputVoltage", speed);
        shooterMotor.set(speed);
    }
}
