package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public class ShooterInputs {
        public double ShooterMotorOutput;
    }

    public void updateInputs(ShooterInputs input);

    public void set(double speed);
}
