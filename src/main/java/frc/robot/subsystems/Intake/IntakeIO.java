package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public class IntakeInputs {
        public double IntakeMotorOutput;
    }

    public void updateInputs(IntakeInputs input);

    public void set(double speed);
}
