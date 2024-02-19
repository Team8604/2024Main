package frc.robot.subsystems.Arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
    @AutoLog
    public class ArmInputs {
        public double rightMotorEncoderAngle;
        public double leftMotorEncoderAngle;
        public double absoluteEncoderAngle;

        public double motorRightAmpsOutput;
        public double motorLeftAmpsOutput;

        public boolean rightFowardArmLimitSwitch;
        public boolean leftFowardArmLimitSwitch;
        public boolean rightBackwardArmLimitSwitch;
        public boolean leftBackwardArmLimitSwitch;

    }

    public void updateInputs(ArmInputs input);

    public void setVoltage(double voltage);
} 