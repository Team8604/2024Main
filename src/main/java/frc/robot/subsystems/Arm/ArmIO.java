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

        public boolean rightFowardArmLimitSwitchEnabled;
        public boolean leftFowardArmLimitSwitchEnabled;
        public boolean rightBackwardArmLimitSwitchEnabled;
        public boolean leftBackwardArmLimitSwitchEnabled;
        
        public boolean rightFowardArmLimitSwitchPressed;
        public boolean leftFowardArmLimitSwitchPressed;
        public boolean rightBackwardArmLimitSwitchPressed;
        public boolean leftBackwardArmLimitSwitchPressed;
    }

    public void updateInputs(ArmInputs input);

    public void setVoltage(double voltage);
} 