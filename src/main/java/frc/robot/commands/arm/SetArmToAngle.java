package frc.robot.commands.arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;

public class SetArmToAngle extends PIDCommand{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    
    double err;

    public SetArmToAngle(double targetAngle) {
        super(
            new PIDController(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD),
            () -> { return RobotContainer.arm.getAngle(); },
            targetAngle,
            (double speed) -> { RobotContainer.arm.setSpeed(speed +  Math.abs(targetAngle - RobotContainer.arm.getAngle()) * 0.4); },
            RobotContainer.arm
        );

        getController().setTolerance(ArmConstants.kPosTolerance, ArmConstants.kSpeedTolerance);
    }
    
    @Override
    public boolean isFinished(){
        return getController().atSetpoint();
    }

}
