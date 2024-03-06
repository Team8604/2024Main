package frc.robot.commands.navX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.NavXConstants;
import frc.robot.RobotContainer;

public class TurnRobotToAngle extends PIDCommand{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    public TurnRobotToAngle(double targetAngle) {
        super(
            new PIDController(NavXConstants.kP, NavXConstants.kI, NavXConstants.kD),
            () -> { return RobotContainer.drivetrain.xAxis; },
            targetAngle,
            (double speed) -> { RobotContainer.drivetrain.setSpeed(0, speed); },
            RobotContainer.drivetrain
        );

        getController().setTolerance(NavXConstants.kToleranceDegrees, NavXConstants.kSpeedTolerance);
    }
    
    @Override
    public boolean isFinished(){
        return getController().atSetpoint();
    }

}
