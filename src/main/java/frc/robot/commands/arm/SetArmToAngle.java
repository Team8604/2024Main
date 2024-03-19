package frc.robot.commands.arm;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;

public class SetArmToAngle extends Command{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    private ProfiledPIDController controller;
    
    double err;
    double targetAngle;

    public SetArmToAngle(double targetAngle) {
        controller = new ProfiledPIDController(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD, new Constraints(ArmConstants.kMaxSpeed, ArmConstants.kMaxAcceleration));
        
        this.targetAngle= targetAngle;
        controller.setTolerance(ArmConstants.kPosTolerance, ArmConstants.kSpeedTolerance);

        addRequirements(RobotContainer.arm);
    }

    @Override
    public void initialize() {
        controller.reset(RobotContainer.arm.getAngle());
    }

    @Override
    public void execute() {
        RobotContainer.arm.setVoltage(controller.calculate(RobotContainer.arm.getAngle(), targetAngle));
    }
    
    @Override
    public void end(boolean interrupted) {
        RobotContainer.arm.setVoltage(0);
    }
    
    @Override
    public boolean isFinished(){
        return controller.atGoal();
    }

}
