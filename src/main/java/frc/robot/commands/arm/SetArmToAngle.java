package frc.robot.commands.arm;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;

public class SetArmToAngle extends Command{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    private ProfiledPIDController controller;
    
    double err;
    double targetAngle;
    public SetArmToAngle(double targetAngle, Subsystem Arm) {
        controller = new ProfiledPIDController(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD, new Constraints(ArmConstants.kMaxSpeed, ArmConstants.kMaxAcceleration));
        
        this.targetAngle= targetAngle;
        controller.setTolerance(ArmConstants.kPosTolerance, ArmConstants.kSpeedTolerance);
    }

    @Override
    public void execute() {
        RobotContainer.arm.setVoltage(controller.calculate(RobotContainer.arm.getAngle(), targetAngle));
    }
    
    @Override
    public boolean isFinished(){
        return controller.atGoal();
    }

}
