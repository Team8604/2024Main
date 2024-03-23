// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.DriveConstants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** An example command that uses an example subsystem. */
public class DriveRobot extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */

    double multiplier, rotMultiplier, fwd, rot, rot1, rot2;

    public DriveRobot() {
      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(RobotContainer.drivetrain);
    }


    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      /* Get forward and rotational throttle from joystick */
      /* invert the joystick Y because forward Y is negative */
      //code from joystick for drivetrain
      fwd = -1 * MathUtil.applyDeadband(RobotContainer.m_driverController.getY(), 0.04);
      rot1 = -1 * MathUtil.applyDeadband(RobotContainer.m_driverController.getX(), 0.02);
      rot2 = -1 * MathUtil.applyDeadband(RobotContainer.m_driverController.getZ(), 0.07);

      //gets either the bigger of twist or sideways
      rot = Math.abs(rot1) >= Math.abs(rot2) ? rot1 : rot2;

      if (RobotContainer.joystickButton12.getAsBoolean()){
        rotMultiplier = DriveConstants.kMaxRotate;
      } else {
        rotMultiplier = DriveConstants.kNormalRotate;
      }

      multiplier = DriveConstants.kMaxSpeed + (RobotContainer.fastButton.getAsBoolean() ? DriveConstants.kSpeedIncrease : 0) + (RobotContainer.slowButton.getAsBoolean() ? DriveConstants.kSpeedDecrease : 0);
      SmartDashboard.putNumber("multiplier", multiplier);
      fwd = Math.pow(fwd, 3) * multiplier;
      rot = Math.pow(rot, 3) * rotMultiplier;
      SmartDashboard.putNumber("fwd", fwd);
      SmartDashboard.putNumber("rot", rot);

      // slider on flight stick for direction change
      if (RobotContainer.m_driverController.getRawAxis(3)>0){
        fwd *= -1;
      }

      /* Set output to control frames */
      RobotContainer.drivetrain.drive(fwd, rot);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}
}