// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ArmConstants;
import frc.robot.RobotContainer;

import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class RunShooter extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private double angle;

  public RunShooter(double armAngle) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.shooter);
    angle = RobotContainer.arm.getAngle();

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    angle = RobotContainer.arm.getAngle();
    double error = angle - ArmConstants.kAmpAngle;
    if (Math.abs(error) < ArmConstants.kMaxError) {
        RobotContainer.shooter.setSpeed(ShooterConstants.kAmpSpeed);
    } else {
        RobotContainer.shooter.setSpeed(ShooterConstants.kMaxSpeed);
    }
    RobotContainer.shooter.running = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.shooter.setSpeed(0);
    RobotContainer.shooter.running = false;
  }
}