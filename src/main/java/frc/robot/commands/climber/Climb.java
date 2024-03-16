// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import frc.robot.Constants.ClimberConstants;
import frc.robot.RobotContainer;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** An example command that uses an example subsystem. */
public class Climb extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  DoubleSupplier leftVolts, rightVolts;

  public Climb(Trigger leftUp, Trigger leftDown, Trigger rightUp, Trigger rightDown) {
    addRequirements(RobotContainer.climber);
    //rightVolts = () -> { return ((rightUp.getAsBoolean() ? ClimberConstants.kManualVolts : 0) - (rightDown.getAsBoolean() ? ClimberConstants.kManualVolts : 0)); };
    leftVolts = () -> { return ((leftUp.getAsBoolean() ? ClimberConstants.kManualVolts : 0) - (leftDown.getAsBoolean() ? ClimberConstants.kManualVolts : 0)); };
}

  // Called when the command is initially scheduled.
  @Override
  public void execute() {
    //RobotContainer.climber.setRightVoltage(rightVolts.getAsDouble());
    RobotContainer.climber.setLeftVoltage(leftVolts.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.climber.setRightVoltage(0);
    RobotContainer.climber.setLeftVoltage(0);
  }
}