// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.IntakeConstants;
import frc.robot.RobotContainer;

import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class RunIntake extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  public RunIntake() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.intake);
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  @Override
  public void execute(){
    if (RobotContainer.buttonBoardOne.getAsBoolean() && RobotContainer.buttonBoardTwo.getAsBoolean() || RobotContainer.buttonBoardOne.getAsBoolean() && RobotContainer.buttonBoardFour.getAsBoolean()){
      RobotContainer.intake.setSpeed(IntakeConstants.kMaxSpeed);
    } else if (RobotContainer.buttonBoardOne.getAsBoolean()){
      RobotContainer.intake.setSpeed(IntakeConstants.kIntakeSpeed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.intake.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
     if (!RobotContainer.intake.isShoot() && RobotContainer.intake.isNote()) {
      return true;
     }
     
    return false;
  }
}