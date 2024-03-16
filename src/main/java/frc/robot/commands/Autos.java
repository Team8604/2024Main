// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Autos {
  /** Example static factory for an autonomous command. */
  private Command shootNote = new SequentialCommandGroup(null);
  public static Command exampleAuto(Drivetrain subsystem) {
    return Commands.sequence(/*subsystem.exampleMethodCommand(), */new DriveRobot());
  }

  public Autos() {

  }

  public Command getAuto() {
    return null;
  }
}