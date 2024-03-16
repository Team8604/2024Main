// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;


public class Autos {

  private static SendableChooser<Command> m_chooser = new SendableChooser<Command>();

  private static Command shootNote = new ParallelDeadlineGroup(
    new WaitCommand(3),
    new RunShooter(),
    new SequentialCommandGroup(
      new WaitCommand(1),
      new RunIntake()
    )
  );

  public static void configureAutos() {
    m_chooser.setDefaultOption("Shoot and Stay", shootNote);
  }

  public static Command getAuto() {

    return m_chooser.getSelected();
  }
  
  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}