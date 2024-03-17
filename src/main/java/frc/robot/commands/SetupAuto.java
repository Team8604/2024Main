// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import frc.robot.Constants.ArmConstants;
import frc.robot.commands.arm.SetArmToAngle;


public class SetupAuto {

  private static SendableChooser<Integer> m_AutoType = new SendableChooser<Integer>();
  private static SendableChooser<Integer> m_StartPosition = new SendableChooser<Integer>();

  public static Command shootNote = new SequentialCommandGroup(
    new SetArmToAngle(ArmConstants.kShootPosition),
    new ParallelDeadlineGroup(
      new WaitCommand(2),
      new RunShooter(),
      new SequentialCommandGroup(
        new WaitCommand(1),
        new RunIntake()
      )
    )
  );

  public static void configureAutos() {
    m_StartPosition.setDefaultOption("Source Side", 1);
    m_StartPosition.addOption("Middle", 2);
    m_StartPosition.addOption("Amp Side", 3);
    SmartDashboard.putData(m_StartPosition);

    m_AutoType.setDefaultOption("Shoot and Stay", 10);
    m_AutoType.addOption("Absolutely Nothing", 20);
    m_AutoType.addOption("Shoot and Move Out", 30);
    SmartDashboard.putData(m_AutoType);
  } 

  public static Command getAuto() {
    int autoValue = m_AutoType.getSelected() + m_StartPosition.getSelected();

    m_AutoType.close();
    m_StartPosition.close();
    
    Command chosenAuto = new WaitCommand(5);

    switch (autoValue) {
      case 11:
        chosenAuto = shootNote;
        break;
      case 12:
        chosenAuto = shootNote;
        break;
      case 13:
        chosenAuto = shootNote;
        break;
      case 31:
        chosenAuto = new PathPlannerAuto("SaMO Source");
        break;
      case 32:
        chosenAuto = new PathPlannerAuto("SaMO Middle");
        break;
      case 33:
        chosenAuto = new PathPlannerAuto("SaMO Amp");
        break;
    }
    return chosenAuto;
  }
  
  private SetupAuto() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}