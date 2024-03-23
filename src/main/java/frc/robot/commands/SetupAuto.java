// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import com.pathplanner.lib.commands.PathPlannerAuto;

import frc.robot.Constants.ArmConstants;
import frc.robot.commands.arm.SetArmToAngle;

public class SetupAuto {

  private static SendableChooser<Integer> m_AutoType = new SendableChooser<Integer>();
  private static SendableChooser<Integer> m_StartPosition = new SendableChooser<Integer>();
  private static SendableChooser<Integer> m_PathOrTime = new SendableChooser<Integer>();

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

  private static Command SaMOAmpBlue = new SequentialCommandGroup(
    new AutoDriveRobot(0.3, 0.3, 1)
  );
  
  private static Command SaMOAmpRed = new SequentialCommandGroup(
    new AutoDriveRobot(0.3, -0.3, 1)
  );

  private static Command SaMOMid = new SequentialCommandGroup(
    new AutoDriveRobot(0.1, 0, 3)
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

    m_PathOrTime.setDefaultOption("Primitive Path", 100);
    m_PathOrTime.addOption("PathPlanner Path", 200);
    SmartDashboard.putData(m_PathOrTime);
  } 

  public static Command getAuto() {
    int autoValue = m_PathOrTime.getSelected() + m_AutoType.getSelected() + m_StartPosition.getSelected();
    boolean isRed = DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() == DriverStation.Alliance.Red : false;

    m_AutoType.close();
    m_StartPosition.close();
    
    Command chosenAuto = new WaitCommand(5);

    switch (autoValue) {
      case 111, 112, 113, 211, 212, 213:
        chosenAuto = shootNote;
        break;
      case 132:
        chosenAuto = SaMOMid;
        break;
      case 133:
        chosenAuto = isRed ? SaMOAmpRed : SaMOAmpBlue;
        break;
      case 231:
        chosenAuto = new PathPlannerAuto("SaMO Source");
        break;
      case 232:
        chosenAuto = new PathPlannerAuto("SaMO Middle");
        break;
      case 233:
        chosenAuto = new PathPlannerAuto("SaMO Amp");
        break;
    }
    return chosenAuto;
  }
  
  private SetupAuto() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}