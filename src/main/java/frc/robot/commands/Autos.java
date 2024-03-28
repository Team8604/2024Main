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

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.arm.SetArmToAngle;

public class Autos {

  private static SendableChooser<Integer> m_AutoType = new SendableChooser<Integer>();
  private static SendableChooser<Integer> m_StartPosition = new SendableChooser<Integer>();

    public static Command nothing = new SequentialCommandGroup(
      new WaitCommand(5)
    );

  public static Command shootNoteAndStay = new SequentialCommandGroup(
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

  public static Command shootNoteAndmoveOut = new SequentialCommandGroup(
    new SetArmToAngle(ArmConstants.kShootPosition),
    new ParallelDeadlineGroup(
      new WaitCommand(2),
      new RunShooter(),
      new SequentialCommandGroup(
        new WaitCommand(1),
        new RunIntake(),
        new AutoDrive(AutoConstants.kDrivePower, 0, AutoConstants.kMoveDrivetime)
      )
    )  
  );

  public static Command moveOut = new SequentialCommandGroup(
    new AutoDrive(AutoConstants.kDrivePower, 0, AutoConstants.kMoveOutDrivetime)
  );
  public static Command moveOutCurve = new SequentialCommandGroup(
    new AutoDrive(AutoConstants.kDrivePower, 0.1, AutoConstants.kMoveOutDrivetime)
  );
  public static Command movePos = new SequentialCommandGroup(
    new AutoEncoderDrive(20,20)
  );

  public static Command shootmovePos = new SequentialCommandGroup(

    new SetArmToAngle(ArmConstants.kShootPosition),
    new ParallelDeadlineGroup(
      new WaitCommand(2),
      new RunShooter(),
      new SequentialCommandGroup(
        new WaitCommand(1),
        new RunIntake()
      )
    ),
    new AutoEncoderDrive(20,20)
  );

  /*public static Command twoNoteAuto = new SequentialCommandGroup(

    new SetArmToAngle(ArmConstants.kShootPosition),
    new ParallelDeadlineGroup(
      new WaitCommand(2),
      new RunShooter(),
      new SequentialCommandGroup(
        new WaitCommand(1),
        new RunIntake()
      ),
      new SequentialCommandGroup(        
        new SetArmToAngle(ArmConstants.kAutoShootPos),
        new RunIntake(),
        new AutoEncoderDrive(20,20),
        new RunShooter(),
        new WaitCommand(2),
        new RunIntake()
      )
    )
  );*/

  public static void configureAutos() {
    m_StartPosition.setDefaultOption("Source Side", 1);
    m_StartPosition.addOption("Middle", 2);
    m_StartPosition.addOption("Amp Side", 3);
    SmartDashboard.putData(m_StartPosition);

    m_AutoType.setDefaultOption("Absolutely Nothing", 10);
    m_AutoType.addOption("Shoot and Stay", 20);
    m_AutoType.addOption("Shoot and Move Out", 30);
    //m_AutoType.addOption("Move Out", 40);
    //m_AutoType.addOption("moveOutCurve", 50);
    m_AutoType.addOption("movePos", 60);
    m_AutoType.addOption("shootmovePos", 70);

    SmartDashboard.putData(m_AutoType);
  } 

  public static Command getAuto() {
    int autoValue = m_AutoType.getSelected() + m_StartPosition.getSelected();
    boolean isRed = DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() == DriverStation.Alliance.Red : false;
    
    Command chosenAuto = nothing;

    switch (autoValue) {
      case 11, 12, 13:
        chosenAuto = nothing;
        break;
      case 21, 22, 23:
        chosenAuto = shootNoteAndStay;
        break;
      case 31, 32, 33:
        chosenAuto = shootNoteAndmoveOut;
      case 41, 42, 43:
        chosenAuto = moveOut;
      case 51, 52, 53:
        chosenAuto = moveOutCurve;
      case 61,62,63:
        chosenAuto = movePos;
      case 71,72,73:
        chosenAuto = shootmovePos;
    }
    return chosenAuto;
  }
  
  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}