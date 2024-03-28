// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.Constants.ArmConstants;
import frc.robot.commands.arm.SetArmToAngle;

public class Autos {

  private static SendableChooser<Integer> m_AutoType = new SendableChooser<Integer>();
  //private static SendableChooser<Integer> m_StartPosition = new SendableChooser<Integer>();

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
  
  public static Command moveOut = new SequentialCommandGroup(
    new AutoEncoderDrive(20,20)
  );

  public static Command shootmoveFromSide = new SequentialCommandGroup(
    new SetArmToAngle(ArmConstants.kShootPosition),
    new ParallelDeadlineGroup(
      new WaitCommand(2),
      new RunShooter(),
      new SequentialCommandGroup(
        new WaitCommand(1),
        new RunIntake(),
        new WaitCommand(1)

      )   
    ),
    new AutoEncoderDrive(20,20)

  );
  public static Command TwoNoteAuto = new SequentialCommandGroup(
    new SetArmToAngle(ArmConstants.kShootPosition),
    new ParallelDeadlineGroup(
      new WaitCommand(2),
      new RunShooter(),
      new SequentialCommandGroup(
        new WaitCommand(1),
        new RunIntake(),
        new WaitCommand(1)

      )   
    ),
    new SetArmToAngle(ArmConstants.kIntakePosition),
    new ParallelDeadlineGroup(
      new RunIntake(),
      new WaitCommand(2),
      new AutoEncoderDrive(20,20)
      
    ),
    new ParallelCommandGroup(
      new SetArmToAngle(ArmConstants.kAutoShootPos),
      new RunShooter(),
      
      new WaitCommand(2),
      new RunIntake()
    )
  );

  public static void configureAutos() {
    /*m_StartPosition.setDefaultOption("Source Side", 1);
    m_StartPosition.addOption("Middle", 2);
    m_StartPosition.addOption("Amp Side", 3);
    SmartDashboard.putData(m_StartPosition);
    */

    m_AutoType.setDefaultOption("Absolutely Nothing", 10);
    m_AutoType.addOption("Shoot and Stay", 20);
    m_AutoType.addOption("moveOut", 30);
    m_AutoType.addOption("shootmoveFromSide", 40);
    m_AutoType.addOption("TwoNoteAuto", 50);

    SmartDashboard.putData(m_AutoType);
  } 

  public static Command getAuto() {
    int autoValue = m_AutoType.getSelected();// + m_StartPosition.getSelected();
    //boolean isRed = DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() == DriverStation.Alliance.Red : false;
    
    Command chosenAuto = shootNoteAndStay;
    System.out.println("Auto value" + autoValue);
    System.out.println("Chosen auto" + chosenAuto);
    switch (autoValue) {
      case 10:
        chosenAuto = nothing;
        break;
      case 20:
        chosenAuto = shootNoteAndStay;
        break;
      case 30:
        chosenAuto = moveOut;
        break;

      case 40:
        chosenAuto = shootmoveFromSide;
        break;
      case 50:
      chosenAuto = TwoNoteAuto;

    }
    return chosenAuto;
  }
  
  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}