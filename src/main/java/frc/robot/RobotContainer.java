// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.*;
import frc.robot.commands.*;
import frc.robot.commands.arm.*;
import frc.robot.commands.climber.Climb;
import frc.robot.subsystems.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static Drivetrain drivetrain = new Drivetrain();
  public static Intake intake = new Intake();
  public static Shooter shooter = new Shooter();
  public static Arm arm = new Arm();
  public static Climber climber = new Climber();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  public static CommandJoystick m_driverController = new CommandJoystick(OperatorConstants.kDriverControllerPort); 
  public static CommandXboxController m_operatorController = new CommandXboxController(OperatorConstants.kOperatorControllerPort);
  public static CommandXboxController m_operatorButtonBoard = new CommandXboxController(OperatorConstants.kOperatorButtonBoardPort);

  //driver buttons
  public static Trigger fastButton = m_driverController.button(1);
  public static Trigger slowButton = m_driverController.button(2);
  public static Trigger climbOverideButton = m_driverController.button(3); // determine overide button #
  public static Trigger joystickButton7 = m_driverController.button(7);
  public static Trigger joystickButton8 = m_driverController.button(8);
  public static Trigger joystickButton9 = m_driverController.button(9);
  public static Trigger joystickButton10 = m_driverController.button(10);
  public static Trigger joystickButton4 = m_driverController.button(4);


    //operator buttons
    public static Trigger operatorA = m_operatorController.b();
    public static Trigger operatorB = m_operatorController.x();
    public static Trigger operatorX = m_operatorController.a();
    public static Trigger operatorY = m_operatorController.y();
    public static Trigger operatorRightBumper = m_operatorController.rightBumper();

    //operator buttonboard buttons
    public static Trigger buttonBoardOne = m_operatorButtonBoard.button(1);
    public static Trigger buttonBoardTwo = m_operatorButtonBoard.button(2);
    public static Trigger buttonBoardThree = m_operatorButtonBoard.button(3);
    public static Trigger buttonBoardFour = m_operatorButtonBoard.button(4);
    public static Trigger buttonBoardFive = m_operatorButtonBoard.button(5);
    public static Trigger buttonBoardSix = m_operatorButtonBoard.button(6);
    public static Trigger buttonBoardSeven = m_operatorButtonBoard.button(7);
    public static Trigger buttonBoardEight = m_operatorButtonBoard.button(8);
    public static Trigger buttonBoardNine = m_operatorButtonBoard.button(9);
    public static Trigger buttonBoardTen = m_operatorButtonBoard.button(10);
    public static Trigger buttonBoardEleven = m_operatorButtonBoard.button(11);
    public static Trigger buttonBoardTwelve = m_operatorButtonBoard.button(12);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() { 
    // Configure the trigger bindings
    configureButtonBindings();

    // Set default commands
    CommandScheduler.getInstance().setDefaultCommand(RobotContainer.drivetrain, new DriveRobot());
    CommandScheduler.getInstance().setDefaultCommand(RobotContainer.arm, new RunArm());
    CommandScheduler.getInstance().setDefaultCommand(RobotContainer.climber, new Climb(joystickButton9, joystickButton7, joystickButton10, joystickButton8));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureButtonBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    //new Trigger(drivetrain::exampleCondition)
        //.onTrue(new DriveRobot(drivetrain));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    operatorA.whileTrue(new RunIntake());
    operatorX.whileTrue(new RunShooter(arm.getAngle()));
    operatorB.whileTrue(new BackOut());
    operatorY.onTrue(new SetArmToAngle(ArmConstants.kAmpAngle, arm));

    buttonBoardOne.whileTrue(new RunIntake());
    joystickButton4.whileTrue(new RunIntake());
    buttonBoardTwo.whileTrue(new RunShooter(ShooterConstants.kMaxSpeed));
    buttonBoardThree.whileTrue(new BackOut());
    buttonBoardFour.whileTrue(new RunShooter(ShooterConstants.kAmpSpeed));

    buttonBoardEight.whileTrue(new SetArmToAngle(ArmConstants.kShootPosition, arm));
    buttonBoardTen.whileTrue(new SetArmToAngle(ArmConstants.kIntakePosition, arm));
    buttonBoardEleven.whileTrue(new SetArmToAngle(ArmConstants.kAmpAngle, arm));
    buttonBoardTwelve.whileTrue(new SetArmToAngle(ArmConstants.kStartPosition, arm));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(drivetrain);//add intake,shooter,arm to this later
  }
}