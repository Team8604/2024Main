// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.DriveConstants;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class DriveRobot extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public DriveRobot() {
      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(RobotContainer.drivetrain);
    }


    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      /* Get forward and rotational throttle from joystick */
      /* invert the joystick Y because forward Y is negative */
      //code from joystick for drivetrain
      double fwd = -1 * Math.pow(RobotContainer.m_driverController.getY(), 3);
      double rot;
      double rot1 = Math.pow(RobotContainer.m_driverController.getX(), 3);
      double rot2 = Math.pow(RobotContainer.m_driverController.getZ(), 3);

      //gets either the bigger of twist or sideways
      if (Math.abs(rot1) >= Math.abs(rot2)){
        rot = rot1;
      } else {
        rot = rot2;
      }
      //dead zone
      if (rot < 0.08 && rot > -0.08){
        rot = 0;
      }
      if (fwd < 0.08 && fwd > -0.08){
        fwd = 0;
      }

      double multiplier = DriveConstants.kMaxSpeed + (RobotContainer.fastButton.getAsBoolean() ? DriveConstants.kSpeedIncrease : 0) + (RobotContainer.slowButton.getAsBoolean() ? DriveConstants.kSpeedDecrease : 0);
      fwd *= multiplier;
      rot *= multiplier;

      /* Set output to control frames */
      RobotContainer.drivetrain.drive(fwd + rot , fwd - rot);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}
}