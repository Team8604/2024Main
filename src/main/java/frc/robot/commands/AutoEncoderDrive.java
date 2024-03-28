// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class AutoEncoderDrive extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    private double leftclicks, rightclicks, leftStart, rightStart;

    public AutoEncoderDrive() {
      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(RobotContainer.drivetrain);
      
    }

    public AutoEncoderDrive(double leftclicks, double rightclicks){
        this.leftclicks = leftclicks;
        this.rightclicks = rightclicks;
    }


    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        leftStart = RobotContainer.drivetrain.getLeftEncoder();
        rightStart = RobotContainer.drivetrain.getRightEncoder();
    }


    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        /* Set output to control frames */
        System.out.println("left Start" + leftStart);
        System.out.println("right start" + rightStart);
        
        if (RobotContainer.drivetrain.getLeftEncoder() <= leftStart+leftclicks && RobotContainer.drivetrain.getRightEncoder() <= rightStart+rightclicks ) {
            RobotContainer.drivetrain.drive(0.2, 0);
        } /*else if (Drivetrain.getRightEncoder() <= rightStart+rightclicks+20 ) {
            RobotContainer.drivetrain.drive(0, 0.2);

            //RobotContainer.drivetrain.runLeftMotor(1);
            //RobotContainer.drivetrain.runRightMotor(0);
        } else if (Drivetrain.getRightEncoder() <= rightStart+rightclicks+20 ) {
            RobotContainer.drivetrain.runRightMotor(1);
            RobotContainer.drivetrain.runLeftMotor(0);
        } */
        /*else if (RobotContainer.drivetrain.getLeftEncoder() <= leftStart+leftclicks) {
            RobotContainer.drivetrain.setMotorVoltages(2, 0);
        } else if (RobotContainer.drivetrain.getRightEncoder() <= leftStart+leftclicks){
            RobotContainer.drivetrain.setMotorVoltages(0, 2);
        }*/else {
            RobotContainer.drivetrain.drive(0, 0);
            System.out.println("end");
        }
    }
}