// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class AutoDrive extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    private double fwd, rot, time;
    private Timer timer = new Timer();

    public AutoDrive() {
      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(RobotContainer.drivetrain);
      
    }

    public AutoDrive(double foward, double rotation, double duration){
        this.fwd = foward;
        this.rot = rotation;
        this.time = duration;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        timer.restart();
    }


    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        /* Set output to control frames */
        RobotContainer.drivetrain.drive(fwd, rot); //fwd, rot
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        RobotContainer.drivetrain.drive(0, 0);
    }

    @Override
    public boolean isFinished() {
      return timer.hasElapsed(time);
    }
}