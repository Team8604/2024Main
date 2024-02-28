package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class RunArm extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    public RunArm() {
        // Use addRequirements() here
        addRequirements(RobotContainer.arm);
    }

    
    // Called when the command is initially scheduled
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled
    @Override
    public void execute() {
        double adjustArm = Math.pow(RobotContainer.m_operatorController.getLeftY(), 3);

        RobotContainer.arm.setSpeed(-1*adjustArm);
    }
    
}
