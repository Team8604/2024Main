package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.RobotContainer;

public class RunRightClimber extends Command {
    public double voltage;

    public RunRightClimber(double v) {
        // Use addRequirements() here
        addRequirements(RobotContainer.climber);
        this.voltage=v;
    }

    @Override
    public void execute(){
        RobotContainer.climber.setRightVoltage(voltage);
    }
}
