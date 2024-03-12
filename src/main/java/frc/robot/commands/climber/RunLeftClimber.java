package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.RobotContainer;

public class RunLeftClimber extends Command {
    public double voltage;

    public RunLeftClimber(double v) {
        // Use addRequirements() here
        addRequirements(RobotContainer.climber);
        this.voltage=v;
    }

    @Override
    public void execute(){
        RobotContainer.climber.setLeftVoltage(voltage);
    }
}
