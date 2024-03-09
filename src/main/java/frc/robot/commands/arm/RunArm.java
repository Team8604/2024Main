package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class RunArm extends Command {

    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    public static double adjustArm = 0;

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
        adjustArm = 0.3 * RobotContainer.m_operatorButtonBoard.getRawAxis(1);
        /*if (RobotContainer.arm.getAngle() <= 0.243 || RobotContainer.arm.getAngle() >= 4.5){
            adjustArm /= 2;
        }*/


        //deadband
        if (Math.abs(adjustArm) <= 0.2) {
            adjustArm=0;
        }

        if (RobotContainer.m_operatorButtonBoard.button(6).getAsBoolean()){
            adjustArm *= 0.5;
        }

        //sets speed to be at 0 if either limit switch is pressed and if arm is told to move in that direction
        if (RobotContainer.arm.rightFowardLimitSwitch.isPressed() != RobotContainer.arm.leftFowardLimitSwitch.isPressed() && adjustArm<0){
            RobotContainer.arm.setSpeed(0);
        } else if(RobotContainer.arm.rightBackwardLimitSwitch.isPressed() != RobotContainer.arm.leftBackwardLimitSwitch.isPressed() && adjustArm>0){
            RobotContainer.arm.setSpeed(0);
        } else if (Math.abs(adjustArm)<0.1){
            RobotContainer.arm.setSpeed(0);
        }else {
            RobotContainer.arm.setSpeed(-0.7*adjustArm);
        }      
    }
}