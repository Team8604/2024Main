package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake.IntakeIO;
import frc.robot.subsystems.Intake.IntakeIONeo;
import frc.robot.Constants.IntakeConstants;


public class Intake extends SubsystemBase  {
    private IntakeIO io;
    private IntakeInputsAutoLogged inputs;

    public Intake() {
        io = new IntakeIONeo();
        inputs = new IntakeInputsAutoLogged();
        io.updateInputs(inputs);
    }
    //public void setIntakeSpeed(){}

    public void setIntakeSpeed(double speed){
        io.set(speed);
    }

    public void periodic() {
        io.updateInputs(inputs);

        Logger.processInputs("speed", inputs);
        
        //io.set(RobotContainer.m_operatorController.getLeftY() * 3);
        
        //RobotContainer.operatorX.debounce(0.1, Debouncer.DebounceType.kBoth).whileTrue(Command setIntakeSpeed());   
        //RobotContainer.operatorX.onTrue(Command set(IntakeConstants.kIntakeSpeed));
        System.out.println("---Intake x b :"+RobotContainer.m_operatorController.a());
        //if (RobotContainer.operatorX){
          //  io.set(0.1);
        //}

        /*if (RobotContainer.operatorX){

            //spin Intake motor foward for shooting
            setIntakeSpeed(IntakeConstants.kIntakeSpeed);
        } else if (RobotContainer.operatorY) {
            //spin Intake motor foward for scoring in amp
            setIntakeSpeed(IntakeConstants.kIntakeSpeed);
        } else if (RobotContainer.operatorRightBumper) {
            //spin Intake motor backwards (gets rid of jamed note)
            setIntakeSpeed(-IntakeConstants.kAmpIntakeSpeed);
        } else {
            //set Intake motor to stop
            setIntakeSpeed(0);
        }*/
    }
}
