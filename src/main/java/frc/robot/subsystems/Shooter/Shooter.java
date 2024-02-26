package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter.ShooterIO;
import frc.robot.subsystems.Shooter.ShooterIONeo;
import frc.robot.Constants.ShooterConstants;


public class Shooter extends SubsystemBase  {
    private ShooterIO io;
    private ShooterInputsAutoLogged inputs;

    public Shooter() {
        io = new ShooterIONeo();
        inputs = new ShooterInputsAutoLogged();
        io.updateInputs(inputs);
    }
    //public void setShooterSpeed(){}

    /*public void setShooterSpeed(double speed){
        io.set(speed);
    }*/

    public void periodic() {
        io.updateInputs(inputs);

        Logger.processInputs("speed", inputs);
        
        //io.set(RobotContainer.m_operatorController.getLeftY() * 3);
        //io.set(1);
        //RobotContainer.operatorX.debounce(0.1, Debouncer.DebounceType.kBoth).whileTrue(Command setShooterSpeed());   
        //RobotContainer.operatorX.onTrue(Command set(ShooterConstants.kShooterSpeed));
        //System.out.println("---Shooter x b :"+RobotContainer.m_operatorController.a());
        /*if (RobotContainer.operatorX){
            setShooterSpeed(0.1);
        }*/

        /*if (RobotContainer.operatorX){

            //spin shooter motor foward for shooting
            setShooterSpeed(ShooterConstants.kShooterSpeed);
        } else if (RobotContainer.operatorY) {
            //spin shooter motor foward for scoring in amp
            setShooterSpeed(ShooterConstants.kShooterSpeed);
        } else if (RobotContainer.operatorRightBumper) {
            //spin shooter motor backwards (gets rid of jamed note)
            setShooterSpeed(-ShooterConstants.kAmpShooterSpeed);
        } else {
            //set shooter motor to stop
            setShooterSpeed(0);
        }*/
    }
}
