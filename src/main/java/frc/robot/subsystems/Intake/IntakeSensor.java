package frc.robot.subsystems.Intake;

import com.revrobotics.Rev2mDistanceSensor;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.IterativeRobotBase;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.*;
import com.revrobotics.Rev2mDistanceSensor.Port;

public class IntakeSensor {
    private static Rev2mDistanceSensor distOnboard;
    public static double getIntakeSensorDistance(){
        //distOnboard.setAutomaticMode(true);

        //return distOnboard.getRange();
        distOnboard = new Rev2mDistanceSensor(Port.kOnboard);
        distOnboard.setAutomaticMode(true);

        Logger.recordOutput("Intake/Distance Distance", distOnboard.getRange());        

        return distOnboard.getRange();
    }
}
