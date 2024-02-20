package frc.robot.subsystems.Intake;

import com.revrobotics.Rev2mDistanceSensor;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.proto.System;
import edu.wpi.first.wpilibj.IterativeRobotBase;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.*;
import com.revrobotics.Rev2mDistanceSensor.Port;

public class IntakeSensor {
    public static Rev2mDistanceSensor distOnboard;
    public static Rev2mDistanceSensor distMXP;

    public static double returnIntakeSensorDistance(){
        distOnboard = new Rev2mDistanceSensor(Port.kOnboard);
        distMXP = new Rev2mDistanceSensor(Port.kMXP);

        if(distOnboard.isRangeValid()) {
            Logger.recordOutput("Intake/Distance Distance", distOnboard.getRange());        
            SmartDashboard.putNumber("Range Onboard", distOnboard.getRange());
            SmartDashboard.putNumber("Timestamp Onboard", distOnboard.getTimestamp());
        }
        if(distMXP.isRangeValid()) {
            Logger.recordOutput("Intake/Distance Distance", distMXP.getRange());        
            SmartDashboard.putNumber("Range MXP", distMXP.getRange());
            SmartDashboard.putNumber("Timestamp MXP", distMXP.getTimestamp());
          }

        return distOnboard.getRange();
    }
}
