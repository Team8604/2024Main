// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.IterativeRobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.*;
import com.revrobotics.Rev2mDistanceSensor.Port;
/** Add your docs here. */
public class IntakeSensor {
    private static Rev2mDistanceSensor distOnboard;
    private static Rev2mDistanceSensor distMXP;

    public static double getIntakeSensorDistance(){
        //distOnboard.setAutomaticMode(true);

        //return distOnboard.getRange();
        distOnboard = new Rev2mDistanceSensor(Port.kOnboard);
       // distMXP = new Rev2mDistanceSensor(Port.kOnboard);
        distOnboard.setAutomaticMode(true);

        //System.out.println("distOnboard"+distOnboard.getRange());
        //System.out.println("distMXP"+distMXP.getRange());
        SmartDashboard.putNumber("Intake sensor distOnboard",distOnboard.getRange());
        //SmartDashboard.putNumber("Intake sensor distMXP",distMXP.getRange());

        return distOnboard.getRange();
    }
}
