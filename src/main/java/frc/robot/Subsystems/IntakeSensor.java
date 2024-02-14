// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.IterativeRobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.*;
import com.revrobotics.Rev2mDistanceSensor.*;

/** Add your docs here. */
public class IntakeSensor {
    private static Rev2mDistanceSensor distOnboard;

    public static double getIntakeSensorDistance(){
        distOnboard.setAutomaticMode(true);

        return distOnboard.getRange();
    }
}