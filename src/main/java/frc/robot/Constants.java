// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


/** Add your docs here. */
public final class Constants {
    
    //PID numbers *TO BE MARKED FINAL ONCE TESTED* for Intake
    public static double kIntakeP = 0.1;
    public static double kIntakeI = 1e-4;
    public static double kIntakeD = 1;
    public static double kIntakeIz = 0;
    public static double kIntakeFF = 0;
    public static double kIntakeMaxOutput = 1;
    public static double kIntakeMinOutput = -1;

    //PID numbers *TO BE MARKED FINAL ONCE TESTED* for Shooter
    public static double kShooterP = 0.1;
    public static double kShooterI = 1e-4;
    public static double kShooterD = 1;
    public static double kShooterIz = 0;
    public static double kShooterFF = 0;
    public static double kShooterMaxOutput = 1;
    public static double kShooterMinOutput = -1;
}
