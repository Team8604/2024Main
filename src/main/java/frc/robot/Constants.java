// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


/** Add your docs here. */
public final class Constants {
    
    //CAN IDs for devices
    public static final int kRightLeader = 1;
    public static final int kRightFollower = 2;
    public static final int kLeftLeader = 3;
    public static final int kLeftFollower = 4;
    public static final int kIntakeMotor = 5;
    public static final int kShooterMotor = 6;

    //PID numbers *TO BE MARKED FINAL ONCE TESTED* for Intake
    public static double kIntakeP = 0.0010;
    public static double kIntakeI = 0.00000;
    public static double kIntakeD = 0;
    public static double kIntakeIz = 0;
    public static double kIntakeFF = 0;
    public static double kIntakeMaxOutput = 1;
    public static double kIntakeMinOutput = -1;

    //PID numbers *TO BE MARKED FINAL ONCE TESTED* for Shooter
    public static double kShooterP = 0.00;
    public static double kShooterI = 0.00000;
    public static double kShooterD = 0;
    public static double kShooterIz = 0;
    public static double kShooterFF = 0;
    public static double kShooterMaxOutput = 1;
    public static double kShooterMinOutput = -1;
}
