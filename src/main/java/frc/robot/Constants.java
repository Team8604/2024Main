// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


/** Add your docs here. */
public final class Constants {
    
    //Motor speeds
    public static final double kShooterSpeed = 1;
    public static final double kAmpShooterSpeed = 0.1;
    public static final double kIntakeSpeed = 0.25;
    public static final double kMaxIntakeSpeed = 1;
    public static final double kDrivetrainSpeed = 0.25;
    public static final double kArmMaxSpeed = 0.1;

    //Arm PID coefficients
    public static final double kArmP = 0.5;
    public static final double kArmI = 0.05;
    public static final double kArmD = 0.05;
    
    //CAN IDs for devices
    public static final int kRightLeader = 1;
    public static final int kRightFollower = 2;
    public static final int kLeftLeader = 3;
    public static final int kLeftFollower = 4;
    public static final int kIntakeMotor = 5;
    public static final int kShooterMotor = 6;
    public static final int kRightArm = 7;
    public static final int kLeftArm = 8;
    //public static final int kRightClimber = 9;
    //public static final int kRightClimber = 10;

    public static final int kArmEncoder = 0;

}
