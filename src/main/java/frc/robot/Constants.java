// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.SparkLimitSwitch;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    // IDs of controllers
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final int kOperatorButtonBoardPort = 2;
  }

  public static class DriveConstants {
    // Identification of motors
    public static final String CANBUS_NAME = "rio";
    public static final int kRightLeader = 1;
    public static final int kRightFollower = 2;
    public static final int kLeftLeader = 3;
    public static final int kLeftFollower = 4;

    // Track width
    public static final double kTrackWidth = .555;
    public static final double kMotorMultiplier = 0.0565932293625;
    public static final double kMaxSpeedMetric = 25;
    
    // Speed modifier
    public static final double kMaxSpeed = 0.6;
    public static final double kSpeedIncrease = 0.4;
    public static final double kSpeedDecrease = -0.3;

    // PID
    public static final double kP = 1; //to be determined
    public static final double kI = 0; //to be determined
    public static final double kD = 0; //to be determined

  }

  public static class IntakeConstants {
    // CAN ID
    public static final int kIntake = 5;

    // Speed modifiers
    public static final double kMaxSpeed = 1;
    public static final double kIntakeSpeed = 0.25;
    public static final double kBackOut = -0.5;


    // Distance options
    public static final double kNoteDistance = 10; // TO BE DETERMINED
  }

  public static class ShooterConstants {
    // CAN ID
    public static final int kShooter = 6;

    // Speed modifiers
    public static final double kMaxSpeed = 1;
    public static final double kAmpSpeed = 0.1;
    public static final double kBackOut = -0.1; // TO BE DETERMINED
  }

  public static class ArmConstants {
    // CAN IDs
    public static final int kRightArm = 7;
    public static final int kLeftArm = 8;
    public static final int kArmEncoder = 9; // TO BE DETERMINED

    // limit switch type
    public static final SparkLimitSwitch.Type kArmLimitSwitchType = SparkLimitSwitch.Type.kNormallyOpen;

    // Possible arm error
    public static final double kMaxError = 0.12; 

    // Arm positions
    // Note - arm range on encoder is about 0.224-0.467
    public static final double kAmpAngle = 0.22;
    public static final double kIntakePosition = 0.467;
    public static final double kShootPosition = 0.447; 
    public static final double kTrapPosition = 0.25; // TO BE DETERMINED
    public static final double kClimbPosition = 0.25; // TO BE DETERMINED
    public static final double kStartPosition = 0.305; 
    public static final double kDistaneShootPosition = 0.4206; // about 1 robot distance away
    public static final double kDistancedddd = 0.405; //distance is about 1.5-2 robots

    //at 16 ft away and full speed 0.3708

    //used highest spot to still go under chain. Might combine with current shooting position
    public static final double kdriveTemp = 0.44; 

    // PID constants
    // Speed modifier
    public static final double kMaxSpeed = 500;

    public static final double kMaxAcceleration = 200; 

    public static final double kP = 50;
    public static final double kI = 3;
    public static final double kD = 0.9;
    public static final double kPosTolerance = 0.004;
    public static final double kSpeedTolerance = 0.08;  
  }

  public static class NavXConstants {
    //PID constants
    public static final double kP = 0.1;
    public static final double kI = 0.1;
    public static final double kD = 0.1;
    public static final double kF = 0.1;

    public static final double kToleranceDegrees = 0.1;    
    
    public static final double kSpeedTolerance = 0.1;
  }

  public static class ClimberConstants {
    // CAN IDs
    public static final String CANBUS_NAME = "rio";
    public static final int kRightClimber = 9;
    public static final int kLeftClimber = 10;

    public static final double kManualVolts = 3;

    //positions
    public static final double downPosition = -1; // to be determined
    public static final double upPosition = -358; // to be determined

    public static final double toleranceToStartSlow = 20;

    public static final double slowModeSpeed = 0.5; // 50 percentage of normal
  }
}

