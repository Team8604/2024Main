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
  }

  public static class DriveConstants {
    // Identification of motors
    public static final String CANBUS_NAME = "rio";
    public static final int kRightLeader = 1;
    public static final int kRightFollower = 2;
    public static final int kLeftLeader = 3;
    public static final int kLeftFollower = 4;

    // Speed modifier
    public static final double kMaxSpeed = 0.3;
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
    public static final double kMaxSpeed = 0.3;
    public static final double kAmpSpeed = 0.1;
    public static final double kBackOut = -0.1; // TO BE DETERMINED
  }

  public static class ArmConstants {
    // CAN IDs
    public static final int kRightArm = 7;
    public static final int kLeftArm = 8;
    public static final int kArmEncoder = 9; // TO BE DETERMINED

    //limit switch type
    public static final SparkLimitSwitch.Type kArmLimitSwitchType = SparkLimitSwitch.Type.kNormallyOpen;

    // Speed modifier
    public static final double kMaxSpeed = 0.5;

    // Possible arm error
    public static final double kMaxError = 0.02; // TO BE DETERMINED

    // Arm positions
    public static final double kAmpAngle = 0.25; // TO BE DETERMINED
    public static final double kIntakePosition = 0; // TO BE DETERMINED
  }

  public static class ClimberConstants {
    // CAN IDs
    //public static final int kRightClimber = 9;
    //public static final int kLeftClimber = 10;

  }}
