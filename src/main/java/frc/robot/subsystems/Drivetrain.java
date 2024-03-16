// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.kauailabs.navx.frc.AHRS;

import frc.robot.Constants.DriveConstants;

public class Drivetrain extends SubsystemBase {

  //initialize motors
  private final TalonFX leftLeader = new TalonFX(DriveConstants.kLeftLeader, DriveConstants.CANBUS_NAME);
  private final TalonFX leftFollower = new TalonFX(DriveConstants.kLeftFollower, DriveConstants.CANBUS_NAME);
  private final TalonFX rightLeader = new TalonFX(DriveConstants.kRightLeader, DriveConstants.CANBUS_NAME);
  private final TalonFX rightFollower = new TalonFX(DriveConstants.kRightFollower, DriveConstants.CANBUS_NAME);

  private final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(DriveConstants.kTrackWidth);
  private final DifferentialDriveOdometry m_odometry;

  private final PIDController m_leftPIDController = new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD);
  private final PIDController m_rightPIDController = new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD);

  double leftForward, rightForward, leftOut, rightOut;
  double leftPos, leftVelocity, rightPos, rightVelocity;

  
  //navx values
  private double xAxis, yAxis, zAxis; //Roll, Pitch, Yaw
  private AHRS ahrs;

  public Drivetrain() {
    rightLeader.setInverted(true);

    /* Set up followers to follow leaders */
    leftFollower.setControl(new Follower(leftLeader.getDeviceID(), false));
    rightFollower.setControl(new Follower(rightLeader.getDeviceID(), false));
        
    leftLeader.setSafetyEnabled(false);
    rightLeader.setSafetyEnabled(false);

    //add limelight generated default pose later
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(yAxis), leftPos, rightPos);

    //set navx
    ahrs = new AHRS();
    ahrs.enableLogging(true);
  }

  public void setSpeeds(DifferentialDriveWheelSpeeds speed){
    leftForward = m_leftPIDController.calculate(speed.leftMetersPerSecond);
    rightForward = m_rightPIDController.calculate(speed.rightMetersPerSecond);

    leftOut = m_leftPIDController.calculate(leftVelocity, speed.leftMetersPerSecond); //add encoder value
    rightOut = m_rightPIDController.calculate(rightVelocity, speed.rightMetersPerSecond);  //add encoder value 

    leftLeader.setVoltage(MathUtil.clamp(leftOut + leftForward, -12, 12));
    rightLeader.setVoltage(MathUtil.clamp(rightOut + rightForward, -12, 12));
  }

  /**
  * Drives the robot with the given linear velocity and angular velocity.
  *
  * @param xSpeed Linear velocity in m/s.
  * @param rot Angular velocity in rad/s.
  */
  public void drive(double xSpeed, double rot) {  
    xSpeed *= DriveConstants.kMaxSpeedMetric;
    rot *= DriveConstants.kMaxSpeedMetric;
    var wheelSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, rot));
    setSpeeds(wheelSpeeds);
  }

  public void updateOdometry(Pose2d pose) {
    m_odometry.update(Rotation2d.fromDegrees(yAxis), leftPos, rightPos);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Left out", leftOut);
    SmartDashboard.putNumber("Right out", rightVelocity);
    SmartDashboard.putNumber("Left Position", leftForward);
    SmartDashboard.putNumber("Right Position", rightPos);


    xAxis = ahrs.getRoll();
    yAxis = ahrs.getPitch(); 
    zAxis = ahrs.getYaw();

    leftPos = leftLeader.getPosition().getValueAsDouble() * DriveConstants.kMotorMultiplier;
    rightPos = rightLeader.getPosition().getValueAsDouble() * DriveConstants.kMotorMultiplier;
    leftVelocity = leftLeader.getVelocity().getValueAsDouble() * DriveConstants.kMotorMultiplier;
    rightVelocity = rightLeader.getVelocity().getValueAsDouble() * DriveConstants.kMotorMultiplier;

    SmartDashboard.putBoolean("IMU_Connected", ahrs.isConnected());
    SmartDashboard.putBoolean("IMU_IsCalibrating", ahrs.isCalibrating());
    SmartDashboard.putNumber("X axis", xAxis);
    SmartDashboard.putNumber("Y axis", yAxis);
    SmartDashboard.putNumber("Z axis", zAxis);
  }
}