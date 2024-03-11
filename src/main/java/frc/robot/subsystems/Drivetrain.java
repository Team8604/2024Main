// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.kauailabs.navx.frc.AHRS.SerialDataType;
import com.kauailabs.navx.frc.AHRS;

import frc.robot.Constants.DriveConstants;

public class Drivetrain extends SubsystemBase {

  //initialize motors
  private final TalonFX leftLeader = new TalonFX(DriveConstants.kLeftLeader, DriveConstants.CANBUS_NAME);
  private final TalonFX leftFollower = new TalonFX(DriveConstants.kLeftFollower, DriveConstants.CANBUS_NAME);
  private final TalonFX rightLeader = new TalonFX(DriveConstants.kRightLeader, DriveConstants.CANBUS_NAME);
  private final TalonFX rightFollower = new TalonFX(DriveConstants.kRightFollower, DriveConstants.CANBUS_NAME);

  private final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(DriveConstants.kWidth);
  private final PIDController m_leftPIDController = new PIDController(DriveConstants.kp, DriveConstants.ki, DriveConstants.kd);
  private final PIDController m_rightPIDController = new PIDController(DriveConstants.kp, DriveConstants.ki, DriveConstants.kd);

  
  //navx values
  public double xAxis; //Roll
  public double yAxis; //Pitch 
  public double zAxis; //Yaw
  public AHRS ahrs;

  public Drivetrain() {
    /* Set up followers to follow leaders */
    leftFollower.setControl(new Follower(leftLeader.getDeviceID(), false));
    rightFollower.setControl(new Follower(rightLeader.getDeviceID(), false));
        
    leftLeader.setSafetyEnabled(true);
    rightLeader.setSafetyEnabled(true);

    //set navx
    ahrs = new AHRS();
    ahrs.enableLogging(true);
  }
  public void setSpeed(DifferentialDriveWheelSpeeds speed){
    double leftFoward = m_leftPIDController.calculate(speed.leftMetersPerSecond);
    double rightFoward = m_rightPIDController.calculate(speed.rightMetersPerSecond);

    double leftOut = m_leftPIDController.calculate(, speed.leftMetersPerSecond); //add encoder value
    double rightOut = m_leftPIDController.calculate(, speed.rightMetersPerSecond);  //add encoder value 

    leftLeader.setVoltage(leftOut + leftFoward);
    rightLeader.setVoltage(rightOut + rightFoward);
  }

  public void drive(double left, double right) {  
    var speed = m_kinematics.toWheelSpeeds(new ChassisSpeeds(left+right, 0.0, right));
    setSpeed(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Left out", leftLeader.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Right out", rightLeader.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Left Position", leftLeader.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Right Position", rightLeader.getPosition().getValueAsDouble());


    xAxis = ahrs.getRoll();
    yAxis = ahrs.getPitch(); 
    zAxis = ahrs.getYaw();
    SmartDashboard.putBoolean("IMU_Connected", ahrs.isConnected());
    SmartDashboard.putBoolean("IMU_IsCalibrating", ahrs.isCalibrating());
    SmartDashboard.putNumber("X axis", xAxis);
    SmartDashboard.putNumber("Y axis", yAxis);
    SmartDashboard.putNumber("Z axis", zAxis);
  }
}