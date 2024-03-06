// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;

public class Drivetrain extends SubsystemBase {
  private double multiplier;

  //initialize motors
  private final TalonFX leftLeader = new TalonFX(DriveConstants.kLeftLeader, DriveConstants.CANBUS_NAME);
  private final TalonFX leftFollower = new TalonFX(DriveConstants.kLeftFollower, DriveConstants.CANBUS_NAME);
  private final TalonFX rightLeader = new TalonFX(DriveConstants.kRightLeader, DriveConstants.CANBUS_NAME);
  private final TalonFX rightFollower = new TalonFX(DriveConstants.kRightFollower, DriveConstants.CANBUS_NAME);

  //drivetrain duty cycles
  private final DutyCycleOut leftOut = new DutyCycleOut(0);
  private final DutyCycleOut rightOut = new DutyCycleOut(0);

  public Drivetrain() {
    /* Configure the devices */
    var leftConfiguration = new TalonFXConfiguration();
    var rightConfiguration = new TalonFXConfiguration();

    /* User can optionally change the configs or leave it alone to perform a factory default */
    leftConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    rightConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    leftLeader.getConfigurator().apply(leftConfiguration);
    leftFollower.getConfigurator().apply(leftConfiguration);
    rightLeader.getConfigurator().apply(rightConfiguration);
    rightFollower.getConfigurator().apply(rightConfiguration);

    /* Set up followers to follow leaders */
    leftFollower.setControl(new Follower(leftLeader.getDeviceID(), false));
    rightFollower.setControl(new Follower(rightLeader.getDeviceID(), false));
    
    leftLeader.setSafetyEnabled(true);
    rightLeader.setSafetyEnabled(true);
  }

  public void setSpeed(double left, double right) {
    multiplier = DriveConstants.kMaxSpeed + DriveConstants.kSpeedIncrease * RobotContainer.fastButton+ DriveConstants.kSpeedDecrease * RobotContainer.slowButton;
    leftOut.Output = multiplier * MathUtil.clamp(left, -1, 1);
    rightOut.Output = multiplier * MathUtil.clamp(right, -1, 1);    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Left out", leftLeader.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Right out", rightLeader.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Left Position", leftLeader.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Right Position", rightLeader.getPosition().getValueAsDouble());
    leftLeader.setControl(leftOut);
    rightLeader.setControl(rightOut);
  }
}