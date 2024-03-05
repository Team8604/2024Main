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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
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

    //drivetrain duty cycles
    private final DutyCycleOut leftOut = new DutyCycleOut(0);
    private final DutyCycleOut rightOut = new DutyCycleOut(0);

    //navx values
    public static double xAxis; //Pitch 
    public static double yAxis; //Roll 
    public static double zAxis; //Yaw
    public AHRS ahrs;

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

        //set navx
        try {
            ahrs = new AHRS(SerialPort.Port.kUSB1);
            ahrs.enableLogging(true);
        } catch (RuntimeException ex ) {
            DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
        }
    }

    public void setSpeed(double left, double right) {
        leftOut.Output = DriveConstants.kMaxSpeed * MathUtil.clamp(left, -1, 1);
        rightOut.Output = DriveConstants.kMaxSpeed * MathUtil.clamp(right, -1, 1);    
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

        xAxis = ahrs.getPitch();
        yAxis = ahrs.getRoll();
        zAxis = ahrs.getYaw();

        SmartDashboard.putNumber("X axis", xAxis);
        SmartDashboard.putNumber("Y axis", yAxis);
        SmartDashboard.putNumber("Z axis", zAxis);

    }
}