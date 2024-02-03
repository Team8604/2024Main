// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.InvertedValue;

import frc.robot.Constants;

/** Add your docs here. */
public class Drivetrain {
    public Drivetrain(){
        /* Configure the devices */
        var leftConfiguration = new TalonFXConfiguration();
        var rightConfiguration = new TalonFXConfiguration();

        /* User can optionally change the configs or leave it alone to perform a factory default */
        leftConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        rightConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        Constants.leftLeader.getConfigurator().apply(leftConfiguration);
        Constants.leftFollower.getConfigurator().apply(leftConfiguration);
        Constants.rightLeader.getConfigurator().apply(rightConfiguration);
        Constants.rightFollower.getConfigurator().apply(rightConfiguration);

        /* Set up followers to follow leaders */
        Constants.leftFollower.setControl(new Follower(Constants.leftLeader.getDeviceID(), false));
        Constants.rightFollower.setControl(new Follower(Constants.rightLeader.getDeviceID(), false));
    
        Constants.leftLeader.setSafetyEnabled(true);
        Constants.rightLeader.setSafetyEnabled(true);

        /* Currently in simulation, we do not support FOC, so disable it while simulating */
        if (Utils.isSimulation()){
        Constants.leftOut.EnableFOC = false;
        Constants.rightOut.EnableFOC = false;
        }

    }
}
