// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.InvertedValue;

import frc.robot.Constants;
import frc.robot.RobotContainer;

/** Add your docs here. */
public class Drivetrain {
    public Drivetrain(RobotContainer RobotContainer){
        /* Configure the devices */
        var leftConfiguration = new TalonFXConfiguration();
        var rightConfiguration = new TalonFXConfiguration();

        /* User can optionally change the configs or leave it alone to perform a factory default */
        leftConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        rightConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        RobotContainer.leftLeader.getConfigurator().apply(leftConfiguration);
        RobotContainer.leftFollower.getConfigurator().apply(leftConfiguration);
        RobotContainer.rightLeader.getConfigurator().apply(rightConfiguration);
        RobotContainer.rightFollower.getConfigurator().apply(rightConfiguration);

        /* Set up followers to follow leaders */
        RobotContainer.leftFollower.setControl(new Follower(RobotContainer.leftLeader.getDeviceID(), false));
        RobotContainer.rightFollower.setControl(new Follower(RobotContainer.rightLeader.getDeviceID(), false));
    
        RobotContainer.leftLeader.setSafetyEnabled(true);
        RobotContainer.rightLeader.setSafetyEnabled(true);

        /* Currently in simulation, we do not support FOC, so disable it while simulating */
        if (Utils.isSimulation()){
        RobotContainer.leftOut.EnableFOC = false;
        RobotContainer.rightOut.EnableFOC = false;
        }

    }
}
