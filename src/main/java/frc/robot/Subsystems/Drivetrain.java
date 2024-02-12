// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.InvertedValue;

import frc.robot.RobotContainer;

/** Add your docs here. */
public class Drivetrain {
    public Drivetrain(RobotContainer robotContainer){
        /* Configure the devices */
        var leftConfiguration = new TalonFXConfiguration();
        var rightConfiguration = new TalonFXConfiguration();

        /* User can optionally change the configs or leave it alone to perform a factory default */
        leftConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        rightConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        robotContainer.leftLeader.getConfigurator().apply(leftConfiguration);
        robotContainer.leftFollower.getConfigurator().apply(leftConfiguration);
        robotContainer.rightLeader.getConfigurator().apply(rightConfiguration);
        robotContainer.rightFollower.getConfigurator().apply(rightConfiguration);

        /* Set up followers to follow leaders */
        robotContainer.leftFollower.setControl(new Follower(robotContainer.leftLeader.getDeviceID(), false));
        robotContainer.rightFollower.setControl(new Follower(robotContainer.rightLeader.getDeviceID(), false));
    
        robotContainer.leftLeader.setSafetyEnabled(true);
        robotContainer.rightLeader.setSafetyEnabled(true);

        /* Currently in simulation, we do not support FOC, so disable it while simulating */
        if (Utils.isSimulation()){
        robotContainer.leftOut.EnableFOC = false;
        robotContainer.rightOut.EnableFOC = false;
        }

    }
}
