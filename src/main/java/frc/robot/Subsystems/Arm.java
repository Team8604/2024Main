// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.Constants;
import frc.robot.RobotContainer;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

/** Add your docs here. */
public class Arm {
  RobotContainer RobotContainer;
  public Arm(RobotContainer rc){
    this.RobotContainer = rc;
    RobotContainer.rightArm.restoreFactoryDefaults();
    RobotContainer.leftArm.restoreFactoryDefaults();

    RobotContainer.leftArm.follow(RobotContainer.rightArm, true);

  }

  public void accelerate(double topSpeed, double totalRotations){
    double encoderStartPosition = RobotContainer.armEncoder.getPosition();
    double encoderEndPosition = encoderStartPosition+totalRotations;

    RobotContainer.rightArm.set(0);
    for (double encoderPostion = RobotContainer.armEncoder.getPosition(); encoderPostion < encoderEndPosition; encoderPostion = RobotContainer.armEncoder.getPosition()) {
      double percentageRelativeToRotation = (encoderPostion-encoderStartPosition)/totalRotations;
      RobotContainer.rightArm.set(0.079);
      System.out.println("arm motor should be set to 0.1"+ topSpeed * percentageRelativeToRotation);
      //RobotContainer.rightArm.set(topSpeed * percentageRelativeToRotation);
    }
  }


}

