// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import frc.robot.RobotContainer;
import frc.robot.Constants;

/** Add your docs here. */
public class Arm {
  RobotContainer robotContainer;
  public Arm(RobotContainer rc){
    this.robotContainer = rc;
    robotContainer.rightArm.restoreFactoryDefaults();
    robotContainer.leftArm.restoreFactoryDefaults();

    robotContainer.leftArm.follow(robotContainer.rightArm, true);

  }

  public void accelerate(double totalRotations){
    double encoderStartPosition = robotContainer.armEncoder.getAbsolutePosition();
    double encoderEndPosition = encoderStartPosition+totalRotations;

    robotContainer.rightArm.set(0);
    for (double encoderPostion = robotContainer.armEncoder.getAbsolutePosition(); encoderPostion < encoderEndPosition; encoderPostion = robotContainer.armEncoder.getAbsolutePosition()) {
      double percentageRelativeToRotation = (encoderPostion-encoderStartPosition)/totalRotations;
      robotContainer.rightArm.set(Constants.kArmSpeed);
      System.out.println("arm motor should be set to 0.1"+ Constants.kArmSpeed * percentageRelativeToRotation);
      //robotContainer.rightArm.set(topSpeed * percentageRelativeToRotation);
    }
  }

}

