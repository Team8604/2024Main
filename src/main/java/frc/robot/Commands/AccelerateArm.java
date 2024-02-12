// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import frc.robot.RobotContainer;

/** Add your docs here. */
public class AccelerateArm {
    public static void accelerateArm(RobotContainer robotContainer, double topSpeed, double totalRotations){
    double encoderStartPosition = robotContainer.armEncoder.getPosition();
    double encoderEndPosition = encoderStartPosition+totalRotations;

    robotContainer.rightArm.set(0);
    for (double encoderPostion = robotContainer.armEncoder.getPosition(); encoderPostion < encoderEndPosition; encoderPostion = robotContainer.armEncoder.getPosition()) {
      double percentageRelativeToRotation = (encoderPostion-encoderStartPosition)/totalRotations;
      robotContainer.rightArm.set(0.079);
      System.out.println("arm motor should be set to 0.1"+ topSpeed * percentageRelativeToRotation);
      //robotContainer.rightArm.set(topSpeed * percentageRelativeToRotation);
    }
  }
}
