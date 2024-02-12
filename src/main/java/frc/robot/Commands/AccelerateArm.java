// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import frc.robot.RobotContainer;

/** Add your docs here. */
public class AccelerateArm {
    public static void accelerateArm(RobotContainer robotContainer, double topSpeed, double totalRotations){
    double encoderStartPosition = robotContainer.m_ArmEncoder.getPosition();
    double encoderEndPosition = encoderStartPosition+totalRotations;

    robotContainer.m_RightArmMotor.set(0);
    for (double encoderPostion = robotContainer.m_ArmEncoder.getPosition(); encoderPostion < encoderEndPosition; encoderPostion = robotContainer.m_ArmEncoder.getPosition()) {
      double percentageRelativeToRotation = (encoderPostion-encoderStartPosition)/totalRotations;
      robotContainer.m_RightArmMotor.set(0.079);
      System.out.println("arm motor should be set to 0.1"+ topSpeed * percentageRelativeToRotation);
      //robotContainer.m_RightArmMotor.set(topSpeed * percentageRelativeToRotation);
    }
  }
}
