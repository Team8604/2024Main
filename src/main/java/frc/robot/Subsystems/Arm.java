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
  
  public Arm(RobotContainer RobotContainer){
    RobotContainer.m_RightArmMotor.restoreFactoryDefaults();
    RobotContainer.m_LeftArmMotor.restoreFactoryDefaults();

    RobotContainer.m_LeftArmMotor.follow(RobotContainer.m_RightArmMotor, true);

  }

  public static void accelerate(double topSpeed, double totalRotations){
    double encoderStartPosition = RobotContainer.m_ArmEncoder.getPosition();
    double encoderEndPosition = encoderStartPosition+totalRotations;

    RobotContainer.m_RightArmMotor.set(0);
    for (double encoderPostion = RobotContainer.m_ArmEncoder.getPosition(); encoderPostion < encoderEndPosition; encoderPostion = RobotContainer.m_ArmEncoder.getPosition()) {
      double percentageRelativeToRotation = (encoderPostion-encoderStartPosition)/totalRotations;
      RobotContainer.m_RightArmMotor.set(0.079);
      System.out.println("arm motor should be set to 0.1"+ topSpeed * percentageRelativeToRotation);
      //RobotContainer.m_RightArmMotor.set(topSpeed * percentageRelativeToRotation);
    }
  }


}

