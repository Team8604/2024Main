// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import frc.robot.RobotContainer;
import frc.robot.Constants;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class Arm extends SubsystemBase {
  RobotContainer robotContainer;
  public PIDController pid;

  private DutyCycleEncoder absoluteEncoder;

  public Arm(RobotContainer rc){
    this.robotContainer = rc;

    robotContainer.rightArm.restoreFactoryDefaults();
    robotContainer.leftArm.restoreFactoryDefaults();

    robotContainer.leftArm.follow(robotContainer.rightArm, true);

    pid = new PIDController(Constants.kArmP, Constants.kArmI, Constants.kArmD);

    absoluteEncoder = new DutyCycleEncoder(9);
  }

  @Override
  public void periodic() {
      Logger.recordOutput("Absolute Encoder", absoluteEncoder.getAbsolutePosition());
  }

  public void accelerate(double totalRotations){
    double encoderStartPosition = robotContainer.armEncoder.getAbsolutePosition();
    double encoderEndPosition = encoderStartPosition+totalRotations;

    robotContainer.rightArm.set(0);
    for (double encoderPostion = robotContainer.armEncoder.getAbsolutePosition(); encoderPostion < encoderEndPosition; encoderPostion = robotContainer.armEncoder.getAbsolutePosition()) {
      double percentageRelativeToRotation = (encoderPostion-encoderStartPosition)/totalRotations;
      robotContainer.rightArm.set(Constants.kArmMaxSpeed);
      System.out.println("arm motor should be set to 0.1"+ Constants.kArmMaxSpeed * percentageRelativeToRotation);
      //robotContainer.rightArm.set(topSpeed * percentageRelativeToRotation);
    }
  }

}

