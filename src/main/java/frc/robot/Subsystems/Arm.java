// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import frc.robot.RobotContainer;


/** Add your docs here. */
public class Arm {
  RobotContainer RobotContainer;
  public Arm(RobotContainer rc){
    this.RobotContainer = rc;
    RobotContainer.rightArm.restoreFactoryDefaults();
    RobotContainer.leftArm.restoreFactoryDefaults();

    RobotContainer.leftArm.follow(RobotContainer.rightArm, true);

  }

}

