// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import frc.robot.RobotContainer;


/** Add your docs here. */
public class Arm {
  
  public Arm(RobotContainer RobotContainer){
    RobotContainer.m_RightArmMotor.restoreFactoryDefaults();
    RobotContainer.m_LeftArmMotor.restoreFactoryDefaults();

    RobotContainer.m_LeftArmMotor.follow(RobotContainer.m_RightArmMotor, true);
  }

}

