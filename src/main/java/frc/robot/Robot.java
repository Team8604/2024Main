// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

import frc.Constants;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String CANBUS_NAME = "rio";
  


  private int printCount = 0;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    /* Configure the devices */
    var leftConfiguration = new TalonFXConfiguration();
    var rightConfiguration = new TalonFXConfiguration();

    /* User can optionally change the configs or leave it alone to perform a factory default */
    leftConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    rightConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    Constants.leftLeader.getConfigurator().apply(leftConfiguration);
    Constants.leftFollower.getConfigurator().apply(leftConfiguration);
    Constants.rightLeader.getConfigurator().apply(rightConfiguration);
    Constants.rightFollower.getConfigurator().apply(rightConfiguration);

    /* Set up followers to follow leaders */
    Constants.leftFollower.setControl(new Follower(Constants.leftLeader.getDeviceID(), false));
    Constants.rightFollower.setControl(new Follower(Constants.rightLeader.getDeviceID(), false));
  
    Constants.leftLeader.setSafetyEnabled(true);
    Constants.rightLeader.setSafetyEnabled(true);

    /* Currently in simulation, we do not support FOC, so disable it while simulating */
    if (Utils.isSimulation()){
      Constants.leftOut.EnableFOC = false;
      Constants.rightOut.EnableFOC = false;
    }
  }

  @Override
  public void robotPeriodic() {
    if (++printCount >= 10) {
      printCount = 0;
      System.out.println("Left out: " + Constants.leftLeader.get());
      System.out.println("Right out: " + Constants.rightLeader.get());
      System.out.println("Left Pos: " + Constants.leftLeader.getPosition());
      System.out.println("Right Pos: " + Constants.rightLeader.getPosition());
    }
  }

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    /* Get forward and rotational throttle from joystick */
    /* invert the joystick Y because forward Y is negative */
    //code from joystic for drivetrain
    double fwd = -Constants.joystick.getY();
    double rot;
    double rot1 = Constants.joystick.getX();
    double rot2 = Constants.joystick.getZ()*1.5;
    if (rot2 > 1){
      rot2 = 1;
    } else if (rot2 < -1){
      rot2 = -1;
    }
    //gets either the bigger of twist or sideways
    if (Math.abs(rot1) >= Math.abs(rot2)){
      rot = rot1;
    } else {
      rot = rot2;
    }
    //dead zone
    if (rot < 0.08 && rot > -0.08){
      rot = 0;
    }
    if (fwd < 0.08 && fwd > -0.08){
      fwd = 0;
    }

    //Code from controller for drivertain
    //double fwd = -Constants.joystick.getRawAxis(Constants.FOWARDCONTROL);
    //double rot = Constants.joystick.getRawAxis(Constants.TURNCONTROL);

    /* Set output to control frames */
    Constants.leftOut.Output = fwd + rot;
    Constants.rightOut.Output = fwd - rot;
    /* And set them to the motors */
    if (!Constants.joystick.getRawButtonPressed(2)/*getAButton()*/) {
      Constants.leftLeader.setControl(Constants.leftOut);
      Constants.rightLeader.setControl(Constants.rightOut);
    }
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    /* Zero out controls so we aren't just relying on the enable frame */
    Constants.leftOut.Output = 0;
    Constants.rightOut.Output = 0;
    Constants.leftLeader.setControl(Constants.leftOut);
    Constants.rightLeader.setControl(Constants.rightOut);
  }

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
