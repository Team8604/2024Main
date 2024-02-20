// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
//https://github.com/REVrobotics/2m-Distance-Sensor/releases

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Subsystems.IntakeSensor;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  
  private int printCount = 0;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  private RobotContainer robotContainer;

  @Override
  public void robotInit() {
    Logger.recordMetadata("ProjectName", "MyProject"); // Set a metadata value

    if (isReal()) {
        // Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
        // new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
    } else {
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
        Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
    }

    // Logger.disableDeterministicTimestamps() // See "Deterministic Timestamps" in the "Understanding Data Flow" page
    Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.

    robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    if (++printCount >= 10) {
      printCount = 0;
      SmartDashboard.putNumber("Left out", robotContainer.leftLeader.getVelocity().getValueAsDouble());
      SmartDashboard.putNumber("Right out", robotContainer.rightLeader.getVelocity().getValueAsDouble());
      SmartDashboard.putNumber("Left Position", robotContainer.leftLeader.getPosition().getValueAsDouble());
      SmartDashboard.putNumber("Right Position", robotContainer.rightLeader.getPosition().getValueAsDouble());
      SmartDashboard.putNumber("Arm Position", robotContainer.armEncoder.getAbsolutePosition());
      SmartDashboard.putNumber("Arm Speed", MathUtil.clamp(robotContainer.arm.pid.calculate(robotContainer.armEncoder.getAbsolutePosition(), 0.90), -1 * Constants.kArmMaxSpeed, Constants.kArmMaxSpeed));
    }
    CommandScheduler.getInstance().run();
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
    double fwd = -1 * Math.pow(robotContainer.driverJoystick.getY(), 2.6);
    double rot;
    double rot1 = Math.pow(robotContainer.driverJoystick.getX(), 2.6);
    double rot2 = Math.pow(robotContainer.driverJoystick.getZ(), 2.6);

    if (rot2 > 1){
      rot2 = 1;
    } else if (rot2 < -1){
      rot2 = -1;
    }
    //gets either the bigger of twist or sideways
    if (Math.abs(rot2) < 0.1){
      rot2=0;
    }
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
    //sets drivetrain motors to 25% speed
    fwd *= Constants.kDrivetrainSpeed;
    rot *= Constants.kDrivetrainSpeed;
    
    /* Set output to control frames */
    robotContainer.leftOut.Output = fwd + rot;
    robotContainer.rightOut.Output = fwd - rot;
    /* And set them to the motors */
    if (!robotContainer.driverJoystick.getRawButtonPressed(2)/*getAButton()*/) {
      robotContainer.leftLeader.setControl(robotContainer.leftOut);
      robotContainer.rightLeader.setControl(robotContainer.rightOut);
    }
    
      //intake control
      boolean operatorA = robotContainer.operator.getAButton();
      boolean operatorB = robotContainer.operator.getBButton();
      //shooter conrol
      boolean operatorX = robotContainer.operator.getXButton();
      boolean operatorY = robotContainer.operator.getYButton();
      boolean righBumper = robotContainer.operator.getRightBumperPressed();

      //arm control
      boolean leftBumper = robotContainer.operator.getLeftBumperPressed(); 
      
      System.out.println("------called function result"+IntakeSensor.getIntakeSensorDistance());
      //intake
      if (operatorA){
        //spin intake motor foward
        if (operatorA && operatorX){
          robotContainer.intakeMotor.set(Constants.kMaxIntakeSpeed);
        }
        else if (/*IntakeSensor.getIntakeSensorDistance()*/0 < 20){//adjust number so that intake stops when object is blocking sensor the motor doesn't spin
          robotContainer.intakeMotor.set(0);
        }
        else {
          robotContainer.intakeMotor.set(Constants.kIntakeSpeed);
        }
      } else if (operatorB) {
        //spin intake motor backwards (gets rid of jamed note)
        robotContainer.intakeMotor.set(Constants.kIntakeSpeed);
        //System.out.println("Intake set to -0.5");
      } else {
        //set intake motor to stop
        robotContainer.intakeMotor.set(0);
        //System.out.println("Intake set to 0");
      }
      
      //shooter
      if (operatorX){
        //spin shooter motor foward for shooting
        robotContainer.shooterMotor.set(Constants.kShooterSpeed);
      } else if (operatorY) {
        //spin shooter motor foward for scoring in amp
        robotContainer.shooterMotor.set(Constants.kAmpShooterSpeed);
      } else if (righBumper) {
        //spin shooter motor backwards (gets rid of jamed note)
        robotContainer.shooterMotor.set(-Constants.kAmpShooterSpeed);
      } else {
        //set shooter motor to stop
        robotContainer.shooterMotor.set(0);
      }


      //arm
      //gear box is 100-1
      robotContainer.rightArm.set(MathUtil.clamp(robotContainer.arm.pid.calculate(robotContainer.armEncoder.getAbsolutePosition(), 0.90), -1 * Constants.kArmMaxSpeed, Constants.kArmMaxSpeed));

  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    /* Zero out controls so we aren't just relying on the enable frame */
    //drivetrain
    robotContainer.leftOut.Output = 0;
    robotContainer.rightOut.Output = 0;
    robotContainer.leftLeader.setControl(robotContainer.leftOut);
    robotContainer.rightLeader.setControl(robotContainer.rightOut);
    //intake
    robotContainer.intakeMotor.set(0);
    //shooter
    robotContainer.shooterMotor.set(0);
    //arm
    robotContainer.rightArm.set(0);

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
