// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Commands.*;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  
  private int printCount = 0;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  private RobotContainer robotContainer;

  @Override
  public void robotInit() {
    robotContainer = new RobotContainer();

  }

  @Override
  public void robotPeriodic() {
    if (++printCount >= 10) {
      printCount = 0;
      SmartDashboard.putNumber("Left out", RobotContainer.leftLeader.getVelocity().getValueAsDouble());
      SmartDashboard.putNumber("Right out", RobotContainer.rightLeader.getVelocity().getValueAsDouble());
      SmartDashboard.putNumber("Left Position", RobotContainer.leftLeader.getPosition().getValueAsDouble());
      SmartDashboard.putNumber("Right Position", RobotContainer.rightLeader.getPosition().getValueAsDouble());
      SmartDashboard.putNumber("Arm Position", RobotContainer.armEncoder.getPosition());
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
    double fwd = -robotContainer.driverJoystick.getY();
    double rot;
    double rot1 = robotContainer.driverJoystick.getX();
    double rot2 = robotContainer.driverJoystick.getZ()*1.2;

    if (rot2 > 1){
      rot2 = 1;
    } else if (rot2 < -1){
      rot2 = -1;
    }
    //gets either the bigger of twist or sideways
    if (rot2 < 0.1){
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
    fwd *=0.25;
    rot *=0.25;
    
    /* Set output to control frames */
    robotContainer.leftOut.Output = fwd + rot;
    robotContainer.rightOut.Output = fwd - rot;
    /* And set them to the motors */
    if (!robotContainer.driverJoystick.getRawButtonPressed(2)/*getAButton()*/) {
      robotContainer.leftLeader.setControl(robotContainer.leftOut);
      robotContainer.rightLeader.setControl(robotContainer.rightOut);
    }
    
      //intake control
      boolean intakeFwd = robotContainer.operator.getAButton();
      boolean intakeBwd = robotContainer.operator.getBButton();
      //shooter conrol
      boolean shooterFwd = robotContainer.operator.getXButton();
      boolean shooterBwd = robotContainer.operator.getYButton();

      //arm control
      boolean armFwd = robotContainer.operator.getLeftBumperPressed(); 
      
      //intake
      if (intakeFwd){
        //spin intake motor foward
        if (intakeFwd && shooterFwd){
          robotContainer.intakeMotor.set(1);
          //System.out.println("Intake set to 1");
        } else{
          robotContainer.intakeMotor.set(0.25);
          //System.out.println("Intake set to 0.8");
        }
      } else if (intakeBwd) {
        //spin intake motor backwards (gets rid of jamed note)
        robotContainer.intakeMotor.set(-0.25);
        //System.out.println("Intake set to -0.5");
      } else {
        //set intake motor to stop
        robotContainer.intakeMotor.set(0);
        //System.out.println("Intake set to 0");
      }
      
      //shooter
      if (shooterFwd){
        //spin shooter motor fowardss
        robotContainer.shooterMotor.set(1);
        //System.out.println("Shooter set to 0.5");
      } else if (shooterBwd) {
        //spin shooter motor backwards (gets rid of jamed note)
        robotContainer.shooterMotor.set(-0.5);
        //System.out.println("Shooter set to -0.5");
      } else {
        //set shooter motor to stop
        robotContainer.shooterMotor.set(0);
        //System.out.println("Shooter set to 0");
      }


      //arm
      //gear box is 100-1
      if (armFwd){
        double start = RobotContainer.armEncoder.getPosition();
        double end = start +100;
        double desiredSpeed = 0.1; //10% of full speed for arm
        SmartDashboard.putNumber("Encoder Position start", RobotContainer.armEncoder.getPosition());

        RobotContainer.rightArm.set(0/*RobotContainer.operator.getLeftX()*/);
        /*while (robotContainer.armEncoder.getPosition() < end){
          
          System.out.println("motor should be set to 0.1----");
          SmartDashboard.putNumber("Encoder Position", robotContainer.armEncoder.getPosition());
        }*/
        System.out.println("Accelerate start------");
        RobotContainer.arm.accelerate(desiredSpeed, (end/4));
        System.out.println("Accelerate end------");

        RobotContainer.rightArm.set(0/*RobotContainer.operator.getLeftX()*/);

      }
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    /* Zero out controls so we aren't just relying on the enable frame */
    //drivetrain
    RobotContainer.leftOut.Output = 0;
    RobotContainer.rightOut.Output = 0;
    RobotContainer.leftLeader.setControl(RobotContainer.leftOut);
    RobotContainer.rightLeader.setControl(RobotContainer.rightOut);
    //intake
    robotContainer.intakeMotor.set(0);
    //shooter
    RobotContainer.shooterMotor.set(0);
    //arm
    RobotContainer.rightArm.set(0);

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
