// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
/*import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.revrobotics.CANSparkMax;*/
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.TimedRobot;/**
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Subsystem;*/
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Arm;

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
  private RobotContainer RobotContainer;

  @Override
  public void robotInit() {
    RobotContainer = new RobotContainer();

  }

  @Override
  public void robotPeriodic() {
    if (++printCount >= 10) {
      printCount = 0;
      //System.out.println("Left out: " + RobotContainer.leftLeader.get());
      //System.out.println("Right out: " + RobotContainer.rightLeader.get());
      //System.out.println("Left Pos: " + RobotContainer.leftLeader.getPosition());
      //System.out.println("Right Pos: " + RobotContainer.rightLeader.getPosition());
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
    double fwd = -RobotContainer.driverJoystick.getY();
    double rot;
    double rot1 = RobotContainer.driverJoystick.getX();
    double rot2 = RobotContainer.driverJoystick.getZ()*1.2;
    SparkPIDController m_pidController;
    m_pidController = RobotContainer.intakeMotor.getPIDController();

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
    RobotContainer.leftOut.Output = fwd + rot;
    RobotContainer.rightOut.Output = fwd - rot;
    /* And set them to the motors */
    if (!RobotContainer.driverJoystick.getRawButtonPressed(2)/*getAButton()*/) {
      RobotContainer.leftLeader.setControl(RobotContainer.leftOut);
      RobotContainer.rightLeader.setControl(RobotContainer.rightOut);
    }
    
      //intake control
      boolean intakeFwd = RobotContainer.operator.getAButton();
      boolean intakeBwd = RobotContainer.operator.getBButton();
      //shooter conrol
      boolean shooterFwd = RobotContainer.operator.getXButton();
      boolean shooterBwd = RobotContainer.operator.getYButton();

      //arm control
      boolean armFwd = RobotContainer.operator.getLeftBumperPressed(); 
      
      //intake
      if (intakeFwd){
        //spin intake motor foward
        if (intakeFwd && shooterFwd){
          RobotContainer.intakeMotor.set(1);
          //System.out.println("Intake set to 1");
        } else{
          RobotContainer.intakeMotor.set(0.25);
          //System.out.println("Intake set to 0.8");
        }
      } else if (intakeBwd) {
        //spin intake motor backwards (gets rid of jamed note)
        RobotContainer.intakeMotor.set(-0.25);
        //System.out.println("Intake set to -0.5");
      } else {
        //set intake motor to stop
        RobotContainer.intakeMotor.set(0);
        //System.out.println("Intake set to 0");
      }
      
      //shooter
      if (shooterFwd){
        //spin shooter motor foward
        RobotContainer.shooterMotor.set(1);
        //System.out.println("Shooter set to 0.5");
      } else if (shooterBwd) {
        //spin shooter motor backwards (gets rid of jamed note)
        RobotContainer.shooterMotor.set(-0.5);
        //System.out.println("Shooter set to -0.5");
      } else {
        //set shooter motor to stop
        RobotContainer.shooterMotor.set(0);
        //System.out.println("Shooter set to 0");
      }


      //arm
      //gear box is 100-1
      if (armFwd){
        double start = RobotContainer.m_ArmEncoder.getPosition();
        double end = start +100;
        double disiredSpeed = 0.1; //10% of full speed for arm
        SmartDashboard.putNumber("Encoder Position start", RobotContainer.m_ArmEncoder.getPosition());

        RobotContainer.m_RightArmMotor.set(0.1/*RobotContainer.operator.getLeftX()*/);
        while (RobotContainer.m_ArmEncoder.getPosition() < end){
          
          System.out.println("motor should be set to 0.1----");
          SmartDashboard.putNumber("Encoder Position", RobotContainer.m_ArmEncoder.getPosition());
        }
        RobotContainer.m_RightArmMotor.set(0/*RobotContainer.operator.getLeftX()*/);

      }
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    /* Zero out controls so we aren't just relying on the enable frame */
    //drivetrian
    RobotContainer.leftOut.Output = 0;
    RobotContainer.rightOut.Output = 0;
    RobotContainer.leftLeader.setControl(RobotContainer.leftOut);
    RobotContainer.rightLeader.setControl(RobotContainer.rightOut);
    //intake
    //m_pidController.setReference(0, CANSparkMax.ControlType.kVelocity);
    /* RobotContainer.intakeMotor.setRotations(0);
    //shooter
    RobotContainer.shooterMotor.setRotations(0); */

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
