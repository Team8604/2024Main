// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

//depricated import com.revrobotics.CANEncoder;
//depricated import com.revrobotics.CANPIDController;
//import com.revrobotics.CANSparkMax;
//depricated import com.revrobotics.ControlType;
//depricated import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotContainer;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;



/** Add your docs here. */
public class Shooter {

  public double rotations = 0;

  public double getRotations(){
    return rotations;
  }

  public void setRotations(double r){
    rotations = r;
  }

    public Shooter(){
        //int deviceID = 6;//5 for the intake 6 for shooter
        SparkPIDController m_pidController;
        //RelativeEncoder m_encoder;
        
        // initialize motor

      /**
       * The restoreFactoryDefaults method can be used to reset the configuration parameters
       * in the SPARK MAX to their factory default state. If no argument is passed, these
       * parameters will not persist between power cycles
       */
      //m_motor.restoreFactoryDefaults();

      /**
       * In order to use PID functionality for a controller, a SparkPIDController object
       * is constructed by calling the getPIDController() method on an existing
       * CANSparkMax object
       */
      m_pidController = RobotContainer.shootermotor.getPIDController();

      // Encoder object created to display position values
      //m_encoder = m_motor.getEncoder();


      // set PID coefficients
      m_pidController.setP(Constants.kShooterP);
      m_pidController.setI(Constants.kShooterI);
      m_pidController.setD(Constants.kShooterD);
      m_pidController.setIZone(Constants.kShooterIz);
      m_pidController.setFF(Constants.kShooterFF);
      m_pidController.setOutputRange(Constants.kShooterMinOutput, Constants.kShooterMaxOutput);

      // display PID coefficients on SmartDashboard
      SmartDashboard.putNumber("P Gain", Constants.kShooterP);
      SmartDashboard.putNumber("I Gain", Constants.kShooterI);
      SmartDashboard.putNumber("D Gain", Constants.kShooterD);
      SmartDashboard.putNumber("I Zone", Constants.kShooterIz);
      SmartDashboard.putNumber("Feed Forward", Constants.kShooterFF);
      SmartDashboard.putNumber("Max Output", Constants.kShooterMaxOutput);
      SmartDashboard.putNumber("Min Output", Constants.kShooterMinOutput);
      SmartDashboard.putNumber("Set Rotations", 0);

      // read PID coefficients from SmartDashboard
      double p = SmartDashboard.getNumber("P Gain", 0);
      double i = SmartDashboard.getNumber("I Gain", 0);
      double d = SmartDashboard.getNumber("D Gain", 0);
      double iz = SmartDashboard.getNumber("I Zone", 0);
      double ff = SmartDashboard.getNumber("Feed Forward", 0);
      double max = SmartDashboard.getNumber("Max Output", 0);
      double min = SmartDashboard.getNumber("Min Output", 0);
      rotations = SmartDashboard.getNumber("Set Rotations", 0);


      // if PID coefficients on SmartDashboard have changed, write new values to controller
      if((p != Constants.kShooterP)) { m_pidController.setP(p); Constants.kShooterP = p; }
      if((i != Constants.kShooterI)) { m_pidController.setI(i); Constants.kShooterI = i; }
      if((d != Constants.kShooterD)) { m_pidController.setD(d); Constants.kShooterD = d; }
      if((iz != Constants.kShooterIz)) { m_pidController.setIZone(iz); Constants.kShooterIz = iz; }
      if((ff != Constants.kShooterFF)) { m_pidController.setFF(ff); Constants.kShooterFF = ff; }
      if((max != Constants.kShooterMaxOutput) || (min != Constants.kShooterMinOutput)) { 
        m_pidController.setOutputRange(min, max); 
        Constants.kShooterMinOutput = min; Constants.kShooterMaxOutput = max; 
      }

      /**
       * PIDController objects are commanded to a set point using the 
       * SetReference() method.
       * 
       * The first parameter is the value of the set point, whose units vary
       * depending on the control type set in the second parameter.
       * 
       * The second parameter is the control type can be set to one of four 
       * parameters:
       *  com.revrobotics.CANSparkMax.ControlType.kDutyCycle
       *  com.revrobotics.CANSparkMax.ControlType.kPosition
       *  com.revrobotics.CANSparkMax.ControlType.kVelocity
       *  com.revrobotics.CANSparkMax.ControlType.kVoltage
       * 
       * 
       * 
       */
      
    m_pidController.setReference(rotations, CANSparkMax.ControlType.kVelocity);
    SmartDashboard.putNumber("SetPoint", rotations);
    //SmartDashboard.putNumber("ProcessVariable", m_encoder.getPosition());
  }
}
