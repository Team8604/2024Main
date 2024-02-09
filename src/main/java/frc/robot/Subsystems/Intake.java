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
public class Intake {

  public double rotations = 0;

  public double getRotations(){
    return rotations;
  }

  public void setRotations(double r){
    rotations = r;
  }

    public Intake(){
        //int deviceID = 5;//5 for the intake 6 for shooter
        SparkPIDController m_pidController;
        //RelativeEncoder m_encoder;
        
        // initialize motor
         m_pidController = RobotContainer.intakemotor.getPIDController();
      /**
       * The restoreFactoryDefaults method can be used to reset the configuration parameters
       * in the SPARK MAX to their factory default state. If no argument is passed, these
       * parameters will not persist between power cycles
       */
      // m_motor.restoreFactoryDefaults();

      /**
       * In order to use PID functionality for a controller, a SparkPIDController object
       * is constructed by calling the getPIDController() method on an existing
       * CANSparkMax object
       */
      

      // Encoder object created to display position values
      //m_encoder = m_motor.getEncoder();

      // set PID coefficients
      m_pidController.setP(kIntakeP);
      m_pidController.setI(kIntakeI);
      m_pidController.setD(kIntakeD);
      m_pidController.setIZone(kIntakeIz);
      m_pidController.setFF(kIntakeFF);
      m_pidController.setOutputRange(kIntakeMinOutput, kIntakeMaxOutput);

      // display PID coefficients on SmartDashboard
      SmartDashboard.putNumber("P Gain", kIntakeP);
      SmartDashboard.putNumber("I Gain", kIntakeI);
      SmartDashboard.putNumber("D Gain", kIntakeD);
      SmartDashboard.putNumber("I Zone", kIntakeIz);
      SmartDashboard.putNumber("Feed Forward", kIntakeFF);
      SmartDashboard.putNumber("Max Output", kIntakeMaxOutput);
      SmartDashboard.putNumber("Min Output", kIntakeMinOutput);
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
      if((p != kIntakeP)) { m_pidController.setP(p); kIntakeP = p; }
      if((i != kIntakeI)) { m_pidController.setI(i); kIntakeI = i; }
      if((d != kIntakeD)) { m_pidController.setD(d); kIntakeD = d; }
      if((iz != kIntakeIz)) { m_pidController.setIZone(iz); kIntakeIz = iz; }
      if((ff != kIntakeFF)) { m_pidController.setFF(ff); kIntakeFF = ff; }
      if((max != kIntakeMaxOutput) || (min != kIntakeMinOutput)) { 
        m_pidController.setOutputRange(min, max); 
        kIntakeMinOutput = min; kIntakeMaxOutput = max; 
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
      
    //m_pidController.setReference(rotations, CANSparkMax.ControlType.kVelocity);
    SmartDashboard.putNumber("SetPoint", rotations);
    //SmartDashboard.putNumber("ProcessVariable", m_encoder.getPosition());
  }
}
