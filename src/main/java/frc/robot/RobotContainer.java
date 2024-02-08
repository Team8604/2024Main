// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import frc.robot.Constants;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;

public class RobotContainer {
    //drivetrain 
    public static final String CANBUS_NAME = "rio";
    public static final TalonFX leftLeader = new TalonFX(3, CANBUS_NAME);
    public static final TalonFX leftFollower = new TalonFX(4, CANBUS_NAME);
    public static final TalonFX rightLeader = new TalonFX(1, CANBUS_NAME);
    public static final TalonFX rightFollower = new TalonFX(2, CANBUS_NAME);
  
    public static final DutyCycleOut leftOut = new DutyCycleOut(0);
    public static final DutyCycleOut rightOut = new DutyCycleOut(0);

    public static final CANSparkMax intakemotor = new CANSparkMax(5, MotorType.kBrushless);
    public static final CANSparkMax shootermotor = new CANSparkMax(6, MotorType.kBrushless);


    public static Drivetrain drivetrain = new Drivetrain();
    public Intake intake = new Intake();
    public Shooter shooter = new Shooter();

    public RobotContainer(){

    }
}
