// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import com.revrobotics.CANSparkMax;


import frc.robot.Constants;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Arm;

public class RobotContainer {
    //drivetrain 
    public static final String CANBUS_NAME = "rio";
    public static final TalonFX leftLeader = new TalonFX(Constants.kLeftLeader, CANBUS_NAME);
    public static final TalonFX leftFollower = new TalonFX(Constants.kLeftFollower, CANBUS_NAME);
    public static final TalonFX rightLeader = new TalonFX(Constants.kRightLeader, CANBUS_NAME);
    public static final TalonFX rightFollower = new TalonFX(Constants.kRightFollower, CANBUS_NAME);
  
    public static final DutyCycleOut leftOut = new DutyCycleOut(0);
    public static final DutyCycleOut rightOut = new DutyCycleOut(0);

    //arm
    public static final CANSparkMax m_RightArmMotor = new CANSparkMax(Constants.kRightArm, MotorType.kBrushless);
    public static final CANSparkMax m_LeftArmMotor = new CANSparkMax(Constants.kLeftArm, MotorType.kBrushless);
    public static final RelativeEncoder m_ArmEncoder = m_RightArmMotor.getEncoder(); 

    //intake and shooter
    public static final CANSparkMax intakeMotor = new CANSparkMax(Constants.kIntakeMotor, MotorType.kBrushless);
    public static final CANSparkMax shooterMotor = new CANSparkMax(Constants.kShooterMotor, MotorType.kBrushless);

    //controllers
    public static final XboxController operator = new XboxController(1);
    public static final Joystick driverJoystick = new Joystick(0);

    //initialize everything
    public static Drivetrain drivetrain = new Drivetrain();
    public Intake intake = new Intake();
    public Shooter shooter = new Shooter();
    public Arm arm = new Arm();

    public RobotContainer(){

    }
}
