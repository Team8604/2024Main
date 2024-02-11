// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

import frc.robot.Constants;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;

public class RobotContainer {
    //drivetrain 
    public final String CANBUS_NAME = "rio";
    public final TalonFX leftLeader = new TalonFX(Constants.kLeftLeader, CANBUS_NAME);
    public final TalonFX leftFollower = new TalonFX(Constants.kLeftFollower, CANBUS_NAME);
    public final TalonFX rightLeader = new TalonFX(Constants.kRightLeader, CANBUS_NAME);
    public final TalonFX rightFollower = new TalonFX(Constants.kRightFollower, CANBUS_NAME);
  
    public final DutyCycleOut leftOut = new DutyCycleOut(0);
    public final DutyCycleOut rightOut = new DutyCycleOut(0);

    public final CANSparkMax intakeMotor = new CANSparkMax(Constants.kIntakeMotor, MotorType.kBrushless);
    public final CANSparkMax shooterMotor = new CANSparkMax(Constants.kShooterMotor, MotorType.kBrushless);

    public final XboxController operator = new XboxController(1);
    public final Joystick driverJoystick = new Joystick(0);

    public Drivetrain drivetrain;
    public Intake intake = new Intake();
    public Shooter shooter = new Shooter();

    public RobotContainer(){}
}
