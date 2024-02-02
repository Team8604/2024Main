// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;



/** Add your docs here. */
public final class Constants {
    //drivetrain 
    public static final String CANBUS_NAME = "rio";
    public static final TalonFX leftLeader = new TalonFX(3, CANBUS_NAME);
    public static final TalonFX leftFollower = new TalonFX(4, CANBUS_NAME);
    public static final TalonFX rightLeader = new TalonFX(1, CANBUS_NAME);
    public static final TalonFX rightFollower = new TalonFX(2, CANBUS_NAME);
  
    public static final DutyCycleOut leftOut = new DutyCycleOut(0);
    public static final DutyCycleOut rightOut = new DutyCycleOut(0);
  
    //percents for different drivemodes
    //needs to be incorpritated
    //when certain button is pressed movenment will be multiplied by percent
    //public final double NORMALSPEEDPERCENT = 0.7; //adjust - normal speed lowered to prevent slamming and further issues
    //public final double SLOWMODESPEEDPERCENT = 0.4; //adjust for nessary speed
    //public final double FASTMODESPEEDPERCENT = 1.0; //100% of possible power applied to wheeles


    //controls
    //public static final XboxController joystick = new XboxController(0);
    public static Joystick joystick = new Joystick(0);

    //nums for driving with XboxController
    //public static final int FOWARDCONTROL = 5;
    //public static final int TURNCONTROL = 0;
    
    //public static final int FASTMODE;//add value that goes with button on controller
    //public static final int SLOWMODE;//add value that goes with button on controller
}
