package frc.robot.subsystems.Arm;

import com.revrobotics.CANSparkLowLevel.MotorType;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {
    
    private ArmIO io;
    private ArmInputsAutoLogged inputs;

    public Arm() {
        io = new ArmIONeos();
        inputs = new ArmInputsAutoLogged();
        io.updateInputs(inputs);
    }

    public void periodic() {
        io.updateInputs(inputs);

        Logger.processInputs("Pivot", inputs);
        //io.setVoltage(RobotContainer.m_operatorController.getLeftY() * 3);
    }
}
