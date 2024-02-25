package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import frc.robot.Constants.IntakeConstants;

public class IntakeIONeo implements IntakeIO{
    private final CANSparkMax IntakeMotor = new CANSparkMax(IntakeConstants.kIntake, MotorType.kBrushless);

    public IntakeIONeo(){
        IntakeMotor.restoreFactoryDefaults();

    }
    public void updateInputs(IntakeInputs inputs) {
        inputs.IntakeMotorOutput = IntakeMotor.get();

    }

    public void set(double speed) {
        Logger.recordOutput("Arm/OutputVoltage", speed);
        IntakeMotor.set(speed);
    }
}
