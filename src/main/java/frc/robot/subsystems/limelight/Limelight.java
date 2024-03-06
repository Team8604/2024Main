package frc.robot.subsystems.limelight;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.limelight.LimelightHelpers;
import frc.robot.RobotContainer;

public class Limelight extends SubsystemBase {
    
    public Limelight() {
        
    }

    @Override
    public void periodic() {
        SmartDashboard.putnumber("Limelight thing",  idkrightnow);
    }

}
