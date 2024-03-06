package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class AutoAlignArm extends Command {

    private LimelightSubsystem m_LimelightSubsystem;
    private ArmSubsystem m_ArmSubsystem;

    public AutoAlignArm(LimelightSubsystem LimelightSub, ArmSubsystem ArmSub) {
        m_LimelightSubsystem = LimelightSub;
        m_ArmSubsystem = ArmSub;
        addRequirements(LimelightSub, ArmSub);
    }

    @Override
    public void initialize() {
        SmartDashboard.putBoolean("Aut Moving Arm", true);
        double elevation = m_LimelightSubsystem.getElevation();

        m_ArmSubsystem.setAngle(elevation + Constants.ArmShooterConstants.ShooterCollectorConstants.Intake.angle);
        m_ArmSubsystem.runPID();
    }

    @Override
    public void execute() {
        double elevation = m_LimelightSubsystem.getElevation();

        m_ArmSubsystem.setAngle(elevation + Constants.ArmShooterConstants.ShooterCollectorConstants.Intake.angle);
        m_ArmSubsystem.runPID();

        SmartDashboard.putNumber("Current Arm Angle", m_ArmSubsystem.getArmAngle());
        SmartDashboard.putNumber("Desired Arm Angle", elevation);
    }

    @Override
    public boolean isFinished() {
        // Check if the button is released or if the specified duration has passed
        return (m_LimelightSubsystem.getElevation() == m_ArmSubsystem.getArmAngle());
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("Aut Moving Arm", false);
    }
}
