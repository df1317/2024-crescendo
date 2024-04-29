package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Controllers;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

public class AutoAlignArm extends Command {

    private LimelightSubsystem m_LimelightSubsystem;
    private ArmSubsystem m_ArmSubsystem;
    private Controllers m_Controllers;

    private boolean auto;
    private Timer timer = new Timer();
    private double autoTime = 5;

    private double slope = 2.5;
    private double startDist = 2;
    private double endDist = 3.6;

    public AutoAlignArm(LimelightSubsystem LimelightSub, ArmSubsystem ArmSub, Controllers m_Controllers) {
        m_LimelightSubsystem = LimelightSub;
        m_ArmSubsystem = ArmSub;
        this.m_Controllers = m_Controllers;
        addRequirements(LimelightSub, ArmSub);
        auto = DriverStation.isAutonomous();
    }

    /**
     * calculate shooter angle from limelight
     * taking into account arm offset
     */
    public double calculateShooterAngle(double robotDist) {

        double returnAngle = 66;
        int rise = 0;
        int yintercept = 0;

        if (m_LimelightSubsystem.hasTargets) {
            if (robotDist > 1 && robotDist <= 1.5) {
                double slope = rise / .5 * robotDist;
                return slope * robotDist + yintercept;
            } else if (robotDist > 1.5 && robotDist <= 2) {

            } else if (robotDist > 2 && robotDist <= 2.5) {

            } else if (robotDist > 2.5 && robotDist <= 3) {

            }

            else {
                return 66;
            }
        }

        return 0;
    }

    @Override
    public void initialize() {
        SmartDashboard.putBoolean("Aut Moving Arm", true);

        if (m_LimelightSubsystem.hasTargets) {
            m_ArmSubsystem.setAngle(calculateShooterAngle(m_LimelightSubsystem.getSpeakerDistance()));
            timer.start();
            m_ArmSubsystem.runPID();
        }
    }

    @Override
    public void execute() {
        if (m_LimelightSubsystem.hasTargets) {
            m_ArmSubsystem.setAngle(calculateShooterAngle(m_LimelightSubsystem.getSpeakerDistance()));
            m_ArmSubsystem.runPID();
        }
    }

    @Override
    public boolean isFinished() {
        // Check if the button is released or if the specified duration has passed
        if (!m_LimelightSubsystem.hasTargets) {
            return true;
        } else if (auto) {
            return timer.hasElapsed(autoTime);
        } else if (!m_Controllers.rightAutoAlignArmButtonState() && !m_Controllers.leftAutoAlignArmButtonState()) {
            return true;
        }

        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_ArmSubsystem.setAngle(65);
        m_ArmSubsystem.spinDown();

        SmartDashboard.putBoolean("Aut Moving Arm", false);
    }
}
