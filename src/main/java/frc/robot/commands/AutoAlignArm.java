package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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
    private double dist[] = { 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0, 5.5 };
    private double angles[] = { 51, 43, 36, 33.5, 27.76, 24.15, 22.3681022328328,
            21.235281363087378, 20.453772063328003 };

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
        int rangeindex = 0;
        if (!m_LimelightSubsystem.hasTargets) {// return a default angle if limelight can't find targets
            return returnAngle;
        }

        for (int i = 0; i < dist.length - 1; i++) {// identify range your in
            if (robotDist > dist[i] && robotDist <= dist[i + 1]) {// dido
                rangeindex = i;
                break;
            }
        }
        // calculate slope
        double slope = (angles[rangeindex + 1] - angles[rangeindex]) / (dist[rangeindex + 1] - dist[rangeindex]);
        // finding the angle from distance
        returnAngle = slope * (robotDist - dist[rangeindex]) + angles[rangeindex];

        return returnAngle;
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
