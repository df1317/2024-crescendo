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
    private double dist[] = { 1.3, 1.8, 2.3, 2.8, 3.3, 3.8, 4.3, 4.8, 5.3, 5.8 };
    private double angles[] = { 57.9845687018046, 45.12728090106311, 36.67748230546999, 31.190489836459587,
            27.425550553393087, 24.798261095205206, 22.946273956000447, 21.64023629971671, 20.730476163818032,
            20.11619166748816 };

    public AutoAlignArm(LimelightSubsystem LimelightSub, ArmSubsystem ArmSub, Controllers m_Controllers) {
        m_LimelightSubsystem = LimelightSub;
        m_ArmSubsystem = ArmSub;
        this.m_Controllers = m_Controllers;
        addRequirements(LimelightSub, ArmSub);
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
        auto = DriverStation.isAutonomous();
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
