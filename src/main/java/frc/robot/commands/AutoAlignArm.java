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
    private double[] anglesObject[] = { { 1.3, 57.9845687018046 }, { 1.8, 45.12728090106311 },
            { 2.3, 36.67748230546999 }, { 2.8, 31.190489836459587 }, { 3.3, 27.425550553393087 },
            { 3.8, 24.798261095205206 }, { 4.3, 22.946273956000447 }, { 4.8, 21.64023629971671 },
            { 5.3, 20.730476163818032 }, { 5.8, 20.11619166748816 } };

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
        double defaultAngle = 66;
        if (!m_LimelightSubsystem.hasTargets) {// return a default angle if limelight can't find targets
            return defaultAngle;
        }

        for (int i = 0; i < anglesObject.length - 1; i++) {// identify range your in
            if (robotDist > anglesObject[i][0] && robotDist <= anglesObject[i + 1][0]) {// dido
                // calculate slope
                double slope = (anglesObject[i][1] - anglesObject[i][1])
                        / (anglesObject[i + 1][1] - anglesObject[i][1]);
                // finding the angle from distance
                return slope * (robotDist - anglesObject[i][1]) + anglesObject[i][1];
            }
        }

        return defaultAngle;
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
