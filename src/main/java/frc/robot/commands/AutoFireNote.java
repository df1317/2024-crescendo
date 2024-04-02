package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Controllers;
import frc.robot.subsystems.FiringSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class AutoFireNote extends Command {
    private FiringSubsystem m_FiringSubsystem;
    private LimelightSubsystem m_LimelightSubsystem;
    private CommandXboxController xboxController;

    private Controllers m_Controllers;

    private boolean intake;
    private boolean shoot;
    private boolean autoAmp;
    private Trigger autoAmpTrigger;
    private boolean manualAmp;

    private double timer;
    private double intakeSpeed = -600;
    private double intakeAutoAmpSpeed = 1000;
    private double shooterSpeed = -3400;
    private double shooterManualAmp = -2000;

    private double slope = -500;
    private double startDist = 2;
    private double endDist = 3.6;

    private double waitTime = 2000;
    private double shootTime = 1000;

    public AutoFireNote(FiringSubsystem FiringSub, LimelightSubsystem LimeLightSub, Controllers controllers) {
        m_FiringSubsystem = FiringSub;
        m_LimelightSubsystem = LimeLightSub;
        this.m_Controllers = controllers;

        addRequirements(FiringSub);
    }

    @Override
    public void initialize() {
        SmartDashboard.putString("Auto Firing Status", "Initializing");

        intake = m_Controllers.intakeButton.getAsBoolean();
        shoot = m_Controllers.shooterButton.getAsBoolean();
        autoAmp = m_Controllers.ampAutoAlignLeft.or(m_Controllers.ampAutoAlignRight).getAsBoolean();
        manualAmp = m_Controllers.manualArmAimButton.getAsBoolean();
        autoAmpTrigger = m_Controllers.ampAutoAlignLeft.or(m_Controllers.ampAutoAlignRight);

        xboxController = m_Controllers.m_XboxController;

        if (manualAmp) {
            m_FiringSubsystem.spinUpShooter(shooterManualAmp);
        } else if (autoAmp) {
            m_FiringSubsystem.spinUpIntake(intakeAutoAmpSpeed);
        } else if (shoot) {
            double speakerDist = m_LimelightSubsystem.getSpeakerDistance();
            double adujustmentSpeed = 0;
            if (speakerDist > 2) {
                adujustmentSpeed = (speakerDist - startDist) * slope / (endDist - startDist);
            }
            m_FiringSubsystem.spinUpShooter(shooterSpeed + adujustmentSpeed);
        } else if (intake) {
            m_FiringSubsystem.spinUpIntake(intakeSpeed);
        }

        timer = System.currentTimeMillis();
    }

    @Override
    public void execute() {
        if (manualAmp || shoot) {
            if (System.currentTimeMillis() - timer > waitTime) {
                m_FiringSubsystem.spinUpIntake(intakeSpeed);
            }
        }
    }

    @Override
    public boolean isFinished() {
        if (manualAmp || (shoot && !autoAmp)) {
            if (System.currentTimeMillis() - timer > waitTime + shootTime) {
                return true;
            }
        } else if (intake && !autoAmp) {
            if (!m_FiringSubsystem.noteSensor.get()) {
                new Rumble(xboxController, 0.5, 1).schedule();
                return true;
            }
        } else if (autoAmp && !autoAmpTrigger.getAsBoolean()) {
            return true;
        }

        return false;
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putString("Auto Firing Status", "Spinning Down");

        // Spin down motors
        m_FiringSubsystem.spinDownIntake();
        m_FiringSubsystem.spinDownShooter();
    }
}
