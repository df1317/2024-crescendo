package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Controllers;
import frc.robot.subsystems.FiringSubsystem;

public class AutoFireNote extends Command {
    private FiringSubsystem m_FiringSubsystem;
    private CommandXboxController xboxController;

    private Controllers m_Controllers;

    private boolean intake;
    private boolean shoot;
    private boolean autoAmp;
    private Trigger autoAmpTrigger;
    private boolean manualAmp;

    private double timer;
    private double intakeSpeed = -0.6;
    private double intakeAutoAmpSpeed = 1;
    private double shooterSpeed = -3000;
    private double shooterManualAmp = -2000;

    private double waitTime = 2000;
    private double shootTime = 1000;

    public AutoFireNote(FiringSubsystem FiringSub, Controllers controllers) {
        m_FiringSubsystem = FiringSub;
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

        System.out.println("intake: " + intake);
        System.out.println("shoot: " + shoot);
        System.out.println("autoAmp: " + autoAmp);
        System.out.println("manualAmp: " + manualAmp);
        System.out.println("autoAmpTrigger: " + autoAmpTrigger);

        if (manualAmp) {
            m_FiringSubsystem.spinUpShooter(shooterManualAmp);
        } else if (autoAmp) {
            m_FiringSubsystem.spinUpIntake(intakeAutoAmpSpeed);
        } else if (shoot) {
            m_FiringSubsystem.spinUpShooter(shooterSpeed);
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
