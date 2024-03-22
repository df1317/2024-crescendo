package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.FiringSubsystem;

public class AutoFireNote extends Command {
    private FiringSubsystem m_FiringSubsystem;
    private CommandXboxController xboxController;

    private boolean intake;
    private boolean shoot;

    private double timer;

    public AutoFireNote(FiringSubsystem FiringSub, boolean intake,
            boolean shoot, CommandXboxController xboxController) {
        m_FiringSubsystem = FiringSub;
        this.xboxController = xboxController;
        this.intake = intake;
        this.shoot = shoot;
        addRequirements(FiringSub);
    }

    @Override
    public void initialize() {
        SmartDashboard.putBoolean("Auto Firing Intake Status", intake);
        SmartDashboard.putBoolean("Auto Firing Shooter Status", shoot);

        if (intake && m_FiringSubsystem.noteSensor.get()) {
            m_FiringSubsystem.spinUpIntake(Constants.ArmShooterConstants.ShooterCollectorConstants.Intake.Speed);

            timer = System.currentTimeMillis();
        }
        if (shoot) {
            m_FiringSubsystem.spinUpShooter(Constants.ArmShooterConstants.ShooterCollectorConstants.Firing.Speed);

            timer = System.currentTimeMillis();
        }
    }

    @Override
    public void execute() {
        if (intake && !m_FiringSubsystem.noteSensor.get()) {
            new Rumble(xboxController, 0.5, 1).schedule();
        }

        if (shoot && System.currentTimeMillis()
                - timer > Constants.ArmShooterConstants.ShooterCollectorConstants.Firing.Duration
                        * 1000) {
            m_FiringSubsystem
                    .spinUpIntake(Constants.ArmShooterConstants.ShooterCollectorConstants.Intake.ShootSpeed);
        }
    }

    @Override
    public boolean isFinished() {
        // check for the beam break sensor to be tripped and the clearing delay to have
        // been met

        if (shoot && m_FiringSubsystem.noteSensor.get() && System.currentTimeMillis()
                - timer > (Constants.ArmShooterConstants.ShooterCollectorConstants.Firing.ClearingDelay
                        + Constants.ArmShooterConstants.ShooterCollectorConstants.Firing.Duration) * 1000) {
            return true;
        }

        if (intake && !m_FiringSubsystem.noteSensor.get() || System.currentTimeMillis()
                - timer > Constants.ArmShooterConstants.ShooterCollectorConstants.Intake.Timeout * 1000) {
            return true;
        }

        return false;
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("Auto Firing Intake Status", false);
        SmartDashboard.putBoolean("Auto Firing Shooter Status", false);

        // Spin down motors
        m_FiringSubsystem.spinDownIntake();
        m_FiringSubsystem.spinDownShooter();
    }
}
