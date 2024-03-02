package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.FiringSubsystem;

public class FireNote extends Command {
    private boolean intake;
    private boolean flywheels;
    private double speed = 1;
    private Trigger button;

    private FiringSubsystem m_FiringSubsystem;

    public FireNote(FiringSubsystem FiringSub, Trigger button, boolean intake, boolean flywheels) {
        m_FiringSubsystem = FiringSub;
        this.button = button;
        this.intake = intake;
        this.flywheels = flywheels;
        addRequirements(FiringSub);
    }

    @Override
    public void initialize() {
        if (intake) {
            m_FiringSubsystem.spinUpIntake(speed);
        }
        if (flywheels) {
            m_FiringSubsystem.spinUpShooter(speed);
        }
    }

    @Override
    public void execute() {
        m_FiringSubsystem.logVals();
    }

    @Override
    public boolean isFinished() {
        // Check if the button is released or if the specified duration has passed
        return !button.getAsBoolean();
    }

    @Override
    public void end(boolean interrupted) {
        // Spin down motors
        m_FiringSubsystem.spinDownIntake();
        m_FiringSubsystem.spinDownShooter();
    }
}
