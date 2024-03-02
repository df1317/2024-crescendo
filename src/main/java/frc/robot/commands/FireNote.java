package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.FiringSubsystem;

public class FireNote extends Command {
    private boolean intake;
    private boolean flywheels;
    private Trigger button;
    private CommandJoystick joystick;

    private FiringSubsystem m_FiringSubsystem;

    public FireNote(FiringSubsystem FiringSub, Trigger button, CommandJoystick joystick, boolean intake,
            boolean flywheels) {
        m_FiringSubsystem = FiringSub;
        this.button = button;
        this.joystick = joystick;
        this.intake = intake;
        this.flywheels = flywheels;
        addRequirements(FiringSub);
    }

    @Override
    public void initialize() {
        SmartDashboard.putNumber("Joystick R Throttle", (joystick.getThrottle() - 0.5) * 2);

        if (intake) {
            m_FiringSubsystem.spinUpIntake((joystick.getThrottle() - 0.5) * 2);
        }
        if (flywheels) {
            m_FiringSubsystem.spinUpShooter((joystick.getThrottle() - 0.5) * 2);
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
