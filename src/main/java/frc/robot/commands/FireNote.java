package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.FiringSubsystem;
import frc.robot.Constants;

public class FireNote extends Command {
    private double startTime;
    private double duration = Constants.SwerveConstants.Firing.Duration;
    private double speed;
    private Trigger button;

    private FiringSubsystem m_FiringSubsystem;

    public FireNote(FiringSubsystem FiringSub, boolean far, Trigger button) {
        m_FiringSubsystem = FiringSub;
        this.duration = Constants.SwerveConstants.Firing.Duration;
        this.button = button;
        addRequirements(FiringSub);
        if (far) {
            speed = Constants.SwerveConstants.Firing.FarSpeed;
        } else {
            speed = Constants.SwerveConstants.Firing.NearSpeed;
        }
    }

    @Override
    public void initialize() {
        startTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        // Spin up motors to the specified speed
        m_FiringSubsystem.spinUp(speed);
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        // Check if the button is released or if the specified duration has passed
        return !button.getAsBoolean() ||
                (edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - startTime >= duration);
    }

    @Override
    public void end(boolean interrupted) {
        // Spin down motors
        m_FiringSubsystem.spinDown();
    }
}
