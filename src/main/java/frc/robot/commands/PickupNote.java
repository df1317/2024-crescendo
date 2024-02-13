package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.FiringSubsystem;

public class PickupNote extends Command {
    private FiringSubsystem m_FiringSubsystem;

    public PickupNote(FiringSubsystem FiringSub) {
        m_FiringSubsystem = FiringSub;
        addRequirements(FiringSub);
    }

    @Override
    public void initialize() {
        // spin up the arm
        m_FiringSubsystem.spinUp(Constants.ArmShooterConstants.ShooterCollectorConstants.Intake.Speed);
    }

    @Override
    public boolean isFinished() {
        // if note sensor is triggered return true
        if (m_FiringSubsystem.noteSensor.get()) {
            return true;
        }
        
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        // spin down the arm
        m_FiringSubsystem.spinDown();
    }

}
