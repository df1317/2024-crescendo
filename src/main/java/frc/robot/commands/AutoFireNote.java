package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants;
import frc.robot.subsystems.FiringSubsystem;

public class AutoFireNote extends Command {
    private FiringSubsystem m_FiringSubsystem;

    private boolean intake;
    private boolean shoot;

    private CommandJoystick joystickL;
    private CommandJoystick joystickR;

    private double timer;

    public AutoFireNote(FiringSubsystem FiringSub, CommandJoystick joystickL, CommandJoystick joystickR, boolean intake,
            boolean shoot) {
        m_FiringSubsystem = FiringSub;
        this.joystickL = joystickL;
        this.joystickR = joystickR;
        addRequirements(FiringSub);
    }

    @Override
    public void initialize() {
        if (intake) {
            m_FiringSubsystem.spinUpIntake(joystickL.getThrottle());
        }
        if (shoot) {
            m_FiringSubsystem.spinUpShooter(joystickR.getThrottle());

            timer = System.currentTimeMillis();
        }
    }

    @Override
    public void execute() {
        m_FiringSubsystem.logVals();

        if (shoot && System.currentTimeMillis()
                - timer > Constants.ArmShooterConstants.ShooterCollectorConstants.Firing.Duration
                        * 1000) {
            m_FiringSubsystem.spinUpIntake(joystickL.getThrottle());
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

        if (intake && m_FiringSubsystem.noteSensor.get()) {
            return true;
        }

        return false;
    }

    @Override
    public void end(boolean interrupted) {
        // Spin down motors
        m_FiringSubsystem.spinDownIntake();
        m_FiringSubsystem.spinDownShooter();
    }
}
