package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
        this.intake = intake;
        this.shoot = shoot;
        addRequirements(FiringSub);
    }

    @Override
    public void initialize() {
        SmartDashboard.putBoolean("Auto Firing Intake Status", intake);
        SmartDashboard.putBoolean("Auto Firing Shooter Status", shoot);

        // log joystick values
        SmartDashboard.putNumber("Joystick L Throttle", joystickL.getThrottle());

        if (intake) {
            m_FiringSubsystem.spinUpIntake(Constants.ArmShooterConstants.ShooterCollectorConstants.Intake.Speed);
        }
        if (shoot) {
            m_FiringSubsystem.spinUpShooter(joystickR.getThrottle());

            timer = System.currentTimeMillis();
        }
    }

    @Override
    public void execute() {
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

        if (intake && !m_FiringSubsystem.noteSensor.get()) {
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
