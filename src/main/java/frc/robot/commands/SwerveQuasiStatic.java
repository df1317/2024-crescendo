package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveQuasiStatic extends Command {

    private SwerveSubsystem m_SwerveSubsystem;

    private Trigger endButton;
    private double ampShootAngle = 18;
    private SysIdRoutine.Direction direction;
    // private boolean buttonReleased = false;
    // private boolean end = false;
    // private double timer;

    public SwerveQuasiStatic(SwerveSubsystem swerveSubsystem, Trigger endButton, SysIdRoutine.Direction direction) {
        m_SwerveSubsystem = swerveSubsystem;
        this.endButton = endButton;
        this.direction = direction;

        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        m_SwerveSubsystem.sysIdQuasistatic(direction);
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return !endButton.getAsBoolean();
    }

    @Override
    public void end(boolean interrupted) {
        m_SwerveSubsystem.driveRobotRelative(
                new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}
