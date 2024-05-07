package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbingSubsystem;

public class RobotUnblock extends Command {

    private ClimbingSubsystem m_climbingSubsystem;
    private Timer timer;

    public RobotUnblock(ClimbingSubsystem ClimbingSub) {
        m_climbingSubsystem = ClimbingSub;
        addRequirements(ClimbingSub);
    }

    @Override
    public void initialize() {
        timer = new Timer();
        timer.start();

        m_climbingSubsystem.setLeftArm(0.75);
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(1);
    }

    @Override
    public void end(boolean interrupted) {
        m_climbingSubsystem.setLeftArm(0);
    }
}
