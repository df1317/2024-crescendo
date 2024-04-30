package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Controllers;
import frc.robot.subsystems.ArmSubsystem;

public class FixedAim extends Command {

    private ArmSubsystem m_ArmSubsystem;
    private Controllers m_Controllers;
    private double armAngle = 55;
    private boolean auto;
    private Timer timer = new Timer();

    public FixedAim(ArmSubsystem ArmSub, Controllers Controllers) {
        m_ArmSubsystem = ArmSub;
        m_Controllers = Controllers;
        addRequirements(ArmSub);
    }

    @Override
    public void initialize() {
        auto = DriverStation.isAutonomous();
        timer.reset();
        timer.start();
        m_ArmSubsystem.setAngle(armAngle);
        SmartDashboard.putBoolean("Aut Moving Arm", true);
    }

    @Override
    public void execute() {
        m_ArmSubsystem.runPID();
    }

    @Override
    public boolean isFinished() {
        if (auto) {
            return timer.hasElapsed(4);
        } else {
            return !(m_Controllers.leftAutoAlignArmButton).or(m_Controllers.rightAutoAlignArmButton).getAsBoolean();
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_ArmSubsystem.setAngle(65);
        m_ArmSubsystem.spinDown();

        SmartDashboard.putBoolean("Aut Moving Arm", false);
    }
}
