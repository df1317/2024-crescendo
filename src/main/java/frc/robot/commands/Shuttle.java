package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Controllers;
import frc.robot.subsystems.ArmSubsystem;

public class Shuttle extends Command {

    private ArmSubsystem m_ArmSubsystem;
    private Controllers m_Controllers;
    private double armAngle = 42;

    public Shuttle(ArmSubsystem ArmSub, Controllers Controllers) {
        m_ArmSubsystem = ArmSub;
        m_Controllers = Controllers;
        addRequirements(ArmSub);
    }

    @Override
    public void initialize() {
        m_ArmSubsystem.setAngle(armAngle);
        SmartDashboard.putBoolean("Aut Moving Arm", true);
    }

    @Override
    public void execute() {
        m_ArmSubsystem.runPID();
    }

    @Override
    public boolean isFinished() {
        return !(m_Controllers.shuttleLeft).or(m_Controllers.shuttleRight).getAsBoolean();
    }

    @Override
    public void end(boolean interrupted) {
        m_ArmSubsystem.setAngle(65);
        m_ArmSubsystem.spinDown();

        SmartDashboard.putBoolean("Aut Moving Arm", false);
    }

}
