package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.subsystems.ClimbingSubsystem;
import frc.robot.Constants;
import frc.robot.Controllers;

public class Climb extends Command {
    private ClimbingSubsystem m_ClimbingSubsystem;
    private Controllers m_Controllers;

    private boolean getTrigger() {
        return !m_Controllers.rightClimberButtonState() && !m_Controllers.leftClimberButtonState();
    }

    public Climb(ClimbingSubsystem climbingSub, Controllers m_Controllers) {
        m_ClimbingSubsystem = climbingSub;
        this.m_Controllers = m_Controllers;
        addRequirements(climbingSub);
    }

    @Override
    public void initialize() {
        SmartDashboard.putBoolean("Climbing", true);
    }

    @Override
    public void execute() {
        SmartDashboard.putBoolean("Climbing", true);
        if (m_Controllers.leftJoystickPosistion() != 0) {
            m_ClimbingSubsystem
                    .setLeftArm(m_Controllers.leftJoystickPosistion() * Constants.ClimberConstants.Speed);
            SmartDashboard.putNumber("Climbing Joystick Left", m_Controllers.leftJoystickPosistion());
        }
        if (m_Controllers.rightJoystickPosistion() != 0) {
            m_ClimbingSubsystem
                    .setRightArm(m_Controllers.rightJoystickPosistion() * Constants.ClimberConstants.Speed);
            SmartDashboard.putNumber("Climbing Joystick Right", m_Controllers.rightJoystickPosistion());
        }
    }

    @Override
    public boolean isFinished() {
        if (getTrigger()) {
            return true;
        }

        return false;
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("Climbing", false);
        m_ClimbingSubsystem.stop();
    }
}
