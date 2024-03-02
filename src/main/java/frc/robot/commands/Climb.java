package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.subsystems.ClimbingSubsystem;
import frc.robot.Constants;

public class Climb extends Command {
    private ClimbingSubsystem m_ClimbingSubsystem;
    private CommandJoystick m_JoystickL;
    private CommandJoystick m_JoystickR;

    private boolean getTrigger() {
        return !m_JoystickL.trigger().getAsBoolean() && !m_JoystickR.trigger().getAsBoolean();
    };

    public Climb(ClimbingSubsystem climbingSub, CommandJoystick joystickL, CommandJoystick joystickR) {
        m_ClimbingSubsystem = climbingSub;
        m_JoystickL = joystickL;
        m_JoystickR = joystickR;
        addRequirements(climbingSub);
    }

    @Override
    public void initialize() {
        SmartDashboard.putBoolean("Climbing", true);
    }

    @Override
    public void execute() {
        if (m_JoystickL.getRawAxis(Joystick.AxisType.kY.value) != 0) {
            m_ClimbingSubsystem
                    .setLeftArm(m_JoystickL.getRawAxis(Joystick.AxisType.kY.value) * Constants.ClimberConstants.Speed);
            SmartDashboard.putNumber("Climbing Joystick Left", m_JoystickL.getRawAxis(Joystick.AxisType.kY.value));
        }
        if (m_JoystickR.getRawAxis(Joystick.AxisType.kY.value) != 0) {
            m_ClimbingSubsystem
                    .setRightArm(m_JoystickR.getRawAxis(Joystick.AxisType.kY.value) * Constants.ClimberConstants.Speed);
            SmartDashboard.putNumber("Climbing Joystick Right", m_JoystickR.getRawAxis(Joystick.AxisType.kY.value));
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
