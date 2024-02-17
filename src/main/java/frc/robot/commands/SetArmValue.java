package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.Constants;

public class SetArmValue extends Command {
    private ArmSubsystem m_ArmSubsystem;
    private CommandJoystick m_Joystick;

    public SetArmValue(ArmSubsystem ArmSub, CommandJoystick joystick) {
        m_ArmSubsystem = ArmSub;
        m_Joystick = joystick;
        addRequirements(ArmSub);
    }

    @Override
    public void initialize() {
        SmartDashboard.putBoolean("Arm Command", true);
    }

    @Override
    public void execute() {
        m_ArmSubsystem.spinUp(Constants.ArmShooterConstants.Arm.Speed * m_Joystick.getRawAxis(Joystick.AxisType.kY.value));
        SmartDashboard.putNumber("Arm Value", Constants.ArmShooterConstants.Arm.Speed * m_Joystick.getRawAxis(Joystick.AxisType.kY.value));
    }

    @Override
    public boolean isFinished() {
        if (m_ArmSubsystem.encoder.get() >= Constants.ArmShooterConstants.Arm.EncoderMax || m_ArmSubsystem.encoder.get() <= Constants.ArmShooterConstants.Arm.EncoderMin  || m_Joystick.button(0).getAsBoolean()) {
            m_ArmSubsystem.spinDown();
            return true;
        }

        return false;
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("Arm Command", false);
        m_ArmSubsystem.spinDown();
    }
}
