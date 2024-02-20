package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.Constants;

public class SetArmValueAbsolute extends Command {
    private ArmSubsystem m_ArmSubsystem;
    private double target_value;
    private double encoderShooterZero;

    public SetArmValueAbsolute(ArmSubsystem ArmSub, double value) {
        m_ArmSubsystem = ArmSub;
        target_value = encoderShooterZero + (value / 360);
        addRequirements(ArmSub);
    }

    @Override
    public void initialize() {
        m_ArmSubsystem.spinUp(
                Constants.ArmShooterConstants.Arm.Speed * Math.floor((target_value - m_ArmSubsystem.encoder.get())));
    }

    @Override
    public boolean isFinished() {
        if ((m_ArmSubsystem.encoder.get() >= Constants.ArmShooterConstants.Arm.EncoderMax && target_value < 0)
                || (m_ArmSubsystem.encoder.get() <= Constants.ArmShooterConstants.Arm.EncoderMin && target_value > 0)) {

            return true;
        }

        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_ArmSubsystem.spinDown();
    }
}
