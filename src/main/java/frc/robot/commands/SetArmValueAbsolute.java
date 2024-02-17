package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.Constants;

public class SetArmValueAbsolute extends Command {
    private ArmSubsystem m_ArmSubsystem;
    private double target_value;

    public SetArmValueAbsolute(ArmSubsystem ArmSub, double value) {
        m_ArmSubsystem = ArmSub;
        target_value = value;
        addRequirements(ArmSub);
    }

    @Override
    public void initialize() {
        m_ArmSubsystem.spinUp(Constants.ArmShooterConstants.Arm.Speed);
    }

    @Override
    public boolean isFinished() {
        if (m_ArmSubsystem.encoder.get() >= Constants.ArmShooterConstants.Arm.EncoderMax || m_ArmSubsystem.encoder.get() <= Constants.ArmShooterConstants.Arm.EncoderMin  || m_ArmSubsystem.encoder.get() == target_value) {
            m_ArmSubsystem.spinDown();
            return true;
        }

        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_ArmSubsystem.spinDown();
    }
}
