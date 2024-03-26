package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.ArmSubsystem;

public class AmpAlign extends Command {

    private ArmSubsystem m_ArmSubsystem;

    private Trigger endButton;
    private double ampShootAngle = 25;
    private boolean end;
    private double timer;

    public AmpAlign(ArmSubsystem ArmSub, Trigger endButton) {
        m_ArmSubsystem = ArmSub;
        this.endButton = endButton;

        addRequirements(ArmSub);
    }

    @Override
    public void initialize() {
        m_ArmSubsystem.setAngle(ampShootAngle);
    }

    @Override
    public void execute() {
        if (!endButton.getAsBoolean()) {
            end = true;
            timer = System.currentTimeMillis();
        }

        if (end) {
        }
        m_ArmSubsystem.runPID();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
