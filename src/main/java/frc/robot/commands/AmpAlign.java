package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.ArmSubsystem;

public class AmpAlign extends Command {

    private ArmSubsystem m_ArmSubsystem;

    private Trigger endButton;
    private double ampShootAngle = 25;
    private boolean buttonReleased;
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

    private double lerp(double floor, double ceiling, double lerp) {
        // clamp lerp from 0-1
        if (lerp > 1) {
            lerp = 1;
        } else if (lerp < 0) {
            lerp = 0;
        }

        return floor + (ceiling - floor) * lerp;
    }

    @Override
    public void execute() {
        if (!endButton.getAsBoolean()) {
            buttonReleased = true;
            timer = System.currentTimeMillis();
        }

        if (buttonReleased) {
            if (System.currentTimeMillis() - timer < 1000) {
                m_ArmSubsystem.setAngle(lerp(ampShootAngle, 0, (System.currentTimeMillis() - timer) / 1000));
            } else {
                end = true;
            }
        }
        m_ArmSubsystem.runPID();
    }

    @Override
    public boolean isFinished() {
        return end;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
