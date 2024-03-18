package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Rumble extends Command {
    private double timer;
    private double duration;
    private double intensity;

    private CommandXboxController xboxController;

    public Rumble(CommandXboxController xboxController, double duration, double intensity) {
        this.xboxController = xboxController;
        this.duration = duration;
        this.intensity = intensity;
    }

    @Override
    public void initialize() {
        timer = System.currentTimeMillis();
        xboxController.getHID().setRumble(RumbleType.kBothRumble, intensity);
    }

    @Override
    public boolean isFinished() {
        // check for the beam break sensor to be tripped and the clearing delay to have
        // been met

        if (System.currentTimeMillis() - timer > duration * 1000) {
            return true;
        }

        return false;
    }

    @Override
    public void end(boolean interrupted) {
        xboxController.getHID().setRumble(RumbleType.kBothRumble, 0);
    }
}
