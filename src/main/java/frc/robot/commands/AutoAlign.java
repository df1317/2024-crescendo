package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoAlign extends Command {
    private LimelightSubsystem m_LimelightSubsystem;
    private SwerveSubsystem m_SwerveSubsystem;

    private Trigger endTrigger;

    private PIDController pidController = new PIDController(0, 0, 0);

    public AutoAlign(LimelightSubsystem limelightSub, SwerveSubsystem swerveSub, Trigger endTrigger) {
        this.m_LimelightSubsystem = limelightSub;
        this.m_SwerveSubsystem = swerveSub;
        this.endTrigger = endTrigger;
    }

    public double getAlignedRotation() {
        double speakerX = (DriverStation.getAlliance().orElseThrow() == DriverStation.Alliance.Blue)
                ? Constants.Field.speakerXBlue
                : Constants.Field.speakerXRed;
        // getting X and Y distance from the robot
        double distX = speakerX - m_LimelightSubsystem.botpose3d.getX();
        double distY = Constants.Field.speakerY - m_LimelightSubsystem.botpose3d.getY();

        return Math.atan2(distX, distY);
    }

    @Override
    public void execute() {
        double power = pidController.calculate(getAlignedRotation(),
                m_LimelightSubsystem.botpose3d.getRotation().getAngle());

        m_SwerveSubsystem.drive(null, power, false, false);
    }

    @Override
    public boolean isFinished() {
        // Check if the button is released or if the specified duration has passed
        return !endTrigger.getAsBoolean();
    }

    @Override
    public void end(boolean interrupted) {
        m_SwerveSubsystem.drive(null, 0, false, false);
    }
}
