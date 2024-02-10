package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoAlign extends Command {

    private LimelightSubsystem m_LimelightSubsystem;
    private SwerveSubsystem m_SwerveSubsystem;
    private CommandXboxController m_XboxController;
    private double duration = Constants.SensorConstants.Controller.FeedbackDuration;
    private double startTime;
    private boolean canAlign;

    public AutoAlign(LimelightSubsystem LimelightSub, SwerveSubsystem SwerveSub, CommandXboxController XboxCont) {
        m_LimelightSubsystem = LimelightSub;
        m_XboxController = XboxCont;
        m_SwerveSubsystem = SwerveSub;
        addRequirements(LimelightSub, SwerveSub);
        this.duration = Constants.SensorConstants.Controller.FeedbackDuration;
    }

    @Override
    public void initialize() {
        // get the angles from limelight
        double floorAngle = m_LimelightSubsystem.getFloorAngle();
        double elevation = m_LimelightSubsystem.getElevation();
        // double floorAngle = 0;
        // double elevation = 0;
        canAlign = floorAngle != 0 && elevation != 0;

        // turn the robot to the correct angle
        if (!canAlign) {
            m_XboxController.getHID().setRumble(RumbleType.kBothRumble, Constants.SensorConstants.Controller.rumble);
            startTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        } else {
            m_XboxController.getHID().setRumble(RumbleType.kBothRumble, 0);
            m_SwerveSubsystem.drive(new Translation2d(0,0), floorAngle, true, false);
        }
    }

    @Override
    public boolean isFinished() {
        // Check if the button is released or if the specified duration has passed
        return (m_LimelightSubsystem.getFloorAngle() == 0 && canAlign && m_LimelightSubsystem.getElevation() == 0) ||
            (edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - startTime >= duration && !canAlign);
    }

    @Override
    public void end(boolean interrupted) {
        // Spin down motors
        m_SwerveSubsystem.drive(new Translation2d(0,0), 0, true, false);
        m_XboxController.getHID().setRumble(RumbleType.kBothRumble, 0);
    }
}