package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoAlign extends Command {

    private LimelightSubsystem m_LimelightSubsystem;
    private ArmSubsystem m_ArmSubsystem;
    @SuppressWarnings("unused")
    private Command SetArmValue;
    private SwerveSubsystem m_SwerveSubsystem;
    private CommandXboxController m_XboxController;
    private double duration = Constants.SensorConstants.Controller.FeedbackDuration;
    private double startTime;
    private boolean canAlign;

    public AutoAlign(LimelightSubsystem LimelightSub, SwerveSubsystem SwerveSub, ArmSubsystem ArmSub,
            CommandXboxController XboxCont) {
        m_LimelightSubsystem = LimelightSub;
        m_XboxController = XboxCont;
        m_SwerveSubsystem = SwerveSub;
        m_ArmSubsystem = ArmSub;
        addRequirements(LimelightSub, SwerveSub);
        this.duration = Constants.SensorConstants.Controller.FeedbackDuration;
    }

    @Override
    public void initialize() {
        // get the angles from limelight
        double floorAngle;
        if (m_LimelightSubsystem.getFloorAngle() > 0) {
            floorAngle = 1;
        } else if (m_LimelightSubsystem.getFloorAngle() < 0) {
            floorAngle = -1;
        } else {
            floorAngle = 0;
        }

        SmartDashboard.putNumber("Floor Angle", floorAngle);

        double elevation = m_LimelightSubsystem.getElevation();

        canAlign = floorAngle != 0;

        SmartDashboard.putBoolean("Can Align", canAlign);

        // turn the robot to the correct angle
        if (!canAlign) {
            m_XboxController.getHID().setRumble(RumbleType.kBothRumble, Constants.SensorConstants.Controller.rumble);
            startTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        } else {
            m_XboxController.getHID().setRumble(RumbleType.kBothRumble, 0);
            m_ArmSubsystem.setAngle(elevation + Constants.ArmShooterConstants.ShooterCollectorConstants.Intake.angle);
            m_ArmSubsystem.runPID();
            m_SwerveSubsystem.drive(new Translation2d(0, 0), 1 * floorAngle, true, true);
        }
    }

    @Override
    public boolean isFinished() {
        // Check if the button is released or if the specified duration has passed
        return (m_LimelightSubsystem.getFloorAngle() > -0.25 && m_LimelightSubsystem.getFloorAngle() < 0.25 && canAlign)
                ||
                (edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - startTime >= duration && !canAlign);
    }

    @Override
    public void end(boolean interrupted) {
        m_XboxController.getHID().setRumble(RumbleType.kBothRumble, 0);
        m_SwerveSubsystem.drive(new Translation2d(0, 0), 0, true, true);
    }
}
