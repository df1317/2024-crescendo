package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoAlign extends Command {

    private LimelightSubsystem m_LimelightSubsystem;
    private SwerveSubsystem m_SwerveSubsystem;

    public AutoAlign(LimelightSubsystem LimelightSub, SwerveSubsystem SwerveSub) {
        m_LimelightSubsystem = LimelightSub;
        m_SwerveSubsystem = SwerveSub;
        addRequirements(LimelightSub, SwerveSub);
    }

    @Override
    public void initialize() {
        // get the angles from limelight
        double floorAngle = m_LimelightSubsystem.getFloorAngle();
        double elevation = m_LimelightSubsystem.getElevation();

        // turn the robot to the correct angle
        m_SwerveSubsystem.drive(new Translation2d(0,0), elevation, true, false);
        // tilt the robot to the correct angle
        // TBI
    }

    @Override
    public boolean isFinished() {
        // Check if the button is released or if the specified duration has passed
        return m_LimelightSubsystem.getFloorAngle() == 0;
    }

    @Override
    public void end(boolean interrupted) {
        // Spin down motors
        m_SwerveSubsystem.drive(new Translation2d(0,0), 0, true, false);
    }
}
