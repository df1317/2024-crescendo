package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose3d;
import frc.lib.LimelightHelpers;
import frc.lib.LimelightHelpers.LimelightResults;
import frc.robot.Constants;

public class LimelightSubsystem extends SubsystemBase {

    public Pose3d botpose3d = new Pose3d();
    public boolean hasTargets = false;

    /**
     * Updates the values of the Limelight camera and posts them to the
     * SmartDashboard.
     */
    public void updateLimelight() {
        LimelightResults llresults = LimelightHelpers.getLatestResults("");

        botpose3d = llresults.targetingResults.getBotPose3d();
        hasTargets = llresults.targetingResults.targets_Fiducials.length > 0;

        // post to smart dashboard periodically
        SmartDashboard.putNumber("LimelightX", botpose3d.getX());
        SmartDashboard.putNumber("LimelightY", botpose3d.getY());
        SmartDashboard.putNumber("LimelightRotation", botpose3d.getRotation().toRotation2d().getDegrees());
        SmartDashboard.putNumber("Limelight Speaker Distance", getSpeakerDistance());
        SmartDashboard.putBoolean("Limelight hasTargets", hasTargets);
    }

    /** returning distance from speaker to robot */
    public double getSpeakerDistance() {
        // get alliance speaker X posistion
        double speakerX = (DriverStation.getAlliance().orElseThrow() == DriverStation.Alliance.Blue)
                ? Constants.Field.speakerXBlue
                : Constants.Field.speakerXRed;
        // getting X and Y distance from the robot
        double distX = speakerX - botpose3d.getX();
        double distY = Constants.Field.speakerY - botpose3d.getY();
        // returning distance from speaker to robot
        return (Math.sqrt(distX * distX + distY * distY));
    }
}