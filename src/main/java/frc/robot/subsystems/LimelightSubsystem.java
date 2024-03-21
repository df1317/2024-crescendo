package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;

public class LimelightSubsystem extends SubsystemBase {

    public Pose3d botpose3d;
    private double[] botposeArray;

    /**
     * Updates the values of the Limelight camera and posts them to the
     * SmartDashboard.
     */
    public void updateLimelight() {
        botposeArray = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose")
                .getDoubleArray(new double[6]);

        botpose3d = new Pose3d(botposeArray[0], botposeArray[1], botposeArray[2], new Rotation3d(
                Math.toRadians(botposeArray[3]), Math.toRadians(botposeArray[4]), Math.toRadians(botposeArray[5])));

        // post to smart dashboard periodically
        SmartDashboard.putNumber("LimelightX", botpose3d.getX());
        SmartDashboard.putNumber("LimelightY", botpose3d.getY());
        SmartDashboard.putNumber("LimelightRotation", botpose3d.getRotation().toRotation2d().getDegrees());
        SmartDashboard.putNumber("Limelight Speaker Distance", getSpeakerDistance());
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
        return Math.sqrt(distX * distX + distY * distY) + 1.3;
    }

}