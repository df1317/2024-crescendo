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
        SmartDashboard.putNumber("Limelight Floor Angle", getFloorAngle());
        SmartDashboard.putNumber("Limelight Elevation Angle", getElevation());
        SmartDashboard.putNumber("LimelightRotation", botpose3d.getRotation().toRotation2d().getDegrees());
    }

    public double getFloorAngle() {
        double op;
        double adj;
        if (DriverStation.getAlliance().orElseThrow() == DriverStation.Alliance.Blue) {
            op = botpose3d.getY() - Constants.SensorConstants.Limelight.speakerYBlue;
            adj = botpose3d.getX() - Constants.SensorConstants.Limelight.speakerXBlue;
        } else {
            op = botpose3d.getY() - Constants.SensorConstants.Limelight.speakerYRed;
            adj = botpose3d.getX() - Constants.SensorConstants.Limelight.speakerXRed;
        }

        SmartDashboard.putNumber("Limelight floor op", op);
        SmartDashboard.putNumber("Limelight Floor adj", adj);

        double trig;
        if (botpose3d.getY() == 0 && botpose3d.getX() == 0) {
            trig = 0;
        } else {
            trig = Math.atan(op / adj);
        }

        return -(Math.toDegrees(trig) - botpose3d.getRotation().toRotation2d().getDegrees());
    }

    public double getElevation() {
        // double armOffSetAtBottom = 6 degrees
        // shooter to arm offset = 60 degrees

        double op;
        double adj;

        if (DriverStation.getAlliance().orElseThrow() == DriverStation.Alliance.Blue) {
            op = botpose3d.getZ() - Constants.SensorConstants.Limelight.speakerZBlue;
            adj = botpose3d.getX() - Constants.SensorConstants.Limelight.speakerXBlue;
        } else {
            op = botpose3d.getZ() - Constants.SensorConstants.Limelight.speakerZRed;
            adj = botpose3d.getX() - Constants.SensorConstants.Limelight.speakerXRed;
        }

        double trig;
        if (botpose3d.getZ() == 0 && botpose3d.getX() == 0) {
            trig = 0;
        } else {
            trig = Math.atan(op / adj);
        }

        return Math.toDegrees(trig);
    }
}