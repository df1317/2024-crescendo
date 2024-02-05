package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.LimelightHelpers;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose3d;

import frc.robot.Constants;

public class LimelightSubsystem extends SubsystemBase {

    public Pose3d botpose3d;

    /**
     * Updates the values of the Limelight camera and posts them to the
     * SmartDashboard.
     */
    public void updateLimelight() {
        botpose3d =  LimelightHelpers.getBotPose3d("");

        // post to smart dashboard periodically
        SmartDashboard.putNumber("LimelightX", botpose3d.getTranslation().getX());
        SmartDashboard.putNumber("LimelightY", botpose3d.getTranslation().getY());
        SmartDashboard.putNumber("Limelight Floor Angle", getFloorAngle());
        SmartDashboard.putNumber("Limelight Elevation Angle", getElevation());
    }

    public double getFloorAngle() {
        // x = 5.93 , y = 1.3
        double op = botpose3d.getY() - Constants.SensorConstants.Limelight.speakerAprilTag3TY;
        double adj = botpose3d.getX() - Constants.SensorConstants.Limelight.speakerAprilTag3TX;

        double trig;
        if (op == 0 || adj == 0) {
            trig = 0;
        } else {
            trig = Math.atan(op/adj);
        }
        
        return Math.toDegrees(trig);
    }

    public double getElevation() {
        // x = 5.93 , z = 
        double adj = botpose3d.getX() - Constants.SensorConstants.Limelight.speakerAprilTag3TX;
        double op = botpose3d.getZ() - Constants.SensorConstants.Limelight.speakerAprilTag3TZ;
        
        double trig;
        if (op == 0 || adj == 0) {
            trig = 0;
        } else {
            trig = Math.atan(op/adj);
        }

        return Math.toDegrees(trig);
    }
}