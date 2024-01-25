package frc.lib;

import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveModuleConstants {
  public final int driveMotorID;
  public final int angleMotorID;
  public final int cancoderID;
  public final Rotation2d angleOffset;
  public final double angleKP; 
  public final double angleKI;
  public final double angleKD;
  public final double angleKFF;

  /**
   * Swerve Module Constants to be used when creating swerve modules.
   *
   * @param driveMotorID
   * @param angleMotorID
   * @param canCoderID
   * @param angleOffset
   */
  public SwerveModuleConstants(
      int driveMotorID, int angleMotorID, int canCoderID, Rotation2d angleOffset, double angleKP, double angleKI, double angleKD, double angleKFF) {
    this.driveMotorID = driveMotorID;
    this.angleMotorID = angleMotorID;
    this.cancoderID = canCoderID;
    this.angleOffset = angleOffset;
    this.angleKP = angleKP;
    this.angleKI = angleKI;
    this.angleKD = angleKD;
    this.angleKFF = angleKFF;
  }
}