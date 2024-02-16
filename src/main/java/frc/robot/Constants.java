// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.SwerveModuleConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class SensorConstants {
    public static final class Limelight {
      public static final double speakerAprilTag4TY = 1.442593;
      public static final double speakerAprilTag4TX = 8.308467;
      public static final double speakerAprilTag4TZ = 1.451102;
    }

    public static final class Controller {
      public static final double FeedbackDuration = 0.2;
      public static final double rumble = 0.3;
    }
  }

  public static final class ArmShooterConstants {

    public static final class ShooterCollectorConstants {
      public static final class Firing {
        public static final int MotorID0 = 01;
        public static final int MotorID1 = 02;
        public static final double NearSpeed = 0.35; // must be between -1 and 1
        public static final double FarSpeed = 1; // must be between -1 and 1
        public static final double Duration = 3.0;
      }

      public static final class Intake {
        public static final double Speed = 0.5; // must be between -1 and 1
      }

      public static final int NoteSensorPort = 0; // TBD
    }

    public static final class Arm {
      public static final int MotorID = 02;
      public static final double Speed = 0.5; // must be between -1 and 1
      public static final int EncoderPort = 0; // TBD
    }
  }

  public static final class ClimberConstants {
    public static final int MotorID = 04;
    public static final double Speed = 0.5; // must be between -1 and 1
  }

  public static final class SwerveConstants {
    public static final double inputDeadband = .1;
    public static final boolean invertGyro = true;

    /* Drivetrain Constants */
    public static final double trackWidth = Units.inchesToMeters(23);// to find
    public static final double wheelBase = Units.inchesToMeters(23);// to find
    public static final double wheelDiameter = Units.inchesToMeters(4.0);
    public static final double wheelCircumference = wheelDiameter * Math.PI;

    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    public static final double driveGearRatio = (6.12 / 1.0); // 6.75:1 L2 Mk4 Modules
    // L1 is 8.14:1, L2 is 6.75:1, L3 is 6.12:1, L4 is 5.14:1
    public static final double angleGearRatio = (12.8 / 1.0); // 12.8:1 MK4 SDS Modules

    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
        new Translation2d(wheelBase / 2.0, trackWidth / 2.0), // translation 2d locates the swerve module in cords
        new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));
    // https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html
    // SwerveDrive Kinematics converts between a ChassisSpeeds object and several
    // SwerveModuleState objects,
    // which contains velocities and angles for each swerve module of a swerve drive
    // robot.

    /* Swerve Voltage Compensation */
    public static final double voltageComp = 12.0;

    // Swerve Current Limiting for neos
    public static final int angleContinuousCurrentLimit = 20; // limits current draw of turning motor
    public static final int driveContinuousCurrentLimit = 80; // limits current draw of drive motor

    /* Drive Motor PID Values */
    public static final double driveKP = 0.1; // to tune
    public static final double driveKI = 0.0; // to tune
    public static final double driveKD = 0.0; // to tune
    public static final double driveKFF = 0.0; // to tune

    /* Drive Motor Characterization Values */
    // values to calculate the drive feedforward (KFF)
    public static final double driveKS = 0.667; // to calculate
    public static final double driveKV = 2.44; // to calculate
    public static final double driveKA = 0.27; // to calculate

    /* Drive Motor Conversion Factors */
    public static final double driveConversionPositionFactor = (wheelDiameter * Math.PI) / driveGearRatio;
    public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
    public static final double angleConversionFactor = 360.0 / angleGearRatio;

    /* Swerve Profiling Values */
    public static final double maxSpeed = 9; // meters per second
    public static final double maxAngularVelocity = 11.5; // these are for rotation

    /* Neutral Modes */
    public static final IdleMode angleNeutralMode = IdleMode.kBrake;
    public static final IdleMode driveNeutralMode = IdleMode.kBrake;

    /* Motor Inverts */
    public static final boolean driveInvert = false;
    public static final boolean angleInvert = false;

    /* Angle Encoder Invert */
    public static final boolean canCoderInvert = false;

    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class Mod0 {
      public static final int driveMotorID = 11;
      public static final int angleMotorID = 31;
      public static final int canCoderID = 51;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(180);
      /* Angle Motor PID Values */
      public static final double angleKP = 0.01; // to tune
      public static final double angleKI = 0.0; // to tune
      public static final double angleKD = 0.0; // to tune
      public static final double angleKFF = 0.0; // to tune

      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset, angleKP, angleKI, angleKD, angleKFF);
      // creates a constant with all info from swerve module
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 {
      public static final int driveMotorID = 12;
      public static final int angleMotorID = 32;
      public static final int canCoderID = 52;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(180);
      /* Angle Motor PID Values */
      public static final double angleKP = 0.01; // to tune
      public static final double angleKI = 0.0; // to tune
      public static final double angleKD = 0.0; // to tune
      public static final double angleKFF = 0.0; // to tune

      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset, angleKP, angleKI, angleKD, angleKFF);
      // creates a constant with all info from swerve module
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 {
      public static final int driveMotorID = 21;
      public static final int angleMotorID = 41;
      public static final int canCoderID = 61;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0);
      /* Angle Motor PID Values */
      public static final double angleKP = 0.01; // to tune
      public static final double angleKI = 0.0; // to tune
      public static final double angleKD = 0.0; // to tune
      public static final double angleKFF = 0.0; // to tune

      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset, angleKP, angleKI, angleKD, angleKFF);
      // creates a constant with all info from swerve module
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 {
      public static final int driveMotorID = 22;
      public static final int angleMotorID = 42;
      public static final int canCoderID = 62;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0);
      /* Angle Motor PID Values */
      public static final double angleKP = 0.01; // to tune
      public static final double angleKI = 0.0; // to tune
      public static final double angleKD = 0.0; // to tune
      public static final double angleKFF = 0.0; // to tune

      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset, angleKP, angleKI, angleKD, angleKFF);
      // creates a constant with all info from swerve module
    }

    public static final boolean angleMotorInvert = false;
    public static final boolean driveMotorInvert = false;

  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 1.75;
    public static final double kMaxAccelerationMetersPerSecondSquared = 2;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

}
