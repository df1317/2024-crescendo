// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DriverStation;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

public class SwerveSubsystem extends SubsystemBase {
  private final ADXRS450_Gyro gyro;

  private SwerveDriveOdometry swerveOdometry;
  private SwerveModule[] mSwerveMods;

  private Field2d field;

  SysIdRoutine routine;

  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem() {
    // instantiates new gyro, wipes it, and zeros it
    gyro = new ADXRS450_Gyro();
    zeroGyro();

    // Creates all four swerve modules into a swerve drive
    mSwerveMods = new SwerveModule[] {
        new SwerveModule(0, Constants.SwerveConstants.Mod0.constants),
        new SwerveModule(1, Constants.SwerveConstants.Mod1.constants),
        new SwerveModule(2, Constants.SwerveConstants.Mod2.constants),
        new SwerveModule(3, Constants.SwerveConstants.Mod3.constants)
    };

    routine = new SysIdRoutine(new SysIdRoutine.Config(),
        new SysIdRoutine.Mechanism(this::voltageDrive, this::logMotors, this));
    // creates new swerve odometry (odometry is where the robot is on the field)
    swerveOdometry = new SwerveDriveOdometry(Constants.SwerveConstants.swerveKinematics, getYaw(), getPositions());

    // puts out the field
    field = new Field2d();
    SmartDashboard.putData("Field", field);

    setWheelsToX();

    AutoBuilder.configureHolonomic(
        this::getPose, // Robot pose supplier
        this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
            new PIDConstants(1.0, 0.0, 0.0), // Translation PID constants
            new PIDConstants(1.0, 0.0, 0.0), // Rotation PID constants
            4.5, // Max module speed, in m/s
            0.4, // Drive base radius in meters. Distance from robot center to furthest module.
            new ReplanningConfig() // Default path replanning config. See the API for the options here
        ),
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this // Reference to this subsystem to set requirements
    );

  }

  public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    // Display whether the control is field relative on SmartDashboard
    SmartDashboard.putBoolean("Field Relative", fieldRelative);

    // Calculate swerve module states based on control mode (field-relative or
    // robot-centric)
    SwerveModuleState[] swerveModuleStates;

    SmartDashboard.putNumber("Gyro Yaw", getYaw().getDegrees());
    SmartDashboard.putNumber("Swerve rotation", rotation);

    // log values of translation as graphs
    SmartDashboard.putNumber("Translation X", translation.getX());
    SmartDashboard.putNumber("Translation Y", translation.getY());

    if (fieldRelative) {
      // Use field-relative control if fieldRelative is true
      swerveModuleStates = Constants.SwerveConstants.swerveKinematics.toSwerveModuleStates(
          ChassisSpeeds.fromFieldRelativeSpeeds(
              translation.getX(), translation.getY(), -rotation, getYaw()));
    } else {
      // Use robot-centric control if fieldRelative is false
      swerveModuleStates = Constants.SwerveConstants.swerveKinematics.toSwerveModuleStates(
          new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
    }

    // Set to top speed if above top speed
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.SwerveConstants.maxSpeed);

    // Set states for all 4 modules
    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {
    Translation2d translation = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    double rotation = speeds.omegaRadiansPerSecond;
    drive(translation, rotation, false, false);

  }

  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.SwerveConstants.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

  public Pose2d getPose() {
    return swerveOdometry.getPoseMeters();
  }

  public void resetPose(Pose2d Pose) {
    swerveOdometry.resetPosition(getYaw(), getPositions(), Pose);
  }

  public void resetOdometry(Pose2d pose) {
    swerveOdometry.resetPosition(getYaw(), getPositions(), pose);
  }

  public void setWheelsToX() {
    setModuleStates(new SwerveModuleState[] {
        // front left
        new SwerveModuleState(0.0, Rotation2d.fromDegrees(mSwerveMods[0].getCanCoder().getDegrees())),
        // front right
        new SwerveModuleState(0.0, Rotation2d.fromDegrees(mSwerveMods[1].getCanCoder().getDegrees())),
        // back left
        new SwerveModuleState(0.0, Rotation2d.fromDegrees(mSwerveMods[2].getCanCoder().getDegrees())),
        // back right
        new SwerveModuleState(0.0, Rotation2d.fromDegrees(mSwerveMods[3].getCanCoder().getDegrees()))
    });
  }

  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : mSwerveMods) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (SwerveModule mod : mSwerveMods) {
      positions[mod.moduleNumber] = mod.getPosition();
    }
    return positions;
  }

  public void zeroGyro() {
    gyro.reset();
  }

  public Rotation2d getYaw() {
    // fancy if else loop again
    return (Constants.SwerveConstants.invertGyro)
        ? Rotation2d.fromDegrees(360 - (gyro.getAngle()))
        : Rotation2d.fromDegrees(gyro.getAngle());
  }

  @Override
  public void periodic() {
    swerveOdometry.update(getYaw(), getPositions());
    field.setRobotPose(getPose());

    for (SwerveModule mod : mSwerveMods) {
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Integrated", mod.getState().angle.getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
    }
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {

    SwerveModuleState[] states = { mSwerveMods[0].getState(), mSwerveMods[1].getState(), mSwerveMods[2].getState(),
        mSwerveMods[3].getState() };

    return Constants.SwerveConstants.swerveKinematics.toChassisSpeeds(states);
  }

  private void voltageDrive(Measure<Voltage> voltage) {
    for (SwerveModule mod : mSwerveMods) {
      mod.driveMotor.setVoltage(voltage.magnitude());
    }
  }

  private void logMotors(SysIdRoutineLog log) {
    int i = 0;
    for (SwerveModule mod : mSwerveMods) {
      log.motor("motor" + i).voltage(Units.Volts.of(mod.driveMotor.getBusVoltage()))
          .angularPosition(Units.Rotations.of(mod.driveMotor.getEncoder().getPosition()))
          .angularVelocity(Units.RPM.of(mod.driveMotor.getEncoder().getVelocity()));
      i++;
    }
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return routine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return routine.dynamic(direction);
  }
}
