// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

public class TeleopSwerve extends CommandBase {
  private SwerveSubsystem m_SwerveSubsystem;
  private DoubleSupplier m_translationSupplier;
  private DoubleSupplier m_strafeSupplier;
  private DoubleSupplier m_rotationSupplier;
  private BooleanSupplier m_robotCentricSupplier;

  private SlewRateLimiter translationLimiter = new SlewRateLimiter(3.0); //can only change by 3 m/s in the span of 1 s
  private SlewRateLimiter strafeLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter rotationLimiter = new SlewRateLimiter(3.0);
  /** Creates a new TeleopSwerve. */
  public TeleopSwerve(SwerveSubsystem SwerveSubsystem,
      DoubleSupplier translationSupplier,
      DoubleSupplier strafeSupplier,
      DoubleSupplier rotationSupplier,
      BooleanSupplier robotCentricSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_SwerveSubsystem = SwerveSubsystem;
    addRequirements(m_SwerveSubsystem);
    this.m_translationSupplier = translationSupplier;
    this.m_strafeSupplier = strafeSupplier;
    this.m_rotationSupplier = rotationSupplier;
    this.m_robotCentricSupplier = robotCentricSupplier;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
        /* Get Values, applies Deadband, (doesnt do anything if stick is less than a value)*/
    double translationVal =
        translationLimiter.calculate(
            MathUtil.applyDeadband(m_translationSupplier.getAsDouble(), Constants.SwerveConstants.inputDeadband));
    double strafeVal =
        strafeLimiter.calculate(
            MathUtil.applyDeadband(m_strafeSupplier.getAsDouble(), Constants.SwerveConstants.inputDeadband));
    double rotationVal =
        rotationLimiter.calculate(
            MathUtil.applyDeadband(m_rotationSupplier.getAsDouble(), Constants.SwerveConstants.inputDeadband));

    /* Drive */
    SmartDashboard.putNumber("maxspeed", Constants.SwerveConstants.maxSpeed);
    m_SwerveSubsystem.drive(
        //the joystick values (-1 to 1) multiplied by the max speed of the drivetrain
        new Translation2d(translationVal, strafeVal).times(Constants.SwerveConstants.maxSpeed),
        //rotation value times max spin speed
        rotationVal * Constants.SwerveConstants.maxAngularVelocity,
        //whether or not in field centric mode
        !m_robotCentricSupplier.getAsBoolean(),
        //open loop control
        true);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
