// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.OnboardModuleState;
import frc.lib.SwerveModuleConstants;
import frc.robot.Constants;

/** Add your docs here. */
public class SwerveModule {
    public int moduleNumber;
    public double m_angleKP;
    public double m_angleKI;
    public double m_angleKD;
    public double m_angleKFF;
    private Rotation2d lastAngle;
    private Rotation2d angleOffset;

    private CANSparkMax angleMotor;
    private CANSparkMax driveMotor;

    private RelativeEncoder driveEncoder;
    private RelativeEncoder integratedAngleEncoder;

    public CANcoder angleEncoder;

    private final SparkPIDController driveController;
    private final SparkPIDController angleController;

    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
            Constants.SwerveConstants.driveKS, Constants.SwerveConstants.driveKV, Constants.SwerveConstants.driveKA);
    // creates a feedforward for the swerve drive. feedforward does 90% of the work,
    // estimating stuff
    // PID fixes the error

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
        this.moduleNumber = moduleNumber;
        this.m_angleKP = moduleConstants.angleKP;
        this.m_angleKI = moduleConstants.angleKI;
        this.m_angleKD = moduleConstants.angleKD;
        this.m_angleKFF = moduleConstants.angleKFF;
        angleOffset = moduleConstants.angleOffset;
        // this.?
        /* Angle Encoder Config */
        angleEncoder = new CANcoder(moduleConstants.cancoderID);
        // configAngleEncoder();

        /* Angle Motor Config */
        angleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
        integratedAngleEncoder = angleMotor.getEncoder();
        angleController = angleMotor.getPIDController();

        /* Drive Motor Config */
        driveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
        driveEncoder = driveMotor.getEncoder();
        driveController = driveMotor.getPIDController();

        lastAngle = getState().angle;
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(driveEncoder.getVelocity(), getAngle());
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(driveEncoder.getPosition(), getAngle());
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        // Custom optimize command, since default WPILib optimize assumes continuous
        // controller which
        // REV supports this now so dont have to worry with rev, but need some funky
        // configs i dont want to do
        // have to be sad with falcons but thats what you get for giving money to Tony
        desiredState = OnboardModuleState.optimize(desiredState, getState().angle);

        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {
            // when not taking feedback
            double percentOutput = desiredState.speedMetersPerSecond / Constants.SwerveConstants.maxSpeed;
            driveMotor.set(percentOutput);
        } else {
            driveController.setReference(
                    desiredState.speedMetersPerSecond,
                    ControlType.kVelocity,
                    0,
                    feedforward.calculate(desiredState.speedMetersPerSecond));
        }
    }

    private void setAngle(SwerveModuleState desiredState) {
        // Prevent rotating module if speed is less then 1%. Prevents Jittering.
        // the ? and : are a shorthand for an if-else loop
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.SwerveConstants.maxSpeed * 0.01))
                ? lastAngle
                : desiredState.angle;

        angleController.setReference(angle.getDegrees(), ControlType.kPosition);
        lastAngle = angle;
    }

    public void resetToAbsolute() {
        double absolutePosition = getCanCoder().getRotations() - angleOffset.getRotations();
        integratedAngleEncoder.setPosition(absolutePosition); // may need to change

    }

    public Rotation2d getCanCoder() {
        return Rotation2d.fromRotations(-angleEncoder.getAbsolutePosition().getValueAsDouble());
    }

    private Rotation2d getAngle() {
        return Rotation2d.fromDegrees(angleEncoder.getPosition().getValueAsDouble()); // Must be degrees even though
        // documentation says it should be
        // rotations
    }

}
