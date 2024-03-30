package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Controllers;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoAlign extends Command {
    private LimelightSubsystem m_LimelightSubsystem;
    private SwerveSubsystem m_SwerveSubsystem;

    private Controllers m_Controllers;

    private Trigger endTrigger;

    private SlewRateLimiter translationLimiter = new SlewRateLimiter(3.0); // can only change by 3 m/s in the span of 1
                                                                           // s
    private SlewRateLimiter strafeLimiter = new SlewRateLimiter(3.0);
    private SlewRateLimiter rotationLimiter = new SlewRateLimiter(3.0);

    private PIDController pidController = new PIDController(0.2, 0, 0);

    public AutoAlign(LimelightSubsystem limelightSub, SwerveSubsystem swerveSub, Controllers controllers,
            Trigger endTrigger) {
        this.m_LimelightSubsystem = limelightSub;
        this.m_SwerveSubsystem = swerveSub;
        this.m_Controllers = controllers;
        this.endTrigger = endTrigger;

        addRequirements(swerveSub);
    }

    public double getAlignedRotation() {
        double speakerX = (DriverStation.getAlliance().orElseThrow() == DriverStation.Alliance.Blue)
                ? Constants.Field.speakerXBlue
                : Constants.Field.speakerXRed;
        // getting X and Y distance from the robot
        double distX = speakerX - m_LimelightSubsystem.botpose3d.getX();
        double distY = Constants.Field.speakerY - m_LimelightSubsystem.botpose3d.getY();

        return Math.atan2(distX, distY);
    }

    @Override
    public void execute() {
        double power = 0;

        if (m_LimelightSubsystem.hasTargets) {
            power = pidController.calculate(getAlignedRotation(),
                    m_LimelightSubsystem.botpose3d.getRotation().getAngle());
        }

        // clamp power between -1 and 1
        power = MathUtil.clamp(power, -0.5, 0.5);

        double translationVal = translationLimiter.calculate(
                MathUtil.applyDeadband(m_Controllers.translationAxis(), Constants.SwerveConstants.inputDeadband));
        double strafeVal = strafeLimiter.calculate(
                MathUtil.applyDeadband(m_Controllers.strafeAxis(), Constants.SwerveConstants.inputDeadband));
        double rotationVal = rotationLimiter.calculate(
                MathUtil.applyDeadband(-power, Constants.SwerveConstants.inputDeadband));

        m_SwerveSubsystem.drive(
                // the joystick values (-1 to 1) multiplied by the max speed of the drivetrain
                new Translation2d(translationVal, strafeVal).times(Constants.SwerveConstants.maxSpeed),
                // rotation value times max spin speed
                rotationVal * Constants.SwerveConstants.maxAngularVelocity,
                // whether or not in field centric mode
                m_Controllers.robotCentricButtonState(),
                // open loop control
                false);
    }

    @Override
    public boolean isFinished() {
        // Check if the button is released or if the specified duration has passed
        return !endTrigger.getAsBoolean();
    }

    @Override
    public void end(boolean interrupted) {
        m_SwerveSubsystem.drive(new Translation2d(), 0, true, false);
    }
}
