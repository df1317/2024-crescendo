package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Controllers;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class AutoAlignArm extends Command {

    private LimelightSubsystem m_LimelightSubsystem;
    private ArmSubsystem m_ArmSubsystem;
    private Controllers m_Controllers;

    private double slope = 2.5;
    private double startDist = 2;
    private double endDist = 3.6;

    public AutoAlignArm(LimelightSubsystem LimelightSub, ArmSubsystem ArmSub, Controllers m_Controllers) {
        m_LimelightSubsystem = LimelightSub;
        m_ArmSubsystem = ArmSub;
        this.m_Controllers = m_Controllers;
        addRequirements(LimelightSub, ArmSub);
    }

    /**
     * calculate shooter angle from limelight
     * taking into account arm offset
     */
    public double calculateShooterAngle() {
        // calculate opposite
        double opposite = Constants.Field.speakerZ - Constants.ArmShooterConstants.Arm.jointHeight
                + Constants.ArmShooterConstants.Arm.armLenght
                        * Math.sin(60 - Constants.ArmShooterConstants.Arm.optimizedAngle);
        // calculate adjacent
        double speakerDist = m_LimelightSubsystem.getSpeakerDistance();
        double adjacent = speakerDist
                + Constants.ArmShooterConstants.Arm.armLenght
                        * Math.cos(60 - Constants.ArmShooterConstants.Arm.optimizedAngle);
        // return arctan
        double adujustmentAngle = 0;
        if (speakerDist > 2) {
            adujustmentAngle = (speakerDist - startDist) * slope / (endDist - startDist);
        }
        double desAngle = Math.toDegrees(Math.atan2(opposite, adjacent));

        return desAngle + adujustmentAngle;
    }

    @Override
    public void initialize() {
        SmartDashboard.putBoolean("Aut Moving Arm", true);

        if (m_LimelightSubsystem.hasTargets) {
            m_ArmSubsystem.setAngle(calculateShooterAngle());
            m_ArmSubsystem.runPID();
        }
    }

    @Override
    public void execute() {
        if (m_LimelightSubsystem.hasTargets) {
            m_ArmSubsystem.setAngle(calculateShooterAngle());
            m_ArmSubsystem.runPID();
        }
    }

    @Override
    public boolean isFinished() {
        // Check if the button is released or if the specified duration has passed
        if (!m_Controllers.rightAutoAlignArmButtonState() && !m_Controllers.leftAutoAlignArmButtonState()) {
            return true;
        } else if (!m_LimelightSubsystem.hasTargets) {
            return true;
        }

        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_ArmSubsystem.setAngle(65);
        m_ArmSubsystem.spinDown();

        SmartDashboard.putBoolean("Aut Moving Arm", false);
    }
}
