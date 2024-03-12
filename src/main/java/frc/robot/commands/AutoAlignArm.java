package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class AutoAlignArm extends Command {

    private LimelightSubsystem m_LimelightSubsystem;
    private ArmSubsystem m_ArmSubsystem;
    private Trigger button;

    public AutoAlignArm(LimelightSubsystem LimelightSub, ArmSubsystem ArmSub, Trigger button) {
        m_LimelightSubsystem = LimelightSub;
        m_ArmSubsystem = ArmSub;
        this.button = button;
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
        double adjacent = m_LimelightSubsystem.getSpeakerDistance()
                + Constants.ArmShooterConstants.Arm.armLenght
                        * Math.cos(60 - Constants.ArmShooterConstants.Arm.optimizedAngle);
        // return arctan

        return Math.toDegrees(Math.atan2(opposite, adjacent));
    }

    @Override
    public void initialize() {
        SmartDashboard.putBoolean("Aut Moving Arm", true);

        m_ArmSubsystem.setAngle(calculateShooterAngle());
        m_ArmSubsystem.runPID();
    }

    @Override
    public void execute() {

        m_ArmSubsystem.setAngle(calculateShooterAngle());
        m_ArmSubsystem.runPID();

        SmartDashboard.putNumber("Current Arm Angle", m_ArmSubsystem.getAngle());
        SmartDashboard.putNumber("Desired Arm Angle", calculateShooterAngle());
    }

    @Override
    public boolean isFinished() {
        // Check if the button is released or if the specified duration has passed
        return (!button.getAsBoolean());
    }

    @Override
    public void end(boolean interrupted) {
        m_ArmSubsystem.setAngle(65);
        m_ArmSubsystem.spinDown();

        SmartDashboard.putBoolean("Aut Moving Arm", false);
    }
}
