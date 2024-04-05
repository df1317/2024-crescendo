package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Controllers;
import frc.robot.subsystems.ArmSubsystem;

public class Shuttle extends Command {

    private ArmSubsystem m_ArmSubsystem;
    private Controllers m_Controllers;

    private double joystickMax = 1;
    private double joystickMin = 0;
    private double joystickRange = joystickMax - joystickMin;

    public Shuttle(ArmSubsystem ArmSub, Controllers Controllers) {
        m_ArmSubsystem = ArmSub;
        m_Controllers = Controllers;
        addRequirements(ArmSub);
    }

    private double getAxis() {
        SmartDashboard.putNumber("arm joystick", m_Controllers.rightJoystickPosistion());
        return -m_Controllers.rightJoystickPosistion();
    }

    private double getJoystickArm() {
        // flip the axis range
        double joystickPos = getAxis();
        // clamp joystick input value
        if (joystickPos > joystickMax) {
            joystickPos = joystickMax;
        } else if (joystickPos < joystickMin) {
            joystickPos = joystickMin;
        }
        // get percent of joystick range
        double joySlope = 1 / joystickRange;
        double percent = joySlope * joystickPos + joySlope * joystickMin;
        // get same percent of arm range
        double armAngle = (ArmSubsystem.armRange * percent) + Constants.ArmShooterConstants.Arm.EncoderMin;
        return armAngle;
    }

    @Override
    public void initialize() {
        SmartDashboard.putBoolean("Aut Moving Arm", true);
    }

    @Override
    public void execute() {
        double armAngle = getJoystickArm();
        m_ArmSubsystem.setAngle(armAngle);
        m_ArmSubsystem.runPID();
    }

    @Override
    public boolean isFinished() {
        return !m_Controllers.shuttle();
    }

    @Override
    public void end(boolean interrupted) {
        m_ArmSubsystem.setAngle(65);
        m_ArmSubsystem.spinDown();

        SmartDashboard.putBoolean("Aut Moving Arm", false);
    }
}
