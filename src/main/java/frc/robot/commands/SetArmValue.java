package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants;
import frc.robot.Controllers;
import frc.robot.subsystems.ArmSubsystem;

public class SetArmValue extends Command {
    private ArmSubsystem m_ArmSubsystem;
    private Controllers m_Controllers;

    private double joystickMax = 1;
    private double joystickMin = 0;
    private double joystickRange = joystickMax - joystickMin;

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

    public SetArmValue(ArmSubsystem ArmSub, Controllers m_Controllers) {
        m_ArmSubsystem = ArmSub;
        this.m_Controllers = m_Controllers;
        addRequirements(ArmSub);
    }

    @Override
    public void initialize() {
        SmartDashboard.putBoolean("Arm Command", true);
    }

    @Override
    public void execute() {
        double armAngle = getJoystickArm();
        m_ArmSubsystem.setAngle(armAngle);
        m_ArmSubsystem.runPID();
    }

    @Override
    public boolean isFinished() {
        if (!m_Controllers.manualArmAimButtonState()) {
            m_ArmSubsystem.spinDown();
            return true;
        }

        return false;
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("Arm Command", false);
        m_ArmSubsystem.spinDown();
    }
}
