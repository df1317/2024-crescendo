package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.Constants;

public class SetArmValue extends Command {
    private ArmSubsystem m_ArmSubsystem;
    private CommandJoystick m_Joystick;

    private double joystickMax = -1;
    private double joystickMin = 1;
    private double joystickRange = joystickMax - joystickMin;

    private double getAxis() {
        return m_Joystick.getRawAxis(Joystick.AxisType.kY.value);
    }

    private double getJoystickArm() {
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
        double armAngle = ArmSubsystem.armRange / 1 * percent + Constants.ArmShooterConstants.Arm.EncoderMin;
        return armAngle;
    }

    private boolean getButton() {
        return m_Joystick.button(1).getAsBoolean();
    }

    public SetArmValue(ArmSubsystem ArmSub, CommandJoystick joystick) {
        m_ArmSubsystem = ArmSub;
        m_Joystick = joystick;
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
        SmartDashboard.putNumber("Current Arm Angle", m_ArmSubsystem.getArmAngle());
        SmartDashboard.putNumber("Desired Arm Angle", armAngle);
    }

    @Override
    public boolean isFinished() {
        if (!getButton()) {
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
