package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

public class Controllers {
    public final CommandXboxController m_XboxController = new CommandXboxController(0);
    public final CommandJoystick m_JoystickL = new CommandJoystick(1);
    public final CommandJoystick m_JoystickR = new CommandJoystick(2);

    public boolean shooterButtonState() {
        boolean shooterButtonState = m_JoystickL.button(1).getAsBoolean();
        return shooterButtonState;
    }

    public boolean intakeButtonState() {
        boolean intakeButtonState = m_JoystickR.button(1).getAsBoolean();
        return intakeButtonState;
    }

    public boolean robotCentricButtonState() {
        boolean robotCentricButtonState = m_XboxController.leftBumper().getAsBoolean();
        return robotCentricButtonState;
    }

    public boolean rightClimberButtonState() {
        boolean rightClimberButtonState = m_JoystickR.button(3).getAsBoolean();
        return rightClimberButtonState;
    }

    public boolean leftClimberButtonState() {
        boolean leftClimberButtonState = m_JoystickL.button(3).getAsBoolean();
        return leftClimberButtonState;
    }

    public boolean rightAutoAimArmButtonState() {
        boolean rightAutoAimArmButtonState = m_JoystickR.button(4).getAsBoolean();
        return rightAutoAimArmButtonState;
    }

    public boolean leftAutoAimButtonState() {
        boolean leftAutoAimButtonState = m_JoystickL.button(4).getAsBoolean();
        return leftAutoAimButtonState;
    }

    public boolean manualArmAimButtonState() {
        boolean manualArmAimButtonState = m_JoystickR.button(2).getAsBoolean();
        return manualArmAimButtonState;
    }

    public double rightJoystickPosistion() {
        double rightJoystickPosistion = m_JoystickR.getRawAxis(Joystick.AxisType.kY.value);
        return rightJoystickPosistion;
    }

    public double leftJoystickPosistion() {
        double leftJoystickPosistion = m_JoystickL.getRawAxis(Joystick.AxisType.kY.value);
        return leftJoystickPosistion;
    }

    public double translationAxis() {
        double translationAxis = m_XboxController.getRawAxis(XboxController.Axis.kLeftY.value);
        return translationAxis;
    }

    public double strafeAxis() {
        double strafeAxis = m_XboxController.getRawAxis(XboxController.Axis.kLeftX.value);
        return strafeAxis;
    }

    public double rotationAxis() {
        double rotationAxis = m_XboxController.getRawAxis(XboxController.Axis.kRightX.value);
        return rotationAxis;
    }

}
