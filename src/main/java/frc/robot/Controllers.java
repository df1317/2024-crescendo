package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

public class Controllers {
    public final CommandXboxController m_XboxController = new CommandXboxController(0);
    public final CommandJoystick m_JoystickL = new CommandJoystick(1);
    public final CommandJoystick m_JoystickR = new CommandJoystick(2);

    public final Trigger shooterButton = new Trigger(m_JoystickL.button(1));
    public final Trigger intakeButton = new Trigger(m_JoystickR.button(1));
    public final Trigger robotCentricButton = new Trigger(m_XboxController.leftBumper());
    public final Trigger rightClimberButton = new Trigger(m_JoystickR.button(3));
    public final Trigger leftClimberButton = new Trigger(m_JoystickL.button(3));
    public final Trigger rightAutoAlignArmButton = new Trigger(m_JoystickR.button(4));
    public final Trigger leftAutoAlignArmButton = new Trigger(m_JoystickL.button(4));
    public final Trigger manualArmAimButton = new Trigger(m_JoystickR.button(2));

    public boolean shooterButtonState() {
        boolean shooterButtonState = shooterButton.getAsBoolean();
        return shooterButtonState;
    }

    public boolean intakeButtonState() {
        boolean intakeButtonState = intakeButton.getAsBoolean();
        return intakeButtonState;
    }

    public boolean robotCentricButtonState() {
        boolean robotCentricButtonState = robotCentricButton.getAsBoolean();
        return robotCentricButtonState;
    }

    public boolean rightClimberButtonState() {
        boolean rightClimberButtonState = rightClimberButton.getAsBoolean();
        return rightClimberButtonState;
    }

    public boolean leftClimberButtonState() {
        boolean leftClimberButtonState = leftClimberButton.getAsBoolean();
        return leftClimberButtonState;
    }

    public boolean rightAutoAlignArmButtonState() {
        boolean rightAutoAimArmButtonState = rightAutoAlignArmButton.getAsBoolean();
        return rightAutoAimArmButtonState;
    }

    public boolean leftAutoAlignArmButtonState() {
        boolean leftAutoAimButtonState = leftAutoAlignArmButton.getAsBoolean();
        return leftAutoAimButtonState;
    }

    public boolean manualArmAimButtonState() {
        boolean manualArmAimButtonState = manualArmAimButton.getAsBoolean();
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
