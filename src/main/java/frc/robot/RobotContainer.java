// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.commands.AutoAlign;
import frc.robot.commands.Climb;
import frc.robot.commands.FireNote;
import frc.robot.commands.SetArmValue;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimbingSubsystem;
import frc.robot.subsystems.FiringSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  // Replace with CommandPS4C,ontroller or CommandJoystick if needed
  public final CommandXboxController m_XboxController = new CommandXboxController(0);
  private final CommandJoystick m_JoystickL = new CommandJoystick(1);
  private final CommandJoystick m_JoystickR = new CommandJoystick(2);
  // private final CommandJoystick m_JoystickR = new CommandJoystick(1);

  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  private final Trigger robotCentric = new Trigger(m_XboxController.leftBumper());
  public final Trigger xButton = new Trigger(m_XboxController.x());
  public final Trigger yButton = new Trigger(m_XboxController.y());

  /*
   * private final int translationAxis = Joystick.AxisType.kY.value; //left flight
   * stick
   * private final int strafeAxis = Joystick.AxisType.kX.value; //left flight
   * stick
   * private final int rotationAxis = Joystick.AxisType.kX.value; //right flight
   * stick
   */

  /* Subsystems */
  private final SwerveSubsystem m_SwerveSubsystem = new SwerveSubsystem();
  private final FiringSubsystem m_FiringSubsystem = new FiringSubsystem();
  private final LimelightSubsystem m_LimelightSubsystem = new LimelightSubsystem();
  private final ArmSubsystem m_ArmSubsystem = new ArmSubsystem();
  private final ClimbingSubsystem m_ClimbingSubsystem = new ClimbingSubsystem();

  private final SendableChooser<Command> autoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_SwerveSubsystem.setDefaultCommand(
        new TeleopSwerve(
            m_SwerveSubsystem,
            () -> m_XboxController.getRawAxis(translationAxis),
            () -> m_XboxController.getRawAxis(strafeAxis),
            () -> -m_XboxController.getRawAxis(rotationAxis),
            () -> robotCentric.getAsBoolean()));

    // Configure the trigger bindings
    configureBindings();

    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    AutoAlign autoAlign = new AutoAlign(m_LimelightSubsystem, m_SwerveSubsystem, m_ArmSubsystem, m_XboxController);
    m_XboxController.button(Button.kA.value).onTrue(autoAlign);

    // climb command should be able to work only if a button is pressed on the
    // joystick and the trigger is pressed
    Climb climbCommand = new Climb(m_ClimbingSubsystem, m_JoystickL, m_JoystickR);
    SetArmValue setArmValueCommand = new SetArmValue(m_ArmSubsystem, m_JoystickL);
    // setup two firing speeds
    FireNote fireNoteCommandIntake = new FireNote(m_FiringSubsystem, m_JoystickL.button(5), m_JoystickL, true, false);
    FireNote fireNoteCommandFlywheel = new FireNote(m_FiringSubsystem, m_JoystickR.button(5), m_JoystickR, false, true);
    FireNote fireNoteCommandAll = new FireNote(m_FiringSubsystem, m_JoystickL.button(4), m_JoystickL, true, true);
    m_JoystickL.button(5).onTrue(fireNoteCommandIntake);
    m_JoystickR.button(5).onTrue(fireNoteCommandFlywheel);
    m_JoystickL.button(4).onTrue(fireNoteCommandAll);

    (m_JoystickL.trigger().and(m_JoystickL.button(3))).or(m_JoystickR.trigger().and(m_JoystickR.button(3)))
        .onTrue(climbCommand);
    m_JoystickL.button(2).and(m_JoystickL.trigger()).onTrue(setArmValueCommand);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }

  public void updateLimelight() {
    m_LimelightSubsystem.updateLimelight();
  }
}
