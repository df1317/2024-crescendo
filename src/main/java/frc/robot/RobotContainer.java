// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autos.exampleAuto;
import frc.robot.commands.FireNote;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.FiringSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

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

  public enum Autos {
    A_LEAVE,
    B_LEAVE,
    C_LEAVE,
    A_AMP,
    A_2AMP,
    A_SPEAKER,
    A_2SPEAKER,
    B_SPEKAER,
    B_2SPEAKER,
    B_3SPEAKER,
    C_SPEAKER,
    C_2SPEAKER,
    C_STEALCENTERNOTES
  }

  PathPlannerPath A_Leave = PathPlannerPath.fromPathFile("A_Leave.auto");
  PathPlannerPath A_Amp = PathPlannerPath.fromPathFile("A_Amp.auto");
  PathPlannerPath A_2Amp = PathPlannerPath.fromPathFile("A_2Amp.auto");
  PathPlannerPath A_Speaker = PathPlannerPath.fromPathFile("A_Speaker.auto");
  PathPlannerPath A_2Speaker = PathPlannerPath.fromPathFile("A_2Speaker.auto");
  PathPlannerPath B_Leave = PathPlannerPath.fromPathFile("B_Leave.auto");
  PathPlannerPath B_Speaker = PathPlannerPath.fromPathFile("B_Speaker.auto");
  PathPlannerPath B_2Speaker = PathPlannerPath.fromPathFile("B_2Speaker.auto");
  PathPlannerPath B_3Speaker = PathPlannerPath.fromPathFile("B_3Speaker.auto");
  PathPlannerPath C_Leave = PathPlannerPath.fromPathFile("C_Leave.auto");
  PathPlannerPath C_Speaker = PathPlannerPath.fromPathFile("C_Speaker.auto");
  PathPlannerPath C_2Speaker = PathPlannerPath.fromPathFile("C_2Speaker.auto");
  PathPlannerPath C_StealCenterNotes = PathPlannerPath.fromPathFile("C_StealCenterNotes.auto");

  // Possible other way to call our autos??
  // PathPlanner A_Leave =
  // PathPlannerAuto.getPathGroupFromAutoFile("A_Leave.auto");

  // Replace with CommandPS4C,ontroller or CommandJoystick if needed
  public final CommandXboxController m_XboxController = new CommandXboxController(0);
  // private final CommandJoystick m_JoystickL = new CommandJoystick(0);
  // private final CommandJoystick m_JoystickR = new CommandJoystick(1);

  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  private final Trigger robotCentric = new Trigger(m_XboxController.leftBumper());
  public final Trigger xButton = new Trigger(m_XboxController.x());
  public final Trigger yButton = new Trigger(m_XboxController.y());

  public final SendableChooser<Autos> autoChooser = new SendableChooser<>();

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

  private Autos selectedAuto;// making a selected auto for later use

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_SwerveSubsystem.setDefaultCommand(
        new TeleopSwerve(
            m_SwerveSubsystem,
            () -> m_XboxController.getRawAxis(translationAxis),
            () -> m_XboxController.getRawAxis(strafeAxis),
            () -> m_XboxController.getRawAxis(rotationAxis),
            () -> robotCentric.getAsBoolean()));

    // Configure the trigger bindings
    configureBindings();

    NamedCommands.registerCommand("FireNote", new FireNote(m_FiringSubsystem, false, xButton));

    // Add options to auto chooser
    autoChooser.setDefaultOption("A_Leave", Autos.A_LEAVE);
    for (Autos autoChoice : Autos.values()) {// going through all options and putting them in a drop down in the driver
                                             // station
      autoChooser.addOption(autoChoice.name(), autoChoice);
    }
  }

  public Autos getSelectedAuto() {
    selectedAuto = autoChooser.getSelected();
    return selectedAuto;
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
    m_XboxController.button(Button.kA.value).onTrue(new InstantCommand(() -> m_SwerveSubsystem.zeroGyro()));
    m_XboxController.button(Button.kB.value).onTrue(new InstantCommand(() -> m_SwerveSubsystem.setWheelsToX()));
    // setup two firing speeds
    FireNote fireNoteCommandFar = new FireNote(m_FiringSubsystem, false, xButton);
    FireNote fireNoteCommandNear = new FireNote(m_FiringSubsystem, true, yButton);
    m_XboxController.button(Button.kX.value).onTrue(fireNoteCommandFar);
    m_XboxController.button(Button.kY.value).onTrue(fireNoteCommandNear);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new exampleAuto(m_SwerveSubsystem);
  }
}
