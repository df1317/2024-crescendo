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
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;



/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  

  // Replace with CommandPS4Controller or CommandJoystick if needed
  public final CommandXboxController m_XboxController = new CommandXboxController(0);
  //private final CommandJoystick m_JoystickL = new CommandJoystick(0);
  //private final CommandJoystick m_JoystickR = new CommandJoystick(1);

  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;
  
  private final Trigger robotCentric =
    new Trigger(m_XboxController.leftBumper());
  public final Trigger xButton =
    new Trigger(m_XboxController.x());
  public final Trigger yButton =
    new Trigger(m_XboxController.y());

  /*private final int translationAxis = Joystick.AxisType.kY.value; //left flight stick
  private final int strafeAxis = Joystick.AxisType.kX.value; //left flight stick
  private final int rotationAxis = Joystick.AxisType.kX.value; //right flight stick*/

  /* Subsystems */
  private final SwerveSubsystem m_SwerveSubsystem = new SwerveSubsystem();
  private final FiringSubsystem m_FiringSubsystem = new FiringSubsystem();
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
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
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
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
  
  public Command resetSwerveWheels() {
    return new InstantCommand(() -> m_SwerveSubsystem.setWheelsToX());
  }
}
