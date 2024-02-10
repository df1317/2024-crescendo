package frc.robot.autos;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.List;

import com.pathplanner.lib.path.PathConstraints;
//import com.pathplanner.lib.PathPlanner;
//import com.pathplanner.lib.PathPlannerTrajectory; 

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.DriverStation;

public class exampleAuto extends SequentialCommandGroup {
        public exampleAuto(SwerveSubsystem m_SwerveSubsystem) {
                TrajectoryConfig config = new TrajectoryConfig(
                                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                                .setKinematics(Constants.SwerveConstants.swerveKinematics);

                // An example trajectory to follow. All units in meters.
                Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                                // Start at the origin facing the +X direction
                                new Pose2d(0, 0, new Rotation2d(0)),
                                // Pass through these interior waypoints
                                List.of(new Translation2d(0, -3), new Translation2d(-3, -3)),
                                // End 1.5 meters straight ahead of where we started, facing forward
                                new Pose2d(3, 0, new Rotation2d(0)),
                                config);

                var thetaController = new ProfiledPIDController(
                                Constants.AutoConstants.kPThetaController,
                                0,
                                0,
                                Constants.AutoConstants.kThetaControllerConstraints);
                thetaController.enableContinuousInput(-Math.PI, Math.PI);

                SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                                exampleTrajectory,
                                m_SwerveSubsystem::getPose,
                                Constants.SwerveConstants.swerveKinematics,
                                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                                thetaController,
                                m_SwerveSubsystem::setModuleStates,
                                m_SwerveSubsystem);

                addCommands(
                                new InstantCommand(() -> m_SwerveSubsystem
                                                .resetOdometry(exampleTrajectory.getInitialPose())),
                                swerveControllerCommand);
        }

        /*
         * AutoBuilder.configureHolonomic(
         * this::getPose, // Robot pose supplier
         * this::resetPose, // Method to reset odometry (will be called if your auto has
         * a starting pose)
         * this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT
         * RELATIVE
         * this::driveRobotRelative, // Method that will drive the robot given ROBOT
         * RELATIVE ChassisSpeeds
         * new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should
         * likely live in your Constants class
         * new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
         * new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
         * 4.5, // Max module speed, in m/s
         * 0.4, // Drive base radius in meters. Distance from robot center to furthest
         * module.
         * new ReplanningConfig() // Default path replanning config. See the API for the
         * options here
         * ),
         * () -> {
         * // Boolean supplier that controls when the path will be mirrored for the red
         * alliance
         * // This will flip the path being followed to the red side of the field.
         * // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
         * 
         * var alliance = DriverStation.getAlliance();
         * if (alliance.isPresent()) {
         * return alliance.get() == DriverStation.Alliance.Red;
         * }
         * return false;
         * },
         * this // Reference to this subsystem to set requirements
         * );
         */
}