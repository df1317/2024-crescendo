package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Controllers;
import frc.robot.subsystems.FiringSubsystem;
import frc.robot.Constants;
import frc.robot.subsystems.LimelightSubsystem;

public class AutoFireNote extends Command {
    private FiringSubsystem m_FiringSubsystem;
    private LimelightSubsystem m_LimelightSubsystem;
    private CommandXboxController xboxController;

    private Controllers m_Controllers;

    private boolean intake;
    private boolean shoot;
    private boolean autoAmp;
    private Trigger autoAmpTrigger;
    private boolean manualAmp;
    private boolean auto;

    private double timer;
    private double intakeSpeed = -0.6;
    private double intakeAutoAmpSpeed = 0.8;
    private double shooterManualAmp = -2000;

    private double dist[] = { 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0, 5.5 };
    private double speeds[] = { -3000, -3200, -3400, -3500, -3500, -3500, -3500,
            -3500, -3500 };

    private double waitTime = 2000;
    private double shootTime = 1000;

    private double calculateShooterSpeed(double robotDist) {
        double returnAngle = speeds[0];
        int rangeindex = 0;
        if (!m_LimelightSubsystem.hasTargets) {// return a default angle if limelight can't find targets
            return returnAngle;
        }

        for (int i = 0; i < dist.length - 1; i++) {// identify range your in
            if (robotDist > dist[i] && robotDist <= dist[i + 1]) {// dido
                rangeindex = i;
                break;
            }
        }
        // calculate slope
        double slope = (speeds[rangeindex + 1] - speeds[rangeindex]) / (dist[rangeindex + 1] - dist[rangeindex]);
        // finding the angle from distance
        returnAngle = slope * (robotDist - dist[rangeindex]) + speeds[rangeindex];

        return returnAngle;
    }

    public AutoFireNote(FiringSubsystem FiringSub, LimelightSubsystem LimeLightSub, Controllers controllers,
            Constants.AutoShooterStates state) {
        m_FiringSubsystem = FiringSub;
        m_LimelightSubsystem = LimeLightSub;
        this.m_Controllers = controllers;

        switch (state) {
            case INTAKE:
                intake = true;
                break;
            case SHOOT:
                shoot = true;
                break;
            case TELEOP:
                break;
        }

        addRequirements(FiringSub);
    }

    @Override
    public void initialize() {
        SmartDashboard.putString("Auto Firing Status", "Initializing");
        System.out.println("Initializing");

        auto = DriverStation.isAutonomous();
        System.out.println("AUTO:" + auto);

        if (!auto) {
            intake = m_Controllers.intakeButton.getAsBoolean();
            shoot = m_Controllers.shooterButton.getAsBoolean();
            autoAmp = m_Controllers.ampAutoAlignLeft.or(m_Controllers.ampAutoAlignRight).getAsBoolean();
            manualAmp = m_Controllers.manualArmAimButton.getAsBoolean();
            autoAmpTrigger = m_Controllers.ampAutoAlignLeft.or(m_Controllers.ampAutoAlignRight);
        }

        System.out.println("intake:" + intake);
        System.out.println("shoot:" + shoot);
        System.out.println("autoAmp:" + autoAmp);
        System.out.println("manualAmp:" + manualAmp);
        System.out.println("autoAmpTrigger:" + autoAmpTrigger);

        xboxController = m_Controllers.m_XboxController;

        if (manualAmp) {
            m_FiringSubsystem.spinUpShooter(shooterManualAmp);
        } else if (autoAmp) {
            m_FiringSubsystem.spinUpIntake(intakeAutoAmpSpeed);
        } else if (shoot) {
            double speakerDist = m_LimelightSubsystem.getSpeakerDistance();
            m_FiringSubsystem.spinUpShooter(calculateShooterSpeed(speakerDist));
        } else if (intake) {
            m_FiringSubsystem.spinUpIntake(intakeSpeed);
        }

        timer = System.currentTimeMillis();
    }

    @Override
    public void execute() {
        if (manualAmp || shoot) {
            if (System.currentTimeMillis() - timer > waitTime) {
                m_FiringSubsystem.spinUpIntake(intakeSpeed);
            }
        }
    }

    @Override
    public boolean isFinished() {
        if (manualAmp || (shoot && !autoAmp)) {
            if (System.currentTimeMillis() - timer > waitTime + shootTime) {
                return true;
            }
        } else if (intake && !autoAmp) {
            if (!m_FiringSubsystem.noteSensor.get()) {
                if (!auto) {
                    new Rumble(xboxController, 0.5, 1).schedule();
                }
                return true;
            }
        } else if (!auto && autoAmp) {
            if (!autoAmpTrigger.getAsBoolean()) {
                return true;
            }
        } else if (autoAmp && !autoAmpTrigger.getAsBoolean()) {
            return true;
        }

        return false;
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putString("Auto Firing Status", "Spinning Down");
        System.out.println("Spinning Down");

        // Spin down motors
        m_FiringSubsystem.spinDownIntake();
        m_FiringSubsystem.spinDownShooter();
    }
}
