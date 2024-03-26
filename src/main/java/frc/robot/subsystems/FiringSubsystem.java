package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FiringSubsystem extends SubsystemBase {
    private CANSparkMax shooterTop;
    private CANSparkMax shooterBottom;// the bottom shooter
    public RelativeEncoder shooterTopEndcoder;
    public RelativeEncoder shooterBottomEndcoder;

    private CANSparkMax intake;

    public DigitalInput noteSensor;

    public SparkPIDController shooterTopPID;
    public SparkPIDController shooterBottomPID;
    public SparkPIDController intakePID;
    private boolean editablePIDConstants = true;

    public double intakeKp = 1;
    public double intakeKi = 0;
    public double intakeKd = 0;
    public double intakeFF = 0;

    public double shooterKp = 0;
    public double shooterKi = 0;
    public double shooterKd = 0;
    public double shooterFF = 0.0002;

    public FiringSubsystem() {
        shooterTop = new CANSparkMax(3, MotorType.kBrushless);
        shooterBottom = new CANSparkMax(4, MotorType.kBrushless);

        intake = new CANSparkMax(5, MotorType.kBrushless);

        noteSensor = new DigitalInput(
                Constants.ArmShooterConstants.ShooterCollectorConstants.NoteSensorPort);
        shooterTopPID = shooterTop.getPIDController();
        shooterBottomPID = shooterBottom.getPIDController();
        // intakePID = intake.getPIDController();

        shooterBottomEndcoder = shooterBottom.getEncoder();
        shooterTopEndcoder = shooterTop.getEncoder();

        // intakePID.setReference(0, CANSparkBase.ControlType.kVelocity);

        // intakePID.setP(intakeKp);
        // intakePID.setI(intakeKi);
        // intakePID.setD(intakeKd);
        // intakePID.setFF(intakeFF);

        shooterTopPID.setP(shooterKp);
        shooterTopPID.setI(shooterKi);
        shooterTopPID.setD(shooterKd);
        shooterTopPID.setFF(shooterFF);

        shooterBottomPID.setP(shooterKp);
        shooterBottomPID.setI(shooterKi);
        shooterBottomPID.setD(shooterKd);
        shooterBottomPID.setFF(shooterFF);

        SmartDashboard.putNumber("shooterKp", shooterKp);
        SmartDashboard.putNumber("shooterKi", shooterKi);
        SmartDashboard.putNumber("shooterKd", shooterKd);
        SmartDashboard.putNumber("shooterFF", shooterFF);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter Bottom RPM:", shooterBottomEndcoder.getVelocity());
        SmartDashboard.putNumber("Shooter Top RPM:", shooterTopEndcoder.getVelocity());

        SmartDashboard.putNumber("shooterKp", shooterKp);
        SmartDashboard.putNumber("shooterKi", shooterKi);
        SmartDashboard.putNumber("shooterKd", shooterKd);
        SmartDashboard.putNumber("shooterFF", shooterFF);

        if (editablePIDConstants) {
            shooterKp = SmartDashboard.getNumber("shooterKp", shooterKp);
            shooterKi = SmartDashboard.getNumber("shooterKi", shooterKi);
            shooterKd = SmartDashboard.getNumber("shooterKd", shooterKd);
            shooterFF = SmartDashboard.getNumber("shooterFF", shooterFF);

            shooterTopPID.setP(shooterKp);
            shooterTopPID.setI(shooterKi);
            shooterTopPID.setD(shooterKd);
            shooterTopPID.setFF(shooterFF);

            shooterBottomPID.setP(shooterKp);
            shooterBottomPID.setI(shooterKi);
            shooterBottomPID.setD(shooterKd);
            shooterBottomPID.setFF(shooterFF);
        }

    }

    public void spinUpShooter(double speed) {
        SmartDashboard.putString("Firing Status", "Firing Shooter");
        shooterTopPID.setReference(speed, ControlType.kVelocity);
        shooterBottomPID.setReference(speed, ControlType.kVelocity);
    }

    public void spinDownShooter() {
        SmartDashboard.putString("Firing Status", "Spinning Down Shooter");
        shooterTopPID.setReference(0, ControlType.kVelocity);
        shooterBottomPID.setReference(0, ControlType.kVelocity);
    }

    public void spinUpIntake(double speed) {
        SmartDashboard.putString("Firing Status", "Spin up Intake");
        intake.set(speed);
    }

    public void spinDownIntake() {
        SmartDashboard.putString("Firing Status", "Spin down Intake");
        intake.stopMotor();
    }

    public void logVals() {
        SmartDashboard.putNumber("shooter0 value", shooterTop.get());
        SmartDashboard.putNumber("shooter1 value", shooterBottom.get());
        SmartDashboard.putNumber("intake value", intake.get());
        SmartDashboard.putBoolean("Note Sensor", noteSensor.get());
    }
}
