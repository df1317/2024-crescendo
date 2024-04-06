package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
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
    public RelativeEncoder intakeEncoder;

    private TalonSRX intakeTop;
    private TalonSRX intakeBottom;

    public DigitalInput noteSensor;

    public SparkPIDController shooterTopPID;
    public SparkPIDController shooterBottomPID;
    public SparkPIDController intakePID;

    public double intakeKp = 0.0002 * 5.75;
    public double intakeKi = 0;
    public double intakeKd = 0;
    public double intakeFF = 0.0004;

    public double shooterKp = 0.00001 * 5;
    public double shooterKi = 0;
    public double shooterKd = 0;
    public double shooterFF = 0.0002;

    public FiringSubsystem() {
        shooterTop = new CANSparkMax(3, MotorType.kBrushless);
        shooterBottom = new CANSparkMax(4, MotorType.kBrushless);

        intakeTop = new TalonSRX(5);
        intakeBottom = new TalonSRX(8);

        noteSensor = new DigitalInput(
                Constants.ArmShooterConstants.ShooterCollectorConstants.NoteSensorPort);
        shooterTopPID = shooterTop.getPIDController();
        shooterBottomPID = shooterBottom.getPIDController();

        shooterBottomEndcoder = shooterBottom.getEncoder();
        shooterTopEndcoder = shooterTop.getEncoder();

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
        intakeTop.set(TalonSRXControlMode.PercentOutput, speed);
        intakeBottom.set(TalonSRXControlMode.PercentOutput, speed);
    }

    public void spinDownIntake() {
        SmartDashboard.putString("Firing Status", "Spin down Intake");
        intakeTop.set(TalonSRXControlMode.PercentOutput, 0);
        intakeBottom.set(TalonSRXControlMode.PercentOutput, 0);
    }

    public void logVals() {
        SmartDashboard.putNumber("shooter0 value", shooterTop.get());
        SmartDashboard.putNumber("shooter1 value", shooterBottom.get());
        SmartDashboard.putBoolean("Note Sensor", noteSensor.get());
    }
}
