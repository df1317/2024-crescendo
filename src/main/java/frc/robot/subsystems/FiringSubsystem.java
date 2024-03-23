package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FiringSubsystem extends SubsystemBase {
    private CANSparkMax shooter0;
    private CANSparkMax shooter1;

    private CANSparkMax intake;

    public DigitalInput noteSensor;

    public SparkPIDController shooter0PID;
    public SparkPIDController shooter1PID;
    public SparkPIDController intakePID;

    public double intakeKp = 1;
    public double intakeKi = 0;
    public double intakeKd = 0;
    public double intakeFF = 0;

    public double shooterKp = 1;
    public double shooterKi = 0;
    public double shooterKd = 0;
    public double shooterFF = 0;

    public FiringSubsystem() {
        shooter0 = new CANSparkMax(3, MotorType.kBrushless);
        shooter1 = new CANSparkMax(4, MotorType.kBrushless);

        intake = new CANSparkMax(5, MotorType.kBrushless);

        noteSensor = new DigitalInput(
                Constants.ArmShooterConstants.ShooterCollectorConstants.NoteSensorPort);
        shooter0PID = shooter0.getPIDController();
        shooter1PID = shooter1.getPIDController();
        intakePID = intake.getPIDController();

        intakePID.setReference(0, CANSparkBase.ControlType.kVelocity);

        intakePID.setP(intakeKp);
        intakePID.setI(intakeKi);
        intakePID.setD(intakeKd);
        intakePID.setFF(intakeFF);

        shooter0PID.setP(shooterKp);
        shooter0PID.setI(shooterKi);
        shooter0PID.setD(shooterKd);
        shooter0PID.setFF(shooterFF);

        shooter1PID.setP(shooterKp);
        shooter1PID.setI(shooterKi);
        shooter1PID.setD(shooterKd);
        shooter1PID.setFF(shooterFF);
    }

    public void spinUpShooter(double speed) {
        SmartDashboard.putString("Firing Status", "Firing Shooter");
        shooter0PID.setReference(speed, ControlType.kVelocity);
        shooter1PID.setReference(speed, ControlType.kVelocity);
    }

    public void spinDownShooter() {
        SmartDashboard.putString("Firing Status", "Spinning Down Shooter");
        shooter0PID.setReference(0, ControlType.kVelocity);
        shooter1PID.setReference(0, ControlType.kVelocity);
    }

    public void spinUpIntake(double speed) {
        SmartDashboard.putString("Firing Status", "Spin up Intake");
        intakePID.setReference(speed, CANSparkBase.ControlType.kVelocity);
    }

    public void spinDownIntake() {
        SmartDashboard.putString("Firing Status", "Spin down Intake");
        intakePID.setReference(0, CANSparkBase.ControlType.kVelocity);

    }

    public void logVals() {
        SmartDashboard.putNumber("shooter0 value", shooter0.get());
        SmartDashboard.putNumber("shooter1 value", shooter1.get());
        SmartDashboard.putNumber("intake value", intake.get());
        SmartDashboard.putBoolean("Note Sensor", noteSensor.get());
    }
}
