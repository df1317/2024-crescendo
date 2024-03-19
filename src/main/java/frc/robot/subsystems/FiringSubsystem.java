package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
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

    public FiringSubsystem() {
        shooter0 = new CANSparkMax(3, MotorType.kBrushless);
        shooter1 = new CANSparkMax(4, MotorType.kBrushless);

        intake = new CANSparkMax(5, MotorType.kBrushless);

        noteSensor = new DigitalInput(
                Constants.ArmShooterConstants.ShooterCollectorConstants.NoteSensorPort);
        shooter0PID = shooter0.getPIDController();
        shooter1PID = shooter1.getPIDController();
        intakePID = intake.getPIDController();
    }

    public void spinUpShooter(double speed) {
        SmartDashboard.putString("Firing Status", "Firing Shooter");
        shooter0.set(speed);
        shooter1.set(speed);
    }

    public void spinDownShooter() {
        SmartDashboard.putString("Firing Status", "Spinning Down Shooter");
        shooter0.stopMotor();
        shooter1.stopMotor();
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
        SmartDashboard.putNumber("shooter0 value", shooter0.get());
        SmartDashboard.putNumber("shooter1 value", shooter1.get());
        SmartDashboard.putNumber("intake value", intake.get());
        SmartDashboard.putBoolean("Note Sensor", noteSensor.get());
    }
}
