package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class FiringSubsystem extends SubsystemBase {
    private Spark shooter0 = new Spark(3);
    private Spark shooter1 = new Spark(4);

    private Spark intake = new Spark(5);

    public DigitalInput noteSensor = new DigitalInput(
            Constants.ArmShooterConstants.ShooterCollectorConstants.NoteSensorPort);

    public void spinUpShooter(double speed) {
        SmartDashboard.putString("Firing Status", "Firing");
        shooter0.set(speed);
        shooter1.set(speed);
    }

    public void spinDownShooter() {
        SmartDashboard.putString("Firing Status", "Armed");
        shooter0.stopMotor();
        shooter1.stopMotor();
    }

    public void spinUpIntake(double speed) {
        intake.set(speed);
    }

    public void spinDownIntake() {
        intake.stopMotor();
    }
}
