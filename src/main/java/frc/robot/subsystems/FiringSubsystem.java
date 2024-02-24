package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class FiringSubsystem extends SubsystemBase {
    private Spark motor0 = new Spark(Constants.ArmShooterConstants.ShooterCollectorConstants.Firing.MotorID0);
    private Spark motor1 = new Spark(Constants.ArmShooterConstants.ShooterCollectorConstants.Firing.MotorID1);

    public DigitalInput noteSensor = new DigitalInput(Constants.ArmShooterConstants.ShooterCollectorConstants.NoteSensorPort);

    public void spinUp(double speed) {
        SmartDashboard.putString("Firing Status", "Firing");
        motor0.set(-speed);
        motor1.set(-speed);
    }

    public void spinDown() {
        SmartDashboard.putString("Firing Status", "Armed");
        motor0.stopMotor();
        motor1.stopMotor();
    }
}
