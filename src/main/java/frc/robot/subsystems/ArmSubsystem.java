package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
    // get the firng subsystem
    private TalonSRX motor = new TalonSRX(Constants.ArmShooterConstants.Arm.MotorID);
    public DutyCycleEncoder encoder = new DutyCycleEncoder(0);

    public void spinUp(double speed) {
        motor.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, speed);
    }

    public void spinDown() {
        motor.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, 0);
    }

    public void periodic() {
        SmartDashboard.putNumber("Arm Encoder", encoder.get());
    }
}