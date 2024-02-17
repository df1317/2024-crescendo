package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
    // get the firng subsystem
    private TalonSRX motor0 = new TalonSRX(Constants.ArmShooterConstants.Arm.MotorID0);
    private TalonSRX motor1 = new TalonSRX(Constants.ArmShooterConstants.Arm.MotorID1);
    public DutyCycleEncoder encoder = new DutyCycleEncoder(Constants.ArmShooterConstants.Arm.EncoderPort);

    public void spinUp(double speed) {
        motor0.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, speed);
        motor1.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, speed);
    }

    public void spinDown() {
        motor0.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, 0);
        motor1.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, 0);
    }

    public void periodic() {
        SmartDashboard.putNumber("Arm Encoder", encoder.get());
    }
}