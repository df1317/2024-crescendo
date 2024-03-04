package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
    // get the firng subsystem
    private TalonSRX motor0 = new TalonSRX(Constants.ArmShooterConstants.Arm.MotorID0);
    private TalonSRX motor1 = new TalonSRX(Constants.ArmShooterConstants.Arm.MotorID1);
    public DutyCycleEncoder encoder = new DutyCycleEncoder(Constants.ArmShooterConstants.Arm.EncoderPort);
    public DigitalInput limitSwitch = new DigitalInput(Constants.ArmShooterConstants.Arm.LimitSwitchPort);

    private double armSetPoint = Constants.ArmShooterConstants.Arm.EncoderMin;
    private double errorSum = 0;
    private double prevError = 0;
    public static final double armRange = Constants.ArmShooterConstants.Arm.EncoderMax
            - Constants.ArmShooterConstants.Arm.EncoderMin;

    // TO-DO make constants
    public double Kp = 2.5;
    public double Ki = 0;
    public double Kd = 0;

    public void spinUp(double speed) {
        if (limitSwitch.get() && speed < 0) {
            speed = 0;
        }

        motor0.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, speed);
        motor1.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, speed);
    }

    public void spinDown() {
        motor0.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, 0);
        motor1.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, 0);
    }

    public void periodic() {
        SmartDashboard.putNumber("Arm Encoder", encoder.get());
        SmartDashboard.putBoolean("Limit Switch", limitSwitch.get());
    }

    public double getArmAngle() {
        double armAngle = encoder.get();
        return armAngle;
    }

    public void setAngle(double newArmAngle) {
        if (newArmAngle < Constants.ArmShooterConstants.Arm.EncoderMin) {
            armSetPoint = Constants.ArmShooterConstants.Arm.EncoderMin;
        } else if (newArmAngle > Constants.ArmShooterConstants.Arm.EncoderMax) {
            armSetPoint = Constants.ArmShooterConstants.Arm.EncoderMax;
        } else {
            armSetPoint = newArmAngle;
        }
    }

    public void runPID() {
        double error = armSetPoint - getArmAngle();
        errorSum += error;
        if (errorSum < Constants.ArmShooterConstants.Arm.EncoderMin + 0.2 * armRange) {
            errorSum = Constants.ArmShooterConstants.Arm.EncoderMin + 0.2 * armRange;
        } else if (errorSum > Constants.ArmShooterConstants.Arm.EncoderMax - 0.2 * armRange) {
            errorSum = Constants.ArmShooterConstants.Arm.EncoderMax - 0.2 * armRange;
        }
        double errorDif = error - prevError;
        double motorPower = Kp * error + Ki * errorSum + Kd * errorDif;
        prevError = error;
        spinUp(motorPower);
    }
}