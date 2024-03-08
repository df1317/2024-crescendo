package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
    // put initial code here
    public ArmSubsystem() {
        SmartDashboard.putNumber("Arm Kp", Kp);
        SmartDashboard.putNumber("Arm Ki", Ki);
        SmartDashboard.putNumber("Arm Kd", Kd);
    }

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
    private boolean editablePIDConstants = true;
    public double Kp = 1.8 / 360;
    public double Ki = 0.05 / 360;
    public double Kd = 0 / 360;

    public void spinUp(double speed) {
        SmartDashboard.putNumber("Pre Arm Motor Speed", speed);
        SmartDashboard.putNumber("Arm Encoder", encoder.get());

        if (encoder.get() > Constants.ArmShooterConstants.Arm.EncoderMax && speed < 0) {
            speed = 0;
        } else if (encoder.get() < Constants.ArmShooterConstants.Arm.EncoderMin && speed > 0) {
            speed = 0;
        }

        if (!limitSwitch.get() && speed > 0) {
            speed = 0;
        }

        if (speed == 0) {
            SmartDashboard.putBoolean("Motor E-Stopped", true);
        } else {
            SmartDashboard.putBoolean("Motor E-Stopped", false);
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
        SmartDashboard.putNumber("Arm Motor Speed", motor0.getMotorOutputPercent());

        // get and set PID constants from SmartDashboard
        if (editablePIDConstants) {
            Kp = SmartDashboard.getNumber("Arm Kp", Kp);
            Ki = SmartDashboard.getNumber("Arm Ki", Ki);
            Kd = SmartDashboard.getNumber("Arm Kd", Kd);
        }

        SmartDashboard.putNumber("Arm Kp", Kp);
        SmartDashboard.putNumber("Arm Ki", Ki);
        SmartDashboard.putNumber("Arm Kd", Kd);
    }

    public double getArmAngle() {
        double armAngle = (encoder.get() - Constants.ArmShooterConstants.Arm.EncoderMin) * 360
                + Constants.ArmShooterConstants.Arm.angle;
        return armAngle;
    }

    public void setAngle(double newArmAngle) {
        newArmAngle = (newArmAngle - Constants.ArmShooterConstants.Arm.angle) / 360
                + Constants.ArmShooterConstants.Arm.EncoderMin;

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

        double oldErrorSum = errorSum;
        errorSum += error;

        double errorDif = error - prevError;
        double motorPower = Kp * error + Ki * errorSum + Kd * errorDif;
        prevError = error;

        // anti integral windup https://www.youtube.com/watch?v=NVLXCwc8HzM

        double preClamp = motorPower;

        if (motorPower > 1) {
            motorPower = 1;
        } else if (motorPower < -1) {
            motorPower = -1;
        }
        double postClamp = motorPower;

        boolean inRange = preClamp != postClamp;

        boolean errorGrowing = preClamp > 0 == error > 0;

        boolean intClamp = inRange && errorGrowing;

        if (intClamp) {
            errorSum = oldErrorSum;
        }

        motorPower = Kp * error + Ki * errorSum + Kd * errorDif;

        spinUp(motorPower);
    }
}