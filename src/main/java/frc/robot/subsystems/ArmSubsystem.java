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
    public double Kp = 2.5 / 360;
    public double Ki = 0 / 360;
    public double Kd = 0 / 360;
    public double Kg = -0.15; // half of calculation

    public void spinUp(double speed) {
        SmartDashboard.putNumber("Pre Arm Motor Speed", speed);
        SmartDashboard.putNumber("Arm Encoder", encoder.get());

        if (getAngle() < Constants.ArmShooterConstants.Arm.EncoderMax && speed < 0) {
            speed = 0;
        } else if (getAngle() > Constants.ArmShooterConstants.Arm.EncoderMin && speed > 0) {
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
        SmartDashboard.putNumber("ShooterAngle", getAngle());

        // get and set PID constants from SmartDashboard
        if (editablePIDConstants) {
            Kp = SmartDashboard.getNumber("Arm Kp", Kp) / 360;
            Ki = SmartDashboard.getNumber("Arm Ki", Ki) / 360;
            Kd = SmartDashboard.getNumber("Arm Kd", Kd) / 360;
            Kg = SmartDashboard.getNumber("Arm Kg", Kg);
        }

        SmartDashboard.putNumber("Arm Kp", Kp * 360);
        SmartDashboard.putNumber("Arm Ki", Ki * 360);
        SmartDashboard.putNumber("Arm Kd", Kd * 360);
        SmartDashboard.putNumber("Arm Kg", Kg);
    }

    /** set setpoint */
    public void setAngle(double setpoint) {
        if (setpoint > Constants.ArmShooterConstants.Arm.EncoderMin) {
            armSetPoint = Constants.ArmShooterConstants.Arm.EncoderMin;
        } else if (setpoint < Constants.ArmShooterConstants.Arm.EncoderMax) {
            armSetPoint = Constants.ArmShooterConstants.Arm.EncoderMax;
        } else {
            armSetPoint = setpoint;
        }
    }

    /** convert from encoder rotation to shooter angle */
    public double getAngle() {
        // convert encoder rotation to encoder degrees
        // encoder reads negative when arm goes up
        double degreesEncoder = (encoder.get() - Constants.ArmShooterConstants.Arm.encoderZero) * -360;
        // convert encoder degrees to shooter angle
        return Constants.ArmShooterConstants.Arm.shooterArmOffset - degreesEncoder;
    }

    /** Convert from shooter angle to angle with which gravity acts on arm */
    public double calculateArmAngle(double setPoint) {
        return Constants.ArmShooterConstants.Arm.shooterGravOffset - setPoint;
    }

    public void runPID() {
        double armGravAngle = calculateArmAngle(armSetPoint);
        double error = armSetPoint - getAngle();

        double oldErrorSum = errorSum;
        errorSum += error;

        double errorDif = error - prevError;
        double motorPower = Kp * error + Ki * errorSum + Kd * errorDif + Kg * Math.sin(Math.toRadians(armGravAngle));
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

        motorPower = Kp * error + Ki * errorSum + Kd * errorDif + Kg * Math.sin(Math.toRadians(armGravAngle));

        spinUp(motorPower);
    }
}