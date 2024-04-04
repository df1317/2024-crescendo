package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
    // get the firng subsystem
    public TalonSRX motor0 = new TalonSRX(Constants.ArmShooterConstants.Arm.MotorID0);
    public TalonSRX motor1 = new TalonSRX(Constants.ArmShooterConstants.Arm.MotorID1);
    public DutyCycleEncoder encoder = new DutyCycleEncoder(Constants.ArmShooterConstants.Arm.EncoderPort);
    public DigitalInput limitSwitch = new DigitalInput(Constants.ArmShooterConstants.Arm.LimitSwitchPort);

    private double armSetPoint = Constants.ArmShooterConstants.Arm.EncoderMin;
    public static final double armRange = Constants.ArmShooterConstants.Arm.EncoderMax
            - Constants.ArmShooterConstants.Arm.EncoderMin;

    // TO-DO make constants
    private boolean editablePIDConstants = true;
    public double Kp = 3.5 / 360;
    public double Ki = 0.1 / 360;
    public double Kd = 0.1 / 360;

    public double Ks = 0.07;
    public double Kg = 0.13;
    public double Kv = 1.2;

    public double armVelocity; // from degrees/s

    PIDController pidController;
    ArmFeedforward armFeedforward;

    // put initial code here
    public ArmSubsystem() {
        pidController = new PIDController(Kp, Ki, Kd);
        armFeedforward = new ArmFeedforward(Ks, Kg, Kv);

        SmartDashboard.putNumber("Arm Kp", Kp * 360);
        SmartDashboard.putNumber("Arm Ki", Ki * 360);
        SmartDashboard.putNumber("Arm Kd", Kd * 360);

        SmartDashboard.putNumber("Arm Ks", Ks);
        SmartDashboard.putNumber("Arm Kg", Kg);
        SmartDashboard.putNumber("Arm Kv", Kv);
    }

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
        SmartDashboard.putNumber("calc arm angle: ", calculateArmAngle(armSetPoint));
        SmartDashboard.putNumber("arm velo:", armVelocity);
        SmartDashboard.putNumber("Current Arm Angle", getAngle());
        SmartDashboard.putNumber("Desired Arm Angle", armSetPoint);

        if (!limitSwitch.get()) {
            encoder.reset();
        }

        // get and set PID constants from SmartDashboard
        if (editablePIDConstants) {
            Kp = SmartDashboard.getNumber("Arm Kp", Kp) / 360;
            Ki = SmartDashboard.getNumber("Arm Ki", Ki) / 360;
            Kd = SmartDashboard.getNumber("Arm Kd", Kd) / 360;

            Ks = SmartDashboard.getNumber("Arm Ks", Ks);
            Kg = SmartDashboard.getNumber("Arm Kg", Kg);
            Kv = SmartDashboard.getNumber("Arm Kv", Kv);

            if (Kp != pidController.getP() || Ki != pidController.getI() || Kd != pidController.getD()) {
                pidController.setP(Kp);
                pidController.setI(Ki);
                pidController.setD(Kd);
            }

            if (Ks != armFeedforward.ks || Kg != armFeedforward.kg || Kv != armFeedforward.kv) {
                armFeedforward = new ArmFeedforward(Ks, Kg, Kv);
            }
        }

        SmartDashboard.putNumber("Arm Kp", Kp * 360);
        SmartDashboard.putNumber("Arm Ki", Ki * 360);
        SmartDashboard.putNumber("Arm Kd", Kd * 360);

        SmartDashboard.putNumber("Arm Ks", Ks);
        SmartDashboard.putNumber("Arm Kg", Kg);
        SmartDashboard.putNumber("Arm Kv", Kv);
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

    /**
     * returns arm angle relitive to horizantal
     * means horizantal is zero deg
     */
    public double getArmAngle() {
        // arm rest three deg below horizantal
        return encoder.get() * -360 - 12;
    }

    /** convert from encoder rotation to shooter angle */
    public double getAngle() {
        // convert encoder rotation to encoder degrees
        // encoder reads negative when arm goes up

        // convert encoder degrees to shooter angle
        return Constants.ArmShooterConstants.Arm.shooterArmOffset - getArmAngle();
    }

    /** Convert from shooter angle to angle with which gravity acts on arm */
    public double calculateArmAngle(double setPoint) {
        return Constants.ArmShooterConstants.Arm.shooterGravOffset - setPoint - 90;
    }

    public void runPID() {
        armVelocity = -pidController.calculate(getAngle(), armSetPoint);

        double motorPower = -armFeedforward.calculate(Math.toRadians(calculateArmAngle(armSetPoint)), armVelocity);

        spinUp(motorPower);
    }
}