package frc.robot.subsytems;

import frc.robot.Constants;
import frc.robot.MockDigitalInput;
import frc.robot.TestUtil;
import frc.robot.subsystems.ArmSubsystem;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;

import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;

import edu.wpi.first.hal.HAL;

public class ArmSubsystemTest {

    static ArmSubsystem armSub;
    static TalonSRXSimCollection motor0Sim;
    static TalonSRXSimCollection motor1Sim;
    static DutyCycleEncoderSim encoderSim;
    static MockDigitalInput limitSwitch;

    @BeforeAll
    static void setupSpec() {
        assert HAL.initialize(500, 0);   // the examples all have this, but it seems to be unnecessary?
        armSub = new ArmSubsystem();
        motor0Sim = armSub.motor0.getSimCollection();
        motor1Sim = armSub.motor1.getSimCollection();
        encoderSim = new DutyCycleEncoderSim(armSub.encoder);  // throws exception if this is set up more than once
        limitSwitch = new MockDigitalInput(armSub.limitSwitch);
        armSub.limitSwitch = limitSwitch;
    }

    @Test
    public void spinUp() {
        // arm encoder at minimum, speed is positive, limit switch true
        encoderSim.set(calculateEncoderValue(Constants.ArmShooterConstants.Arm.EncoderMin));
        limitSwitch.set(true);
        armSub.spinUp(0.5);
        TestUtil.sleep();
        // expect positive motor voltage
        assert(motor0Sim.getMotorOutputLeadVoltage() > 0);

        // arm encoder less than minimum, speed is positive, limit switch true
        encoderSim.set(calculateEncoderValue(Constants.ArmShooterConstants.Arm.EncoderMin + 0.1));
        limitSwitch.set(true);
        armSub.spinUp(0.5);
        TestUtil.sleep();
        // expect motor voltage to be zero
        assert(motor0Sim.getMotorOutputLeadVoltage() == 0);

        // arm encoder greater than maximum, speed is positive, limit switch true
        encoderSim.set(calculateEncoderValue(Constants.ArmShooterConstants.Arm.EncoderMax - 0.1));
        limitSwitch.set(true);
        armSub.spinUp(0.5);
        TestUtil.sleep();
        // expect positive motor voltage -- bringing the arm back from having gone too far the other way
        assert(motor0Sim.getMotorOutputLeadVoltage() > 0);

        // arm encoder at minimum, speed is positive, limit switch false
        encoderSim.set(calculateEncoderValue(Constants.ArmShooterConstants.Arm.EncoderMin));
        limitSwitch.set(false);
        armSub.spinUp(0.5);
        TestUtil.sleep();
        // expect zero motor voltage
        assert(motor0Sim.getMotorOutputLeadVoltage() == 0);

        // arm encoder less than minimum, speed is positive, limit switch false
        encoderSim.set(calculateEncoderValue(Constants.ArmShooterConstants.Arm.EncoderMin + 0.1));
        limitSwitch.set(false);
        armSub.spinUp(0.5);
        TestUtil.sleep();
        // expect motor voltage to be zero
        assert(motor0Sim.getMotorOutputLeadVoltage() == 0);

        // arm encoder greater than maximum, speed is positive, limit switch false
        encoderSim.set(calculateEncoderValue(Constants.ArmShooterConstants.Arm.EncoderMax - 0.1));
        limitSwitch.set(false);
        armSub.spinUp(0.5);
        TestUtil.sleep();
        // expect zero motor voltage
        assert(motor0Sim.getMotorOutputLeadVoltage() == 0);


        // arm encoder at maximum, speed is negative, limit switch true
        encoderSim.set(calculateEncoderValue(Constants.ArmShooterConstants.Arm.EncoderMax));
        limitSwitch.set(true);
        armSub.spinUp(-0.5);
        TestUtil.sleep();
        // expect negative motor voltage
        assert(motor0Sim.getMotorOutputLeadVoltage() < 0);

        // arm encoder greater than maximum, speed is negative, limit switch true
        encoderSim.set(calculateEncoderValue(Constants.ArmShooterConstants.Arm.EncoderMax - 0.1));
        limitSwitch.set(true);
        armSub.spinUp(-0.5);
        TestUtil.sleep();
        // expect motor voltage to be zero
        assert(motor0Sim.getMotorOutputLeadVoltage() == 0);

        // arm encoder less than minimum, speed is negative, limit switch true
        encoderSim.set(calculateEncoderValue(Constants.ArmShooterConstants.Arm.EncoderMin + 0.1));
        limitSwitch.set(true);
        armSub.spinUp(-0.5);
        TestUtil.sleep();
        // expect negative motor voltage -- bringing the arm back from having gone too far the other way
        assert(motor0Sim.getMotorOutputLeadVoltage() < 0);

        // arm encoder at maximum, speed is negative, limit switch false
        encoderSim.set(calculateEncoderValue(Constants.ArmShooterConstants.Arm.EncoderMax));
        limitSwitch.set(false);
        armSub.spinUp(-0.5);
        TestUtil.sleep();
        // expect negative motor voltage
        assert(motor0Sim.getMotorOutputLeadVoltage() < 0);
    }

    @Test
    public void getAngle() {
        // is this a test for armSub.getAngle(_)? or for calculateEncoderalue(_)?

        encoderSim.set(calculateEncoderValue(Constants.ArmShooterConstants.Arm.EncoderMax));
        assert armSub.getAngle() == Constants.ArmShooterConstants.Arm.EncoderMax;

        encoderSim.set(calculateEncoderValue(Constants.ArmShooterConstants.Arm.EncoderMin));
        assert armSub.getAngle() == Constants.ArmShooterConstants.Arm.EncoderMin;
    }

    private double calculateEncoderValue(double armAngle) {
        // angle = Constants.ArmShooterConstants.Arm.shooterArmOffset - ((encoder.get() - Constants.ArmShooterConstants.Arm.encoderZero) * -360)
        double encoderVal = ((Constants.ArmShooterConstants.Arm.shooterArmOffset - armAngle) / -360) + Constants.ArmShooterConstants.Arm.encoderZero;
        return encoderVal;
    }
}
