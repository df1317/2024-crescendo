package frc.robot.subsytems;

import frc.robot.Constants;
import frc.robot.MockDigitalInput;
import frc.robot.subsystems.ArmSubsystem;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.unmanaged.Unmanaged;

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
        encoderSim.set(Constants.ArmShooterConstants.Arm.EncoderMin);
        limitSwitch.set(true);
        armSub.spinUp(0.5);
        sleep(100);
        // expect positive motor voltage
        assert(motor0Sim.getMotorOutputLeadVoltage() > 0);

        // arm encoder less than minimum, speed is positive, limit switch true
        encoderSim.set(Constants.ArmShooterConstants.Arm.EncoderMin - 0.1);
        limitSwitch.set(true);
        armSub.spinUp(0.5);
        sleep(100);
        // expect motor voltage to be zero
        assert(motor0Sim.getMotorOutputLeadVoltage() == 0);

        // arm encoder greater than maximum, speed is positive, limit switch true
        encoderSim.set(Constants.ArmShooterConstants.Arm.EncoderMax + 0.1);
        limitSwitch.set(true);
        armSub.spinUp(0.5);
        sleep(100);
        // expect positive motor voltage -- bringing the arm back from having gone too far the other way
        assert(motor0Sim.getMotorOutputLeadVoltage() > 0);

        // arm encoder at minimum, speed is positive, limit switch false
        encoderSim.set(Constants.ArmShooterConstants.Arm.EncoderMin);
        limitSwitch.set(false);
        armSub.spinUp(0.5);
        sleep(100);
        // expect zero motor voltage
        assert(motor0Sim.getMotorOutputLeadVoltage() == 0);

        // arm encoder less than minimum, speed is positive, limit switch false
        encoderSim.set(Constants.ArmShooterConstants.Arm.EncoderMin - 0.1);
        limitSwitch.set(false);
        armSub.spinUp(0.5);
        sleep(100);
        // expect motor voltage to be zero
        assert(motor0Sim.getMotorOutputLeadVoltage() == 0);

        // arm encoder greater than maximum, speed is positive, limit switch false
        encoderSim.set(Constants.ArmShooterConstants.Arm.EncoderMax + 0.1);
        limitSwitch.set(false);
        armSub.spinUp(0.5);
        sleep(100);
        // expect zero motor voltage
        assert(motor0Sim.getMotorOutputLeadVoltage() == 0);
    }

    @Test
    public void spinDown() {

    }

    void sleep(long millis) {
        try {
            for(int i = 0; i < millis; i += 10) {
                Unmanaged.feedEnable(20);
                Thread.sleep(10);
            }
        } catch(InterruptedException ex) {
            ex.printStackTrace();
        }
    }
}
