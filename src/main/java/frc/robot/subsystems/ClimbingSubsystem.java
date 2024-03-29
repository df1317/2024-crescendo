package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class ClimbingSubsystem extends SubsystemBase {
    // device numbers to be changed when we have the devices
    private TalonSRX motorLeft = new TalonSRX(6);
    private TalonSRX motorRight = new TalonSRX(7);

    public void setLeftArm(double output) {
        motorLeft.set(TalonSRXControlMode.PercentOutput, output);
    }

    public void setRightArm(double output) {
        motorRight.set(TalonSRXControlMode.PercentOutput, output);
    }

    // holds the value of the arms
    public void stop() {
        motorLeft.set(TalonSRXControlMode.PercentOutput, 0);
        motorRight.set(TalonSRXControlMode.PercentOutput, 0);
    }

}
