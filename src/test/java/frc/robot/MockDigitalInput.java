package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;

public class MockDigitalInput extends DigitalInput {

    protected boolean state;

    public MockDigitalInput(int channel) {
        super(channel);
    }
    
    public MockDigitalInput(DigitalInput digitalInput) {
        this(cleanupRealDigitalInput(digitalInput));
    }

    private static int cleanupRealDigitalInput(DigitalInput digitalInput) {
        int channel = digitalInput.getChannel();
        digitalInput.close();
        return channel;
    }

    public void set(boolean state) {
        this.state = state;
    }

    @Override
    public boolean get() {
        return state;
    }
}
