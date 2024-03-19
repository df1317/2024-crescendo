package frc.robot;

import com.ctre.phoenix.unmanaged.Unmanaged;

public class TestUtil {
    
    public static void sleep(long millis) {
        try {
            for(int i = 0; i < millis; i += 10) {
                Unmanaged.feedEnable(20);
                Thread.sleep(10);
            }
        } catch(InterruptedException ex) {
            ex.printStackTrace();
        }
    }

    public static void sleep() {
        sleep(150);
    }
}
