package frc.lib;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class OnboardModuleStateTest {
    
    @Test
    public void optimizePositiveDelta() {
        SwerveModuleState desState = new SwerveModuleState();
        /* when the differnce between current and desiered is <= 90 */
        desState.angle = Rotation2d.fromDegrees(35);
        desState.speedMetersPerSecond = 1;
        
        Rotation2d currentAngle =  Rotation2d.fromDegrees(10);

    }
}