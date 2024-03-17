package frc.lib;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class OnboardModuleStateTest {

    public boolean testEquals(SwerveModuleState a, double b, double tol) {
        return Math.abs(a.angle.getDegrees() - b) < tol;

    }

    /*
     * condition 1 where desidered angle is between 0 and 90 deg above current angle
     */
    @Test
    public void optimize_condition_1() {
        double tol = 0.01;
        SwerveModuleState desState = new SwerveModuleState();

        desState.angle = Rotation2d.fromDegrees(35);
        desState.speedMetersPerSecond = 1;

        Rotation2d currentAngle = Rotation2d.fromDegrees(10);

        SwerveModuleState result = OnboardModuleState.optimize(desState, currentAngle);

        assert (result.speedMetersPerSecond > 0);
        assert (result.angle.getDegrees() == 35);

        desState.angle = Rotation2d.fromDegrees(154);
        desState.speedMetersPerSecond = 1;

        currentAngle = Rotation2d.fromDegrees(100);

        result = OnboardModuleState.optimize(desState, currentAngle);

        assert (result.speedMetersPerSecond > 0);
        assert (result.angle.getDegrees() == 154);

        desState.angle = Rotation2d.fromDegrees(265);
        desState.speedMetersPerSecond = 1;

        currentAngle = Rotation2d.fromDegrees(198);

        result = OnboardModuleState.optimize(desState, currentAngle);

        assert result.speedMetersPerSecond > 0;
        assert result.angle.getDegrees() == 265;

        desState.angle = Rotation2d.fromDegrees(346);
        desState.speedMetersPerSecond = 1;

        currentAngle = Rotation2d.fromDegrees(300);

        result = OnboardModuleState.optimize(desState, currentAngle);

        assert result.speedMetersPerSecond > 0;
        assert result.angle.getDegrees() == 346;

        desState.angle = Rotation2d.fromDegrees(370);
        desState.speedMetersPerSecond = 1;

        currentAngle = Rotation2d.fromDegrees(300);

        result = OnboardModuleState.optimize(desState, currentAngle);

        assert result.speedMetersPerSecond > 0;
        assert testEquals(result, 10, tol);
    }

    /*
     * condition 2 and 3 where desiered angle between 90 deg & 270 deg above current
     * angle
     */
    @Test
    public void optimize_condition_2_and_3() {
        double tol = 0.01;
        SwerveModuleState desState = new SwerveModuleState();
        double desAngle = 0;

        // current angle in quad 1 and and desidered stat in quad 2
        desAngle = 175;
        desState.angle = Rotation2d.fromDegrees(desAngle);
        desState.speedMetersPerSecond = 1;

        Rotation2d currentAngle = Rotation2d.fromDegrees(10);

        SwerveModuleState result = OnboardModuleState.optimize(desState, currentAngle);

        assert (result.speedMetersPerSecond < 0);
        assert (result.angle.getDegrees() == desAngle - 180);

        // current angle in quad 1 and and desidered stat in quad 3
        desAngle = 245;
        desState.angle = Rotation2d.fromDegrees(desAngle);
        desState.speedMetersPerSecond = 1;

        currentAngle = Rotation2d.fromDegrees(10);

        result = OnboardModuleState.optimize(desState, currentAngle);

        assert (result.speedMetersPerSecond < 0);
        assert (testEquals(result, desAngle - 180, tol));

        // current angle in quad 2 and and desidered stat in quad 4
        desAngle = 300;
        desState.angle = Rotation2d.fromDegrees(desAngle);
        desState.speedMetersPerSecond = 1;

        currentAngle = Rotation2d.fromDegrees(200);

        result = OnboardModuleState.optimize(desState, currentAngle);

        // current angle in quad 2 and and desidered stat in quad 1
        desAngle = 400;
        desState.angle = Rotation2d.fromDegrees(desAngle);
        desState.speedMetersPerSecond = 1;

        currentAngle = Rotation2d.fromDegrees(200);

        result = OnboardModuleState.optimize(desState, currentAngle);

        assert (result.speedMetersPerSecond < 0);
        assert (testEquals(result, desAngle - 180, tol));

    }

    /*
     * condition 4 where desidered anglis is bewtween 270 deg and 360 deg above
     * current angle
     */
    @Test
    public void optimize_condiion_4() {
        double tol = 0.01;
        SwerveModuleState desState = new SwerveModuleState();
        double desAngle = 0;

        // current angle in quad 4 and and desidered stat in quad 1
        desAngle = 340;
        desState.angle = Rotation2d.fromDegrees(desAngle);
        desState.speedMetersPerSecond = 1;

        Rotation2d currentAngle = Rotation2d.fromDegrees(10);

        SwerveModuleState result = OnboardModuleState.optimize(desState, currentAngle);

        assert (result.speedMetersPerSecond > 0);
        assert (testEquals(result, desAngle - 360, tol));

        // current angle in quad 2 and and desidered stat in quad 1
        desAngle = 45;
        desState.angle = Rotation2d.fromDegrees(desAngle);
        desState.speedMetersPerSecond = 1;

        currentAngle = Rotation2d.fromDegrees(100);

        result = OnboardModuleState.optimize(desState, currentAngle);

        assert (result.speedMetersPerSecond > 0);
        assert (testEquals(result, desAngle - 360, tol));

    }

    /*
     * condition 5 where desired angle is bewtween -90 and 0 below desired angle
     */
    @Test
    public void optimize_condition_5() {
        double tol = 0.01;
        SwerveModuleState desState = new SwerveModuleState();
        double desAngle = 0;

        // current angle in quad 1 and and desidered is in quad 1 and desired angle less
        // than current
        desAngle = 10;
        desState.angle = Rotation2d.fromDegrees(desAngle);
        desState.speedMetersPerSecond = 1;

        Rotation2d currentAngle = Rotation2d.fromDegrees(10);

        SwerveModuleState result = OnboardModuleState.optimize(desState, currentAngle);

        assert (result.speedMetersPerSecond > 0);
        assert (testEquals(result, desAngle, tol));

        // current angle in quad 2 and and desidered is in quad 2 and desired angle less
        // than current
        desAngle = 100;
        desState.angle = Rotation2d.fromDegrees(desAngle);
        desState.speedMetersPerSecond = 1;

        currentAngle = Rotation2d.fromDegrees(150);

        result = OnboardModuleState.optimize(desState, currentAngle);

        assert (result.speedMetersPerSecond > 0);
        assert (testEquals(result, desAngle, tol));

        // current angle in quad 3 and and desidered is in quad 3 and desired angle less
        // than current
        desAngle = 200;
        desState.angle = Rotation2d.fromDegrees(desAngle);
        desState.speedMetersPerSecond = 1;

        currentAngle = Rotation2d.fromDegrees(253);

        result = OnboardModuleState.optimize(desState, currentAngle);

        assert (result.speedMetersPerSecond > 0);
        assert (testEquals(result, desAngle, tol));

        // current angle in quad 4 and and desidered is in quad 4 and desired angle less
        // than current
        desAngle = 323;
        desState.angle = Rotation2d.fromDegrees(desAngle);
        desState.speedMetersPerSecond = 1;

        currentAngle = Rotation2d.fromDegrees(350);

        result = OnboardModuleState.optimize(desState, currentAngle);

        assert (result.speedMetersPerSecond > 0);
        assert (testEquals(result, desAngle, tol));
    }
}
