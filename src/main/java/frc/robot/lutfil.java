package frc.robot;

import java.util.ArrayList;

public class lutfil {
    private static double slope = 2.5;
    private static double startDist = 2;
    private static double endDist = 3.6;

    public static double calculateShooterAngle(double dist) {
        // calculate opposite
        double opposite = Constants.Field.speakerZ - Constants.ArmShooterConstants.Arm.jointHeight
                + Constants.ArmShooterConstants.Arm.armLenght
                        * Math.sin(60 - Constants.ArmShooterConstants.Arm.optimizedAngle);
        // calculate adjacent
        double speakerDist = dist;
        double adjacent = speakerDist
                + Constants.ArmShooterConstants.Arm.armLenght
                        * Math.cos(60 - Constants.ArmShooterConstants.Arm.optimizedAngle);
        // return arctan
        double adujustmentAngle = 0;
        if (speakerDist > 2) {
            adujustmentAngle = (speakerDist - startDist) * slope / (endDist - startDist);
        }
        double desAngle = Math.toDegrees(Math.atan2(opposite, adjacent));

        return desAngle + adujustmentAngle;
    }

    public static void lutfils() {
        double startDist = 1.3;
        double offset = 0.5;
        double stopDist = 6;

        ArrayList<Double> returableDist = new ArrayList();
        ArrayList<Double> returableAngles = new ArrayList();

        for (double i = 0; i < (stopDist - startDist) / offset; i++) {
            returableAngles.add(calculateShooterAngle(startDist + i * offset));
            returableDist.add(startDist + i * offset);

        }
        

        System.out.println(returableDist.toString().replace("[", "{").replace("]", "}"));

    }

    public static void main(String[] args) {
        lutfils();
    }

}
