package frc.robot;

import java.util.ArrayList;
import java.util.Arrays;

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

        ArrayList<double[]> returableDist = new ArrayList<>();

        for (double i = 0; i < (stopDist - startDist) / offset; i++) {
            double e[] = { startDist + i * offset, calculateShooterAngle(startDist + i * offset) };
            returableDist.add(e);
        }

        String stringArray = "{";

        for (int i = 0; i < returableDist.size(); i++) {
            stringArray += Arrays.toString(returableDist.get(i));

            if (i < returableDist.size() - 1) {
                stringArray += ", ";
            }
        }

        stringArray += "}";

        System.out.println(stringArray.replace("[", "{").replace("]", "}"));
    }

    public static void main(String[] args) {
        lutfils();
    }

}
