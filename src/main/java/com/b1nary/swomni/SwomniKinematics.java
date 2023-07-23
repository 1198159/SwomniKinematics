package com.b1nary.swomni;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;

public class SwomniKinematics {
    public static Vector2D[] kinematics(Vector2D power, double rotation, double cvt, double wheelbase, double trackwidth){
        //start with normal swerve kinematics
        Vector2D[] kinematics = swerveKinematics(power, rotation, wheelbase, trackwidth);
        //initialize array for cvt amounts per module
        double[] cvtSplits = new double[]{1,1,1,1};
        // Calculate magnitude and CVT split for each module.

        for (int i = 0; i < 4; i++) {

            double norm = kinematics[i].getNorm();
            int oppositeModule = (i + 2) % 4;
            double newCVT = 2 * cvt * norm / (norm + kinematics[oppositeModule].getNorm());

            if(newCVT < 1) cvtSplits[oppositeModule] += newCVT - 1;
            else cvtSplits[i] = newCVT;
        }

        // Calculate and adjust vectors for each module
        for (int i = 0; i < 4; i++) {
            Vector2D v = kinematics[i].scalarMultiply(1/cvtSplits[i]);
            double angle = Math.acos(1 / cvtSplits[i]);

            if (Math.abs(power.getY()) > Math.abs(power.getX()) ^ i % 2 == 0) angle = -angle;


            kinematics[i] = null;//v.rotated(angle);

        }

        return normalizeVectors(kinematics);


    }
    public static Vector2D[] swerveKinematics(Vector2D power, double rotation, double wheelbase, double trackwidth){
        return new Vector2D[]{
                power.add(new Vector2D(-wheelbase/2,trackwidth/2).scalarMultiply(rotation)),
                power.add(new Vector2D(-wheelbase/2,-trackwidth/2).scalarMultiply(rotation)),
                power.add(new Vector2D(wheelbase/2,-trackwidth/2).scalarMultiply(rotation)),
                power.add(new Vector2D(wheelbase/2,trackwidth/2).scalarMultiply(rotation))
        };
    }

    public static Vector2D[] normalizeVectors(Vector2D[] states){
        double maxModule = 1;
        Vector2D[] output = new Vector2D[4];
        for (Vector2D state : states) {
            maxModule = Math.max(maxModule, state.getNorm());
        }
        for (int i = 0; i < states.length; i++) {
            output[i] = states[i].scalarMultiply(1/maxModule);
        }
        return output;
    }
}