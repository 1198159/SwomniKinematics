package com.b1nary.swomni;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;

public class SwomniKinematics {
    public static ModuleState[] kinematics(Vector2D power, double rotation, double cvt, double wheelbase, double trackwidth){
        //start with normal swerve kinematics
        ModuleState[] kinematics = swerveKinematics(power, rotation, wheelbase, trackwidth);
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
            double angle = Math.acos(1 / cvtSplits[i]);

            if (Math.abs(power.getY()) > Math.abs(power.getX()) ^ i % 2 == 0) angle = -angle;

            kinematics[i] = kinematics[i].scale(1/cvtSplits[i]).rotate(angle);
        }
        
        return normalizeStates(kinematics);

        
    }
    public static ModuleState[] swerveKinematics(Vector2D power, double rotation, double wheelbase, double trackwidth){
        return ModuleState.fromVectors(
                power.add(new Vector2D(-wheelbase/2,trackwidth/2).scalarMultiply(rotation)),
                power.add(new Vector2D(-wheelbase/2,-trackwidth/2).scalarMultiply(rotation)),
                power.add(new Vector2D(wheelbase/2,-trackwidth/2).scalarMultiply(rotation)),
                power.add(new Vector2D(wheelbase/2,trackwidth/2).scalarMultiply(rotation))
        );
    }

    public static ModuleState[] normalizeStates(ModuleState[] states){
        double maxModule = 1;
        for (ModuleState state : states) {
            maxModule = Math.max(maxModule, state.getNorm());
        }
        for (int i = 0; i < states.length; i++) {
            states[i] = states[i].scale(1/maxModule);
        }
        return states;
    }
}