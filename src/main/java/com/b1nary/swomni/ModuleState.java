package com.b1nary.swomni;

import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;

public final class ModuleState {

    private final double norm, azimuth;

    public ModuleState(Vector2D vector) {
       this(vector.getNorm(), Math.atan2(vector.getY(), vector.getX()));

    }

    public ModuleState(double r, double theta) {
        norm = r;
        azimuth = theta;
    }

    public Vector2D getAsVector(){
        return new Vector2D(norm*Math.cos(azimuth), norm*Math.sin(azimuth));
    }

    public double getNorm() {
        return norm;
    }

    public double getAzimuth() {
        return azimuth;
    }

    public ModuleState scale(double factor){
        return new ModuleState(norm*factor, azimuth);
    }

    public ModuleState rotate(double factor){
        return new ModuleState(norm, (azimuth+factor)%(2*Math.PI));
    }


    public static ModuleState[] fromVectors(Vector2D... vectors){
        ModuleState[] output = new ModuleState[4];
        for (int i = 0; i < vectors.length; i++) {
            output[i] = new ModuleState(vectors[i]);
        }
        return output;
    }
    public static Vector2D[] toVectors(ModuleState... states){
        Vector2D[] output = new Vector2D[4];
        for (int i = 0; i < states.length; i++) {
            output[i] = states[i].getAsVector();
        }
        return output;
    }
}
