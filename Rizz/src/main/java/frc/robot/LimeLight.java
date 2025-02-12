package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimeLight {

    private static NetworkTable limelight = NetworkTableInstance.getDefault().getTable("Limelight name here idk it");

    public double getDoubleEntry(String entry) {
        return limelight.getEntry(entry).getDouble(0.0);
    }

    public double[] getArrayEntry (String entry) {
        return limelight.getEntry(entry).getDoubleArray(new double[6]);
    }

    public double getID() {
        return getDoubleEntry("tid");
    }

    public boolean validTarget() {
        return getDoubleEntry("tv") == 1.0;
    }

    public double getTargetArea() {
        return getDoubleEntry("ta");
    }

    public double Xoffset() {
        return getDoubleEntry("tx");
    }

    public double Yoffset() {
        return getDoubleEntry("ty");
    }


}