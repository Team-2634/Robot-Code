package frc.robot.systems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LimeLight {
    private static NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");

    public double getDoubleEntry(String entry) {
        return limelightTable.getEntry(entry).getDouble(0.0);
    }

    public double[] getArrayEntry (String entry) {
        return limelightTable.getEntry(entry).getDoubleArray(new double[6]);
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

    public double yDistanceFromLimelightAngle() {
        double a1 = Math.toRadians(Yoffset());
        //returns the y Distance from the Apriltag 
        return ((Constants.aprilTagHeight - Constants.limeLightHeightFromGround) / Math.tan(a1 + Constants.limeLightAngleInRads));
    }

    public double xDistanceFromLimelightAngle() {
        return (yDistanceFromLimelightAngle() / Math.tan(Math.toRadians(Xoffset())));
    }

}
