package frc.robot.systems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
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

    public double[] getTargetPosition() {
        return getArrayEntry("botpose_wpiblue");

    }

    double[] coordinateValues;

    public double getCoordinateValues(int value) {
        coordinateValues = getArrayEntry("botpose_wpiblue");
        return coordinateValues[value];
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

    public double Zoffset() {
        double[] targetPositionArray = getTargetPosition(); 
        return targetPositionArray[3];
    }

    public double limelightTestValues(int valueEntry) {
        double[] targetPositionArray = getTargetPosition(); 
        return targetPositionArray[valueEntry];
    }

    public double yDistanceFromLimelightAngle() {
        double[] targetPositionArray = getTargetPosition(); 
        double a1 = Math.toRadians(Yoffset());
        //returns the y Distance from the Apriltag
        return ((Constants.aprilTagHeight - Constants.limeLightHeightFromGround) / Math.tan(a1 + Constants.limeLightAngleInRads));
    }

    public double targetYaw() {
        double[] targetPositionArray = getTargetPosition(); 
        return targetPositionArray[5];
    }

 public double xDistanceFromLimelightAngle() {
        double[] targetPositionArray = getTargetPosition(); 
        double tx = Math.toRadians(Xoffset());
        return (yDistanceFromLimelightAngle() / Math.tan(tx));
    }

}
