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
        return getArrayEntry("targetpose_cameraspace");
        /** Array Positions
         * 0 = tx
         * 1 = ty
         * 2 = tz
         * 3 = pitch
         * 4 = yaw
         * 5 = roll
         */

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
        return targetPositionArray[2];
    }

    public double limelightTestValues(int valueEntry) {
        double[] targetPositionArray = getTargetPosition(); 
        return targetPositionArray[valueEntry];
    }

    public double yDistanceFromLimelightAngle() {
        double[] targetPositionArray = getTargetPosition(); 
        double a1 = Math.toRadians(targetPositionArray[1]);
        //returns the y Distance from the Apriltag 
        return ((Constants.aprilTagHeight - Constants.limeLightHeightFromGround) / Math.tan(a1 + Constants.limeLightAngleInRads));
    }

    public double targetYaw() {
        double[] targetPositionArray = getTargetPosition(); 
        return targetPositionArray[4];
    }

    public double xDistanceFromLimelightAngle() {
        double[] targetPositionArray = getTargetPosition(); 
        double tx = Math.toRadians(targetPositionArray[0]);
        return (yDistanceFromLimelightAngle() / Math.tan(tx));
    }

}
