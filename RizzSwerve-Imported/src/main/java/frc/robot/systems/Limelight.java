package frc.robot.systems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight {
    
    public boolean tv;
    public double tx;
    public double ty;
    public double ta;
    public double[] botPose;

    public void updateLimelight() {
        tv = LimelightHelpers.getTV("");
        tx = LimelightHelpers.getTX("");
        ty = LimelightHelpers.getTY("");
        ta = LimelightHelpers.getTA("");
        botPose = LimelightHelpers.getBotPose("");


    }


}
