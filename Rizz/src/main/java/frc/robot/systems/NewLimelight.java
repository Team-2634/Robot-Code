package frc.robot.systems;

import frc.robot.LimelightHelpers;

public class NewLimelight {

    // --- Variables (before the robot even moves) ---
    
    // Constants for the camera
    final double cam_height_from_ground = 0.0; // height of camera from the ground
    final double cam_Zoffset = 0.0; // depth offset from robot center
    final double cam_Xoffset = 0.0; // sideways offset from robot center
    final double cam_pitch_offset = 0.0; // Pitch offset of camera (in degrees)

    // tag infomration
    int tag_id;  // the id of detected tag
    double tag_height_from_ground; // height of detected tag from the ground
    double tag_pitch;
    double tag_yaw;

    // measured values
    double tag_height_difference; // height difference between camera and detected tag
    double cam_angle_pitch; // up and down - the angle of detected tag
    double cam_angle_yaw; // left and right - the angle of detected tag
    double cam_Zpos; // camera distance from tag in Z axis (depth)

    double cam_pitch = LimelightHelpers.getTX(""); // Yaw
    double cam_yaw = LimelightHelpers.getTY(""); // Pitch

    public void detectedTagInfo(int id){
        
        tag_id = id;
        cam_angle_pitch = cam_pitch - cam_pitch_offset;
        cam_angle_yaw = cam_yaw;

        if (id == 1){   
            tag_height_from_ground = 30;
        }

        System.out.format("Tag ID: " + tag_id);
        System.out.println("Camera Pitch: " + cam_angle_pitch);
        System.out.println("Camera Yaw: " + cam_angle_yaw);
    }


    public double getTagHeightDifferance(){
        return tag_height_from_ground - cam_height_from_ground;
    }
    public double getRobotRot(){
        return cam_angle_yaw;
    }

    /**
     * Finds the depth distance of the limelight camera on the Z axis
     * **/
    public double getRobotZdistance(){
        cam_Zpos = getTagHeightDifferance() / Math.tan(cam_angle_pitch);
        return cam_Zpos - cam_Zoffset;
    }

    /**
     * Finds the sideways distance of the limelight camera on the X axis
     * **/
    public double getRobotXdistance(){
        double cam_Xpos = cam_Zpos * Math.tan(cam_angle_yaw); // camera distance from tag in X axis (sideways)
        return cam_Xpos - cam_Xoffset;
    }

    /**
     * Determines the position of the robot from the detected tag using the camera's position and offsets
     * **/
    public double getRobotToTagDistanceDirect(){

        return Math.hypot(getRobotXdistance(), getRobotZdistance());

    }

     /**
     * Calculates the movement needed to get to a target spot (relative to the tag) and moves the robot there
     * **/
    public void go_to_target_spot(double tagTargetXdist, double tagTargetZdist, double tagTargetRot){

        double robotCurrentXpos = getRobotXdistance(); // current robot position (sideways)
        double robotCurrentZpos = getRobotZdistance(); // current robot position  (depth)
        double robotCurrentRot = getRobotRot(); // current robot rotation  (degrees)
        
        // robot to target differences
        double Xdifference = tagTargetXdist - robotCurrentXpos; // difference between robot position and target position (sideways)
        double Zdifference = tagTargetZdist - robotCurrentZpos; // difference between robot position and target position (depth)
        double rotDifference = tagTargetRot - robotCurrentRot; // difference between robot rotation and target rotation (degrees)


    }

}