package frc.robot;



public class Constants {
    //unchanging final stuff
    public final static double talonEncoder_TicksPerRev = 2048;
    public final static double neoEncoder_TicksPerRev = 42;

    public final static double frontLeftAbsEncoderOffset = 197.19;
    public final static double frontRightAbsEncoderOffset = 31.8;
    public final static double backLeftAbsEncoderOffset = 285.205;
    public final static double backRightAbsEncoderOffset = 50.4;

    public final static double maxDegree = 360;

    public final static double maxVelocity_MetersPerSeconds = 6;
    public final static double maxAccel_MetersPerSecondsSquared = 3;

    //configurable stuff

    //device IDs
    public final static int frontLeftDriveID = 0;
    public final static int frontRightDriveID = 2;
    public final static int backLeftDriveID = 4;
    public final static int backRightDriveID = 6;

    public final static int frontLeftSteerID = 1;
    public final static int frontRightSteerID = 3;
    public final static int backLeftSteerID = 5;
    public final static int backRightSteerID = 7;
    
    public final static int frontLeftAbsEncoderID = 3;
    public final static int frontRightAbsEncoderID = 0;
    public final static int backLeftAbsEncoderID = 2;
    public final static int backRightAbsEncoderID = 1;

    //PID values
    public final static double kpDrive = 0.3;
    public final static double kiDrive = 0;
    public final static double kdDrive = 0;

    public final static double kpAuto = 0.2;
    public final static double kiAuto = 0;
    public final static double kdAuto = 0.0000075;

    public final static double kpAutoRotate = 0.1;
    public final static double kiAutoRotate = 0;
    public final static double kdAutoRotate = 0.0000075;
    
    public final static double autoPositionToleranceMeters = 0.2;
    public final static double autoRotationToleranceRadians = 0.2;
    
    //auto
    public final static double maxAutoVelocity = 0.1;
    public final static double maxAutoAccel = 0.1;

    //drivetrain properties
    public final static double wheelDiameterInches = 3.75;
    public final static double driveMotorGearRatio = 1 / 8.14;
    public final static double turningMotorGearRatio = 1.0 / (150.0 / 7.0);
    public final static double armGearRatio = 1/64;
    //controller settings
    public final static double XdriveSensitivity = 1;
    public final static double YdriveSensitivity = 1; 
    public final static double turningSensitivity = 1;
    public final static double maxSpeedMpS = 0.1; 
    public final static double maxSpeedRotation = 0.1; 


    public final static double controllerDeadzone = 0.1;

    //field oriented on/off
    public final static boolean fieldOriented = true;

    //functions
    public final static double clamp(double input, double min, double max) {
        return Math.max(Math.min(input, max), min);
    }




}
