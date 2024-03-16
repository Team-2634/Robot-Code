package frc.robot;



public class Constants {
    //unchanging final stuff
    public final static double talonEncoder_TicksPerRev = 2048;
    public final static double neoEncoder_TicksPerRev = 42;

    public final static double maxDegree = 360;

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

    public final static String canivore = "rio";
    public final static int leftArmID = 8;
    public final static int rightArmID = 9;
    public final static int leftShootID = 11;
    public final static int rightShootID = 12;
    public final static int intakeID = 10;

    public final static int leftClimbID = 14;
    public final static int rightClimbID = 13;


    public final static int frontLeftAbsEncoderID = 0;
    public final static int frontRightAbsEncoderID = 1;
    public final static int backLeftAbsEncoderID = 2;
    public final static int backRightAbsEncoderID = 3;

    //PID values
    public final static double kpDrive = 0.3;
    public final static double kiDrive = 0;
    public final static double kdDrive = 0;

    public final static double kpArm = 0.3;
    public final static double kiArm = 0;
    public final static double kdArm = 0;    

    public final static double kpBotTranslation = 0.2;
    public final static double kiBotTranslation = 0;
    public final static double kdBotTranslation = 0;

    public final static double kpBotRotate = 0.1;
    public final static double kiBotRotate = 0;
    public final static double kdBotRotate = 0;

    public final static double kpLimelightAlign = 0.1;
    public final static double kiLimelightAlign = 0;
    public final static double kdLimelightAlign = 0;
    
    public final static double autoPositionToleranceMeters = 0.1;
    public final static double autoRotationToleranceRadians = 0.1;
    public final static double armPIDTolerance = 0.1;
    
    //auto
    public final static double maxAutoVelocity = 0.1;
    public final static double maxAutoAccel = 0.1;

    //robot properties
    public final static double frontLeftAbsEncoderOffset = 0.060;
    public final static double frontRightAbsEncoderOffset = -0.478;
    public final static double backLeftAbsEncoderOffset = -0.183;
    public final static double backRightAbsEncoderOffset = 0.134;

    public final static double wheelDiameterInches = 3.75;
    public final static double driveMotorGearRatio = 1 / 8.14;
    public final static double turningMotorGearRatio = 1.0 / (150.0 / 7.0);
    public final static double armGearRatio = (1.0 / 64.0) / 4;
    public final static double armOffset = 0;
    public final static double minClimb = -2;
    public final static double maxClimb = 250;
    public final static double minArmRotationRads = -1.40;
    public final static double maxArmRotationRads = 0.60;

    //arm speeds
    public final static double shootSpeed = 0.5;
    public final static double intakeSpeed = 0.5;
    public final static double armSpeed = 0.75;
    public final static double climbSpeed = 1;

    //set positions
    public final static double pickupPosition = -1.4;
    public final static double closeSpeakerPosition = -0.95;
    public final static double ampPosition = 0.4;

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
