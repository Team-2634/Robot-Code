package frc.robot;

public class Constants {
    //unchanging final stuff
    public final static double talonEncoder_TicksPerRev = 2048;
    public final static double neoEncoder_TicksPerRev = 42;

    public final static double frontLeftAbsEncoderOffset = -0.438232421875;
    public final static double frontRightAbsEncoderOffset = 0.2087402343755;
    public final static double backLeftAbsEncoderOffset = -0.403564453125;
    public final static double backRightAbsEncoderOffset = -0.02294921875;

    public final static double maxDegree = 360;

    public final static double maxVelocity_MetersPerSeconds = 6;
    public final static double maxAccel_MetersPerSecondsSquared = 3;

    //configurable stuff

    //device IDs
    /*public final static int frontLeftDriveID = 1;
    public final static int frontRightDriveID = 3;
    public final static int backLeftDriveID = 5;
    public final static int backRightDriveID = 7;

    public final static int frontLeftSteerID = 0;
    public final static int frontRightSteerID = 2;
    public final static int backLeftSteerID = 4;
    public final static int backRightSteerID = 6;
    public final static int backRightSteerID = 6; */

    //switched back so bot can run properly

    public final static int frontLeftDriveID = 3;
    public final static int frontRightDriveID = 5;
    public final static int backLeftDriveID = 1;
    public final static int backRightDriveID = 7;

    public final static int frontLeftSteerID = 2;
    public final static int frontRightSteerID = 4;
    public final static int backLeftSteerID = 0;
    public final static int backRightSteerID = 6;

    public final static int armMotorID = 8;  //Put Device ID Later On "8 is not the right one"
    public final static int elevatorMotorID = 9; 
    
    
    // public final static int frontLeftAbsEncoderID = 3;
    // public final static int frontRightAbsEncoderID = 0;
    // public final static int backLeftAbsEncoderID = 2;
    // public final static int backRightAbsEncoderID = 1;

    //fixed encoder id's

    public final static int frontLeftAbsEncoderID = 1;
    public final static int frontRightAbsEncoderID = 2;
    public final static int backLeftAbsEncoderID = 0;
    public final static int backRightAbsEncoderID = 3;

    //PID values
    public final static double kpDrive = 0.3;
    public final static double kiDrive = 0;
    public final static double kdDrive = 0;

    public final static double kpAuto = 1;
    public final static double kiAuto = 0;
    public final static double kdAuto = 0.0000075;

    public final static double kpAutoRotate = 1;
    public final static double kiAutoRotate = 0;
    public final static double kdAutoRotate = 0.0000075;

    //drivetrain properties
    public final static double kWheelDiameterInches = 3.75;
    public final static double kDriveMotorGearRatio = 1 / 8.14;
    public final static double kTurningMotorGearRatio = 1.0 / (150.0 / 7.0);
    public final static double driveRotsToMeter = 0.055555555; //0.073286;

    //controller settings
    public final static double XdriveSensitivity = 1;
    public final static double YdriveSensitivity = 1; 
    public final static double turningSensitivity = 1;
    public final static double maxSpeedMpS = 0.5; //robot speed

    public final static double controllerDeadzone = 0.1;

    //field oriented on/off
    public final static boolean fieldOriented = true;

    //functions
    public final static double clamp(double input, double min, double max) {
        return Math.max(Math.min(input, max), min);
    }


    // arm & elevator properties

    public static final double ENCODER_TICKS_PER_REV = 2048;
    public static final double ARM_GEAR_RATIO = 2.0; // If arm has a gearbox, adjust this
    public static final double DEGREES_PER_REV = 360.0;        

    public static final double PULLEY_DIAMETER_METERS = 0.05; // Measure this
    public static final double PULLEY_CIRCUMFERENCE = Math.PI * PULLEY_DIAMETER_METERS;

    public static final double L1_HEIGHT = 0.2;  // Values needs to be measured (in meters)
    public static final double L2_HEIGHT = 0.5;
    public static final double L3_HEIGHT = 1.0;
    public static final double L4_HEIGHT = 1.5;

}