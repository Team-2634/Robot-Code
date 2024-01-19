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
    public final static double kpDrive = 0.3;
    public final static double kiDrive = 0.01;
    public final static double kdDrive = 0.01;

    public final static double kpAuto = 1;
    public final static double kiAuto = 0;
    public final static double kdAuto = 0.0000075;

    public final static double kWheelDiameterInches = 3.75;
    public final static double kDriveMotorGearRatio = 1 / 8.14;
    public final static double kTurningMotorGearRatio = 1.0 / (150.0 / 7.0);

    public final static double XdriveSensitivity = 1;
    public final static double YdriveSensitivity = 1; 
    public final static double turningSensitivity = 25;
    public final static double maxSpeedMpS = 20; 

    public final static double controllerDeadzone = 0.15;

    


    
}
