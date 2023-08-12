package rizzler;

public class globalFunctions {
    
    // MATH
    public static double clamp(value){
        return clamp(value, 1f, 0f);
    }
    public static double clamp(value, max, min){
        return Math.max(min, Math.min(max, value));
    }

    // Convertions //TODO
    public double convertRotationToMeter(double gearRatio, double wheelDiameter){
        return gearRatio * Math.PI * wheelDiameter;
    }
    //For steering motor use diameter = 2 (???)
    public double convertRotationToMeter(double gearRatio){
        return gearRatio * 2 * Math.PI;
    }
    public double convertTicksToMPS(double gearRatio, double wheelDiameter, double motorTicksPerRev){
        //Convert Encoder ticks to Metres per second
        return convertRotationToMeter(gearRatio, wheelDiameter) / talonEncoder_TicksPerRev
    }
    public double convertTicksToMPS(double metresRotation, double motorTicksPerRev){
        //Convert Encoder ticks to Metres per second
        return metresRotation / motorTicksPerRev;
    }
    public double convertTicksToMPS(double gearRatio, double wheelDiameter, String motorType){
        //Convert Encoder ticks to Metres per second

        double motorTicksPerRev = 0;

        if(motorType == "WPI_Talon") motorTicksPerRev = Constants.talonEncoder_TicksPerRev;

        return convertRotationToMeter(gearRatio, wheelDiameter) / motorTicksPerRev
    }

    //What is this??
    public double convertTicksToRads(double radians, double motorTicksPerRev){
        //Convert Encoder ticks to Metres per second
        return radians / motorTicksPerRev;

        //public final double kTurningEncoderTicksToRad = kTurningEncoderRot2Rad / talonEncoder_TicksPerRev;

    }


    // Logging
    public static void toDashboard(String title, Double value){
        SmartDashboard.putNumber(title, value);
    }
    public static void toDashboard(String title, Boolean value){
        SmartDashboard.putBoolean(title, value);
    }
    public static void toDashboard(Double value){
        SmartDashboard.putData(navx);
    }
    
    // Helper
    public static double removeDeadzone(XboxController controller, int axisInput) {
        if (Math.abs(controller.getRawAxis(axisInput)) < Config.controllerDeadzone) {
            return 0;
        }
        return controller.getRawAxis(axisInput);
    }

    //TODO make independant of and MotorType
    public static double getEncoderTicksMPS() {
        double driveEncoderMeter = convertRotationToMeter(Config.driveMotorGearRatio, Config.driveWheelDiameterMeters);
        return driveEncoderMeter / Constants.talonEncoder_TicksPerRev;
    }

    //TODO merge functions w/ param
    //TODO make independant of  and MotorType
    public static double getEncoderTicksMPS_steer() {
        double driveEncoderMeter = convertRotationToMeter(Config.steerMotorGearRatio, 2);
        return driveEncoderMeter / Constants.talonEncoder_TicksPerRev;
    }
    

}


public final class Constants {
    public static final double talonEncoder_TicksPerRev = 2048;
    public static final double neoEncoder_TicksPerRev = 42;

    public static double PI = Math.PI;
    public static double maxDegree = 360;


}

public final class Config {
    public static final double driveWheelDiameterMeters = Units.inchesToMeters(3.75);
    public static final double driveMotorGearRatio = 1 / 8.14;
    public static final double steerMotorGearRatio = 1.0 / (150.0 / 7.0);


    //TODO store better as tuple
    //TODO add link to PID tutorial
    public static final double autoSteerP = 1;
    public static final double autoSteerI = 0;
    public static final double autoSteerD = 0.0000075;
    
    public static final double teleopSteerP = 0.3;
    public static final double teleopSteerI = 0.01;
    public static final double teleopSteerD = 0.01;

    public static final double XdriveSensitivity = 1;
    public static final double YdriveSensitivity = 1; // do not change above 1
    public static final double turningSensitivity = 25; // radians
    
    public static final double maxSpeedMpS = 20; // metres/sec
    
    //Controller
    public static final double controllerDeadzone = 0.15; //Percent
}

enum MotorLocation {
    FrontLeft_Drive,
    FrontRight_Drive,
    BackLeft_Drive,
    BackRight_Drive,
    FrontLeft_Steer,
    FrontRight_Steer,
    BackLeft_Steer,
    BackRight_Steer
}


enum MotorType {
    WPI_TalonFX,
    CANSparkMax
}


