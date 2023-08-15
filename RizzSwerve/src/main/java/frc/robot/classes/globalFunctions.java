package frc.robot.classes;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//TODO break up into service files
public class globalFunctions {
    
    // MATH
    public static double clamp(double value){
        return clamp(value, 1f, 0f);
    }
    public static double clamp(double value, double max, double min){
        return Math.max(min, Math.min(max, value));
    }

    // Convertions
    public static double convertRotationToMeter(double gearRatio, double wheelDiameter){
        return gearRatio * Math.PI * wheelDiameter;
    }
    //For steering motor use diameter = 2 (???)
    public static double convertRotationToMeter(double gearRatio){
        return gearRatio * 2 * Math.PI;
    }
    public static double convertTicksToMPS(double gearRatio, double wheelDiameter, double motorTicksPerRev){
        //Convert Encoder ticks to Metres per second
        return convertRotationToMeter(gearRatio, wheelDiameter) / Constants.talonEncoder_TicksPerRev; //TODO use motorType enum
    }
    public static double convertTicksToMPS(double metresRotation, double motorTicksPerRev){
        //Convert Encoder ticks to Metres per second
        return metresRotation / motorTicksPerRev;
    }
    public static double convertTicksToMPS(double gearRatio, double wheelDiameter, String motorType){
        //Convert Encoder ticks to Metres per second

        double motorTicksPerRev = 0;

        if(motorType == "WPI_Talon") motorTicksPerRev = Constants.talonEncoder_TicksPerRev; //TODO use motorType enum

        return convertRotationToMeter(gearRatio, wheelDiameter) / motorTicksPerRev;
    }

    //What is this??
    public static double convertTicksToRads(double radians, double motorTicksPerRev){
        //Convert Encoder ticks to Metres per second
        return radians / motorTicksPerRev;

        //public final double kTurningEncoderTicksToRad = kTurningEncoderRot2Rad / talonEncoder_TicksPerRev;

    }


    // Logging
    public static void toDashboard(String title, Float value){
        SmartDashboard.putNumber(title, value);
    }
    public static void toDashboard(String title, Double value){
        SmartDashboard.putNumber(title, value);
    }
    public static void toDashboard(String title, Boolean value){
        SmartDashboard.putBoolean(title, value);
    }
    public static void toDashboard(Sendable value){
        SmartDashboard.putData(value);
    }
    
    // Helper //TODO move to controller class
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
