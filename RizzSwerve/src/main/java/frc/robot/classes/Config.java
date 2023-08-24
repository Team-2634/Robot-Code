package frc.robot.classes;

import edu.wpi.first.math.util.Units;

public final class Config {
    public static final double driveWheelDiameterMeters = Units.inchesToMeters(3.75);
    public static final double driveMotorGearRatio = 1 / 8.14;
    public static final double steerMotorGearRatio = 1.0 / (150.0 / 7.0);

    // TODO add link to PID tutorial
    public static final double autoSteer_P = 1;
    public static final double autoSteer_I = 0;
    public static final double autoSteer_D = 0.0000075;
    // public static List<Number> autoPID = Arrays.asList(1, 0, 0.0000075);

    public static final double teleopSteer_P = 0.3;
    public static final double teleopSteer_I = 0.01;
    public static final double teleopSteer_D = 0.01;
    // public static List<Number> teleopPID = Arrays.asList(0.3, 0.01, 0.01);

    // TODO not in use
    public static final double armAngle_P = 0.5;
    public static final double armAngle_I = 0.05;
    public static final double armAngle_D = 0.05;

    // TODO not in use
    public static final double armExtend_P = 0.5;
    public static final double armExtend_I = 0;
    public static final double armExtend_D = 0;

    // TODO not in use
    public static final double navxPitch_P = 0.01;
    public static final double navxPitch_I = 0.0;
    public static final double navxPitch_D = 0.0;

    public static final double maxSpeedMpS = 20; // metres/sec

    // TODO rename all controllerXSensitivity...
    // Controller
    public static final double controllerDeadzone = 0.15; // Percentage
    public static final double XdriveSensitivity = 1; // Percentage
    public static final double YdriveSensitivity = 1; // Percentage
    public static final double turningSensitivity = 25; // Radians
    public static final boolean useFieldOrientation = true;
}