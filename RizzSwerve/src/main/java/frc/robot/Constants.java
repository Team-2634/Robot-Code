package frc.robot;

public static class Constants {
    //unchanging final stuff
    final double talonEncoder_TicksPerRev = 2048;
    final double neoEncoder_TicksPerRev = 42;

    final public double frontLeftAbsEncoderOffset = 197.19;
    final public double frontRightAbsEncoderOffset = 31.8;
    final public double backLeftAbsEncoderOffset = 285.205;
    final public double backRightAbsEncoderOffset = 50.4;

    


    //configurable stuff
    final double kpDrive = 0.3;
    final double kiDrive = 0.01;
    final double kdDrive = 0.01;

    final double kpAuto = 1;
    final double kiAuto = 0;
    final double kdAuto = 0.0000075;

    public final double kWheelDiameterInches = 3.75;
    public final double kDriveMotorGearRatio = 1 / 8.14;
    public final double kTurningMotorGearRatio = 1.0 / (150.0 / 7.0);

}
