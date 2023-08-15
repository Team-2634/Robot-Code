/*
public final class Constants {
    public static final double talonEncoder_TicksPerRev = 2048;
    public static final double neoEncoder_TicksPerRev = 42;

    public static double PI = Math.PI;
}

public class Motors {


 
 
    public final WPI_TalonFX frontLeftDrive = new WPI_TalonFX(7);
    public final WPI_TalonFX frontRightDrive = new WPI_TalonFX(1);
    public final WPI_TalonFX backLeftDrive = new WPI_TalonFX(5);
    public final WPI_TalonFX backRightDrive = new WPI_TalonFX(3);



    public final Motor driveFrontLeft = new Motor();
    public final Motor driveFrontRight = new Motor();
    public final Motor driveBackLeft = new Motor();
    public final Motor driveBackRight = new Motor();


}

public class Run{
    //Motors DriveMotors = new Motors();

    SwerveModule m1 = new SwerveModule();

}



public class Motor {
    private final gearRatio;

    public Motor(motorId, motorType, wheelDiameter, gearRatio){
        //gearRatio = 1 / 8.14;
        gearRatio = gearRatio;
    }
}


public class PidMotor {
    public PIDController pid;
    public Motor motor;

    public void PidMotor(kp,ki,kd){
        pid = new PIDController(kp, ki, kd);
    }


}

public class SwerveModule {
    public Motor steerMotor;
    public Motor driveMotor;

    public final double kTurningEncoderTicksToRad;

    public SwerveModule(){


        kTurningEncoderTicksToRad = kTurningEncoderRot2Rad / steerMotor.getTicksPerRev
        //talonEncoder_TicksPerRev
    }

    public double getSensorPos(){
        return steerMotor.getSelectedSensorPosition() *kTurningEncoderTicksToRad;
    }
}
*/