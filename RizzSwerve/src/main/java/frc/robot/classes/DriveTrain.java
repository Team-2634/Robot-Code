package frc.robot.classes;

public abstract class DriveTrain {

    public DriveTrain() {}

    public abstract void GO(double xSpeed, double ySpeed, double rotSpeed);
    public abstract void setMotorBrakes();

    public abstract void setInverted(boolean value);
    public abstract Motor getMotor(MotorLocation location);

    //TODO These are swerve specific
    public abstract SwerveModule getSwerveModule(MotorLocation location);

    //TODO These are motor specific
    public abstract void resetEncoders();    
    public abstract double getAbsoluteEncoderPosition(MotorLocation location);
    public abstract double getDriveEncoderPosition(MotorLocation location);
    public abstract void setDriveEncoderPosition(double value);
    
    //TODO These are motor specific
    public abstract void setPIDs(double pidP, double pidI, double pidD);
    public abstract void resetPIDs();

}
