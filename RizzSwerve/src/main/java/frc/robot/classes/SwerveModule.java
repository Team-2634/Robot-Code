public class SwerveModule {
    public Motor driveMotor;
    public Motor steerMotor;

    PID pid;

    public SwerveModule(Motor driveMotor, Motor steerMotor, PID pid) {
        this.driveMotor = driveMotor;
        this.steerMotor = steerMotor;
        this.pid = pid;
    }

    public void GO(SwerveModuleState moduleState){
        double sensorPos = steerMotor.getSensorPos();
        var currentAngle = new Rotation2d(sensorPos);
        var optimizedAngle = SwerveModuleState.optimize(moduleState, currentAngle);

        // set steer motor power to pid output given current position and desired position in radians
        double turnPower = pid.calculate(sensorPos, optimizedAngle.angle.getRadians());
        
        // positive is clockwise (right side up)
        steerMotor.GO(turnPower);

        // set drive power to desired speed div max speed to get value between 0 and 1
        driveMotor.GO(optimizedAngle.speedMetersPerSecond / Config.maxSpeedMpS);
    }

    public void resetPIDs(){
        pid.reset();
    }

    public void setContinouousInput(){
        pid.enableContinuousInput(-Math.PI, Math.PI);
    }
    
    public void straightenWheel(){ 
        double angle = Math.abs(this.steerMotor.getAbsolutePosition());

        if(angle > 0.0){
            double turnPower = pid.calculate(angle, 0);
            frontLeftSteer.GO(turnPower);
        }
    }
}
