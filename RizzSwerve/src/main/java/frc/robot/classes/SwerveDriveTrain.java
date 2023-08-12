public class SwerveDriveTrain {
    SwerveModule[] swerveModules;
    
    SwerveDriveKinematics swerveKinematics;

    public SwerveDriveTrain(Motor[] motors, double pidP, double pidI, double pidD) {
        this.swerveModules = { 
            new SwerveModule(motors[0], motors[1], new PIDController(pidP, pidI, pidD)),
            new SwerveModule(motors[2], motors[3], new PIDController(pidP, pidI, pidD)),
            new SwerveModule(motors[4], motors[5], new PIDController(pidP, pidI, pidD)),
            new SwerveModule(motors[6], motors[7], new PIDController(pidP, pidI, pidD))
        }

        this.swerveKinematics = new SwerveDriveKinematics(
            motorLocations.get(motorLocation.frontLeftDrive),
            motorLocations.get(motorLocation.frontRightDrive),
            motorLocations.get(motorLocation.backLeftDrive),
            motorLocations.get(motorLocation.backRightDrive)
        );

    }

    public void GO(double xSpeed, double ySpeed, double rotSpeed){
        double maxSpeedMpS = Config.maxSpeedMpS;

        ChassisSpeeds desiredSpeeds = new ChassisSpeeds(xSpeed * maxSpeedMpS, ySpeed * maxSpeedMpS, rotSpeed);

        // make desiredSpeeds into speeds and angles for each module
        SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(desiredSpeeds);

        for (int i = 0; i < this.swerveModules.length; i++) {
            this.swerveModules[i].GO(moduleStates[i]);
        }
    }

    public void setInverted(boolean inversion){ 
        //TODO 
    }

    public void resetEncoders(){ 
        //TODO 
    }
    
    public void setDriveSensorPosition(double value){ 
        for (int i = 0; i < this.swerveModules.length; i++) {
            this.swerveModules[i].driveMotor.setSensorPos(value);
        }
    }

    public void getDriveSensorPosition(MotorLocation location){ 
        //TODO
        this.swerveModules[0].driveMotor.getSelectedSensorPosition();
    }

    public void resetPIDs(){ 
        //TODO 
    }
    
    public void setMotorBreaks(){
        setDriveMotorBreaks();
        setSteerMotorBreaks();
    }

    public void setDriveMotorBreaks(){ 
        for (int i = 0; i < this.swerveModules.length; i++) {
            this.swerveModules[i].driveMotor.setBrake();
        }
    }
    public void setSteerMotorBreaks(){ 
        for (int i = 0; i < this.swerveModules.length; i++) {
            this.swerveModules[i].steerMotor.setBrake();
        }
    }

    public void straightenModules() {
        for (int i = 0; i < this.swerveModules.length; i++) {
            this.swerveModules[i].straightenWheel();
        }
    }

    public void lockWheels() {
        for (int i = 0; i < this.swerveModules.length; i++) {
            this.swerveModules[i].lockWheel();
        }
    }

}
