package frc.robot.classes;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveDriveTrain extends DriveTrain{
    SwerveModule[] swerveModules;    
    SwerveDriveKinematics swerveKinematics;

    public SwerveDriveTrain(Motor[] motors, double pidP, double pidI, double pidD) {
        super(); 

        this.swerveModules = new SwerveModule[]{ 
            new SwerveModule(motors[0], motors[4], new PIDController(pidP, pidI, pidD)),
            new SwerveModule(motors[1], motors[5], new PIDController(pidP, pidI, pidD)),
            new SwerveModule(motors[2], motors[6], new PIDController(pidP, pidI, pidD)),
            new SwerveModule(motors[3], motors[7], new PIDController(pidP, pidI, pidD))
        };

        this.swerveKinematics = new SwerveDriveKinematics(
            newMain.motorLocations.get(MotorLocation.FrontLeft_Drive),
            newMain.motorLocations.get(MotorLocation.FrontRight_Drive),
            newMain.motorLocations.get(MotorLocation.BackLeft_Drive),
            newMain.motorLocations.get(MotorLocation.BackRight_Drive)
        );               
    }

    public void GO(double xSpeed, double ySpeed, double rotSpeed){
        double maxSpeedMpS = Config.maxSpeedMpS;

        ChassisSpeeds desiredSpeeds = new ChassisSpeeds(xSpeed * maxSpeedMpS, ySpeed * maxSpeedMpS, rotSpeed);

        // make desiredSpeeds into speeds and angles for each module
        SwerveModuleState[] moduleStates = this.swerveKinematics.toSwerveModuleStates(desiredSpeeds);

        for (int i = 0; i < this.swerveModules.length; i++) {
            this.swerveModules[i].GO(moduleStates[i]);
        }
    }

    public void setInverted(boolean value){ 
        for (int i = 0; i < this.swerveModules.length; i++) {
            this.swerveModules[i].driveMotor.setInverted(value);
            this.swerveModules[i].steerMotor.setInverted(value);
        }
    }

    public double getAbsoluteEncoderPosition(MotorLocation location){ 
        Motor motor = getMotor(location);
        return motor.getAbsolutePosition();
    }

    public void resetEncoders(){ 
        for (int i = 0; i < this.swerveModules.length; i++) {
            this.swerveModules[i].driveMotor.resetSensorPos();
            this.swerveModules[i].steerMotor.resetSensorPos();
        }
    }
    
    public void setDriveEncoderPosition(double value){ 
        for (int i = 0; i < this.swerveModules.length; i++) {
            this.swerveModules[i].driveMotor.setSensorPos(value);
        }
    }

    public double getDriveEncoderPosition(MotorLocation location){ 
        //TODO
        return this.swerveModules[0].driveMotor.getSensorPos();
    }

    public void setPIDs(double pidP, double pidI, double pidD){ 
        for (int i = 0; i < this.swerveModules.length; i++) {
            this.swerveModules[i].pid = new PIDController(pidP, pidI, pidD);
        }
    }

    public void resetPIDs(){ 
        //TODO singular/drive/steer PID functions
        for (int i = 0; i < this.swerveModules.length; i++) {
            this.swerveModules[i].pid.reset();
        }
    }
    
    public SwerveModule getSwerveModule(MotorLocation location){ 
        //TODO Explain the magic of Regex
        String[] splitLocation = location.toString().split("(?=[A-Z_])");
        
        SwerveModule found;
        if(splitLocation[0] == "Front"){
            if(splitLocation[0] == "Left"){
                return this.swerveModules[0];
            } else {
                return this.swerveModules[1];
            }
        } else {
            if(splitLocation[0] == "Left"){
                return this.swerveModules[2];
            } else {
                return this.swerveModules[3];
            }
        }
    }

    public Motor getMotor(MotorLocation location){ 
        //TODO Explain the magic of Regex
        String[] splitLocation = location.toString().split("(?=[A-Z_])");
        
        SwerveModule module = getSwerveModule(location);
        if(splitLocation[0] == "Drive"){
           return module.driveMotor;
        } else {
            return module.steerMotor;
        }
    }
    
    public void setMotorBrakes(){
        setDriveMotorBrakes();
        setSteerMotorBrakes();
    }

    public void setDriveMotorBrakes(){ 
        for (int i = 0; i < this.swerveModules.length; i++) {
            this.swerveModules[i].driveMotor.setBrake();
        }
    }
    public void setSteerMotorBrakes(){ 
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

    public void setContinouousInput() {
        for (int i = 0; i < this.swerveModules.length; i++) {
            this.swerveModules[i].setContinouousInput();
        }
    }



}
