package frc.robot.classes;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class TankDriveTrain extends DriveTrain{
    Motor[] motors;    

    public TankDriveTrain(Motor[] motors, double pidP, double pidI, double pidD) {
        super();        
        
    }

    public void GO(double xSpeed, double ySpeed, double rotSpeed){

        double clampedSpeed = globalFunctions.clamp(xSpeed);

        for (int i = 0; i < this.motors.length; i++) {
            this.motors[i].GO(clampedSpeed);
        }
    }

    public void setInverted(boolean value){ 
        for (int i = 0; i < this.motors.length; i++) {
            this.motors[i].setInverted(value);
        }
    }

    public double getAbsoluteEncoderPosition(MotorLocation location){ 
        Motor motor = getMotor(location);
        return motor.getAbsolutePosition();
    }

    public void resetEncoders(){ 
        for (int i = 0; i < this.motors.length; i++) {
            this.motors[i].resetSensorPos();
        }
    }
    
    public void setDriveEncoderPosition(double value){ 
        for (int i = 0; i < this.motors.length; i++) {
            this.motors[i].setSensorPos(value);
        }
    }

    public double getDriveEncoderPosition(MotorLocation location){ 
        //TODO
        return this.motors[0].getSensorPos();
    }

    public void setPIDs(double pidP, double pidI, double pidD){
        System.out.println("WARNING: Motor does not have pid");
        return;
    }

    public void resetPIDs(){ 
        System.out.println("WARNING: Motor does not have pid");
        return;
    }

    public Motor getMotor(MotorLocation location){         
        for (int i = 0; i < this.motors.length; i++) {
            if(this.motors[i].location == location) return this.motors[i];
        }
        System.out.println("WARNING - Motor Not Found in location: " + location);
        return null;
    }

    public SwerveModule getSwerveModule(MotorLocation location) {
        return null;
    }
    
    public void setMotorBrakes(){
        for (int i = 0; i < this.motors.length; i++) {
            this.motors[i].setBrake();
        }
    }
}
