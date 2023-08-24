package frc.robot.classes;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {
    public Motor driveMotor;
    public Motor steerMotor;

    PIDController pid;

    public SwerveModule(Motor driveMotor, Motor steerMotor, PIDController pid) {
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

    public void lockWheel(){
        //TODO future challenge:
        // calculate the angle for each individual wheel to turn to given the motorLocation and set each wheel to their respective lock value

        turnWheelToAngle(45);
    }
    public void straightenWheel(){ 
        turnWheelToAngle(0);
    }

    public void turnWheelToAngle(double targetAngle){
        double currentAngle = Math.abs(this.steerMotor.getAbsolutePosition());
        targetAngle = Math.abs(targetAngle);
        
        double tolerance = 0.0; //TODO move to config
        if(Math.abs(targetAngle - currentAngle) > tolerance){
            double turnPower = pid.calculate(currentAngle, targetAngle);
            steerMotor.GO(turnPower);
        }
    }

    public void resetPIDs(){
        pid.reset();
    }

    public void setContinouousInput(){
        pid.enableContinuousInput(-Math.PI, Math.PI);
    }   
}
