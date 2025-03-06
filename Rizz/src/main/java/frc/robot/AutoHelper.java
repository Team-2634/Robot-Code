package frc.robot;

//import com.kauailabs.navx.frc.AHRS;
import com.studica.frc.AHRS;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.systems.Driver;
import frc.robot.systems.Elevator;
import edu.wpi.first.math.*;


public class AutoHelper {

    Driver driver;
    AHRS navx;
    Timer timer;
    Elevator elevator;

    public AutoHelper(Driver driver, Timer timer, Elevator elevator) {
        this.driver = driver;
        this.timer = timer;
        this.elevator = elevator;
    }

    PIDController autoXPID = new PIDController(Constants.kpAuto, Constants.kiAuto, Constants.kdAuto);
    PIDController autoYPID = new PIDController(Constants.kpAuto, Constants.kiAuto, Constants.kdAuto);
    PIDController autoTurnPID = new PIDController(Constants.kpAutoRotate, Constants.kiAutoRotate, Constants.kdAutoRotate);


    public boolean timerInterval_Auto(double min, double max) {
        if (timer.get() > min && timer.get() < max) {
            return true;
        } else {
            return false;
        }
    }

    void initialize(){
        autoXPID.setTolerance(Constants.autoPositionToleranceMeters);
        autoXPID.reset();

        autoYPID.setTolerance(Constants.autoPositionToleranceMeters);
        autoYPID.reset();

        autoTurnPID.enableContinuousInput(-Math.PI, Math.PI);
        autoTurnPID.setTolerance(Constants.autoRotationToleranceRadians);
        autoTurnPID.reset();
    }

    public void resetDriveEncoders() {
        driver.frontLeftDrive.setPosition(0);
        driver.frontRightDrive.setPosition(0);
        driver.backLeftDrive.setPosition(0);
        driver.backRightDrive.setPosition(0);
    }

    public void resetSteerEncoders(){
        driver.frontLeftSteer.setPosition(0);
        driver.frontRightSteer.setPosition(0);
        driver.backLeftSteer.setPosition(0);
        driver.backRightSteer.setPosition(0);
    }

    public void autoDriveByDistance(double distanceX, double distanceY) {
        double[] distanceFieldOriented = Driver.fieldOrient(distanceX, distanceY, navx);
        double fieldDistanceX = distanceFieldOriented[0];
        double fieldDistanceY = distanceFieldOriented[1]; 

        double[] displacementFieldOriented = Driver.fieldOrient(navx.getDisplacementX(), navx.getDisplacementY(), navx);
        double currentDisplacementX = displacementFieldOriented[0];
        double currentDisplacementY = displacementFieldOriented[1]; 
        
        double xSpeed = autoXPID.calculate(currentDisplacementX, fieldDistanceX);
        double ySpeed = autoYPID.calculate(currentDisplacementY, fieldDistanceY);
        driver.swerveDrive(xSpeed, ySpeed, 0);
    }

    public void autoResetPIDs() {
        autoXPID.reset();
        autoYPID.reset();
    }

    // public void driveToPosition(Pose2d endPose){
    //     Pose2d startPose = driver.getPose();

    //     double xSpeed = autoXPID.calculate(startPose.getX(), endPose.getX());
    //     double ySpeed = autoYPID.calculate(startPose.getX(), endPose.getY());
    //     double rotSpeed = autoTurnPID.calculate(MathUtil.angleModulus(startPose.getRotation().getRadians()), MathUtil.angleModulus(endPose.getRotation().getRadians()));
    // }

    /**
     * @deprecated
     * 
     * @param targetYaw_inRad
     * @return
     */
    public boolean driveSwerve_EncoderIf_FwdAndBwd(double targetX) {
        targetX = -targetX;

        double currentDistanceX = driver.readDriveEncoder(0) * driver.ticksToMetersDrive;
        double outPutX = 0;

        double toleranc = 0.05;
        double xSpeed = 0.45;
        if (Math.abs(targetX - currentDistanceX) > toleranc) {
            if (currentDistanceX < targetX) {
                outPutX = xSpeed;
                driver.swerveDrive(outPutX, 0, 0);
                return false;
            }
            if (currentDistanceX > targetX) {
                outPutX = -xSpeed;
                driver.swerveDrive(outPutX, 0, 0);
                return false;
            }
            return false;
        } else {
            driver.swerveDrive(0, 0, 0);
            return true;
        }
    }

    /*
     * clean up
    */
    public boolean autoDriveRotate(double targetYawRadians) {
        double currentYawRadians = Math.toRadians(navx.getYaw());

        double tolerance = 0.2;
        double RotSpeed = 25; // rads per sec
        if (Math.abs(targetYawRadians - currentYawRadians) > tolerance) {
            if (currentYawRadians < targetYawRadians) {
                driver.swerveDrive(0, 0, RotSpeed);
                return false;
            } else if (currentYawRadians > targetYawRadians) {
                driver.swerveDrive(0, 0, -RotSpeed);
                return false;
            }
            return false;
        } else {
            driver.swerveDrive(0, 0, 0);
            return true;
        }
    }

    public void autoDriveRotatePID(double targetYawRadians) {
        driver.swerveDrive(0, 0, autoTurnPID.calculate(targetYawRadians, Math.toRadians(navx.getYaw())));
    }
    
}
