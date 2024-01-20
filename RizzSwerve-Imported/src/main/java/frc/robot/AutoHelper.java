package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.systems.Climber;
import frc.robot.systems.Driver;
import frc.robot.systems.Shooter;

public class AutoHelper {

    Driver driver;
    Shooter shooter;
    Climber climber;
    AHRS navx;
    Timer timer;

    public AutoHelper(Driver driver, Shooter shooter, Climber climber, AHRS navx, Timer timer) {
        this.driver = driver;
        this.shooter = shooter;
        this.climber = climber;
        this.navx = navx;
        this.timer = timer;
    }

    PIDController autoXPID = new PIDController(Constants.kpAuto, Constants.kiAuto, Constants.kdAuto);
    PIDController autoYPID = new PIDController(Constants.kpAuto, Constants.kiAuto, Constants.kdAuto);


    public boolean timerInterval_Auto(double min, double max) {
        if (timer.get() > min && timer.get() < max) {
            return true;
        } else {
            return false;
        }
    }

    public void resetDriveEncoders() {
        driver.frontLeftDrive.setPosition(0);
        driver.frontRightDrive.setPosition(0);
        driver.backLeftDrive.setPosition(0);
        driver.backRightDrive.setPosition(0);
    }

    
    public void autoDriveToPosition(double distanceX, double distanceY) {
        double currentYawRadians = Math.toRadians(navx.getYaw());
        double fieldDistanceX = distanceX * Math.cos(currentYawRadians) - distanceY * Math.sin(currentYawRadians);
        double fieldDistanceY = distanceX * Math.sin(currentYawRadians) + distanceY * Math.cos(currentYawRadians);

        double currentDisplacementX = navx.getDisplacementX() * Math.cos(currentYawRadians) - navx.getDisplacementY() * Math.sin(currentYawRadians);
        double currentDisplacementY = navx.getDisplacementX() * Math.sin(currentYawRadians) + navx.getDisplacementY() * Math.cos(currentYawRadians);
        
        double xSpeed = Math.min(autoXPID.calculate(currentDisplacementX, fieldDistanceX), 1);
        double ySpeed = Math.min(autoXPID.calculate(currentDisplacementY, fieldDistanceY), 1);
        driver.swerveDrive(xSpeed, ySpeed, 0);
    }

    public void autoResetPIDs() {
        autoXPID.reset();
        autoYPID.reset();
    }

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

    public boolean driveSwerve_EncoderIf_turnOnSpot(double targetYaw_inRad) {
        double currentRoationYaw_inRad = Math.toRadians(navx.getYaw());
        double outPutRad;

        double tolerance = 0.2;
        double RotSpeed = 25; // rads per sec
        if (Math.abs(targetYaw_inRad - currentRoationYaw_inRad) > tolerance) {
            if (currentRoationYaw_inRad < targetYaw_inRad) {
                outPutRad = RotSpeed;
                driver.swerveDrive(0, 0, outPutRad);
                return false;
            } else if (currentRoationYaw_inRad > targetYaw_inRad) {
                outPutRad = -RotSpeed;
                driver.swerveDrive(0, 0, outPutRad);
                return false;
            }
            return false;
        } else {
            driver.swerveDrive(0, 0, 0);
            return true;
        }
    }


}
