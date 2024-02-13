package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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

    ProfiledPIDController autoXPID = new ProfiledPIDController(Constants.kpAuto, Constants.kiAuto, Constants.kdAuto, new TrapezoidProfile.Constraints(Constants.maxAutoVelocity, Constants.maxAutoAccel));
    ProfiledPIDController autoYPID = new ProfiledPIDController(Constants.kpAuto, Constants.kiAuto, Constants.kdAuto, new TrapezoidProfile.Constraints(Constants.maxAutoVelocity, Constants.maxAutoAccel));
    PIDController autoTurnPID = new PIDController(Constants.kpAutoRotate, Constants.kiAutoRotate, Constants.kdAutoRotate);

    void initialize() {
        autoXPID.setTolerance(Constants.autoPositionToleranceMeters);
        autoXPID.reset(driver.getPose().getX());
        
        autoYPID.setTolerance(Constants.autoPositionToleranceMeters);
        autoYPID.reset(driver.getPose().getY());

        autoTurnPID.setTolerance(Constants.autoRotationToleranceRadians);
        autoTurnPID.reset();
        autoTurnPID.enableContinuousInput(-Math.PI, Math.PI);
    }

    public boolean timerInterval_Auto(double min, double max) {
        if (timer.get() > min && timer.get() < max) {
            return true;
        } else {
            return false;
        }
    }

    public boolean driveToPosition(Pose2d endPose) {
        Pose2d startPose = driver.getPose();
            double xSpeed = autoXPID.calculate(startPose.getX(), endPose.getX());
            double ySpeed = autoYPID.calculate(startPose.getY(), endPose.getY()); 
            double rotSpeed = autoTurnPID.calculate(startPose.getRotation().getRadians(), endPose.getRotation().getRadians());
        
        driver.swerveDrive(xSpeed, ySpeed, rotSpeed);

        if (autoXPID.atSetpoint() &&
            autoYPID.atSetpoint() &&
            autoTurnPID.atSetpoint() 
        ) {
            return true;
        } else {
            return false;
        }
    }

    public boolean shootNote() {
        shooter.shootNote(1, 1);

        if (true) {
            return true;
        } else {
            return false;
        }
    }

    public boolean intake() {
        shooter.collectNote(1);

        if (true) {
            return true;
        } else {
            return false;
        }
    }
    

    public Pose2d getPose() {
        return driver.getPose();
    }

    public Pose2d setDesiredPose(double x, double y, double rot) {
        return new Pose2d(x, y, new Rotation2d(rot));
    }

    public void stop(){
        driver.swerveDrive(0, 0, 0);
    }

        // public void resetDriveEncoders() {
    //     driver.frontLeftDrive.setPosition(0);
    //     driver.frontRightDrive.setPosition(0);
    //     driver.backLeftDrive.setPosition(0);
    //     driver.backRightDrive.setPosition(0);
    // }

    // public void autoDriveByDistance(double distanceX, double distanceY) {
    //     double[] distanceFieldOriented = driver.fieldOrient(distanceX, distanceY);
    //     double fieldDistanceX = distanceFieldOriented[0];
    //     double fieldDistanceY = distanceFieldOriented[1];

    //     double[] displacementFieldOriented = driver.fieldOrient(navx.getDisplacementX(), navx.getDisplacementY());
    //     double currentDisplacementX = displacementFieldOriented[0];
    //     double currentDisplacementY = displacementFieldOriented[1];
        
    //     double xSpeed = autoXPID.calculate(currentDisplacementX, fieldDistanceX);
    //     double ySpeed = autoYPID.calculate(currentDisplacementY, fieldDistanceY);
    //     driver.swerveDrive(xSpeed, ySpeed, 0);
    // }

    /**
     * @deprecated
     * 
     * @param targetYaw_inRad
     * @return
     */
    // public boolean driveSwerve_EncoderIf_FwdAndBwd(double targetX) {
    //     targetX = -targetX;

    //     double currentDistanceX = driver.readDriveEncoder(0) * driver.ticksToMetersDrive;
    //     double outPutX = 0;

    //     double toleranc = 0.05;
    //     double xSpeed = 0.45;
    //     if (Math.abs(targetX - currentDistanceX) > toleranc) {
    //         if (currentDistanceX < targetX) {
    //             outPutX = xSpeed;
    //             driver.swerveDrive(outPutX, 0, 0);
    //             return false;
    //         }
    //         if (currentDistanceX > targetX) {
    //             outPutX = -xSpeed;
    //             driver.swerveDrive(outPutX, 0, 0);
    //             return false;
    //         }
    //         return false;
    //     } else {
    //         driver.swerveDrive(0, 0, 0);
    //         return true;
    //     }
    // }

    /*
     * clean up
    */
    // public boolean autoDriveRotate(double targetYawRadians) {
    //     double currentYawRadians = Math.toRadians(navx.getYaw());

    //     double tolerance = 0.2;
    //     double RotSpeed = 25; // rads per sec
    //     if (Math.abs(targetYawRadians - currentYawRadians) > tolerance) {
    //         if (currentYawRadians < targetYawRadians) {
    //             driver.swerveDrive(0, 0, RotSpeed);
    //             return false;
    //         } else if (currentYawRadians > targetYawRadians) {
    //             driver.swerveDrive(0, 0, -RotSpeed);
    //             return false;
    //         }
    //         return false;
    //     } else {
    //         driver.swerveDrive(0, 0, 0);
    //         return true;
    //     }
    // }

    // public void autoDriveRotatePID(double targetYawRadians) {
    //     driver.swerveDrive(0, 0, autoTurnPID.calculate(targetYawRadians, Math.toRadians(navx.getYaw())));
    // }
    
}
