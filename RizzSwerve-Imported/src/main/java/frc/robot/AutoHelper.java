package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.systems.Climber;
import frc.robot.systems.Driver;
import frc.robot.systems.Limelight;
import frc.robot.systems.Shooter;

public class AutoHelper {

    Driver driver;
    Shooter shooter;
    Climber climber;
    AHRS navx;
    // Timer timer;
    Limelight limelight;

    public AutoHelper(Driver driver, Shooter shooter, Climber climber, AHRS navx, Timer timer, Limelight limelight) {
        this.driver = driver;
        this.shooter = shooter;
        this.climber = climber;
        // this.navx = navx;
        this.timer = timer;
        this.limelight = limelight;
    }

    PIDController autoXPID = new PIDController(Constants.kpBotTranslation, Constants.kiBotTranslation, Constants.kdBotTranslation);
    PIDController autoYPID = new PIDController(Constants.kpBotTranslation, Constants.kiBotTranslation, Constants.kdBotTranslation);
    // ProfiledPIDController autoXPID = new ProfiledPIDController(Constants.kpBotTranslation, Constants.kiBotTranslation, Constants.kdBotTranslation, new TrapezoidProfile.Constraints(Constants.maxAutoVelocity, Constants.maxAutoAccel));
    // ProfiledPIDController autoYPID = new ProfiledPIDController(Constants.kpBotTranslation, Constants.kiBotTranslation, Constants.kdBotTranslation, new TrapezoidProfile.Constraints(Constants.maxAutoVelocity, Constants.maxAutoAccel));
    PIDController autoTurnPID = new PIDController(Constants.kpBotRotate, Constants.kiBotRotate, Constants.kdBotRotate);

    ColorSensorV3 noteSensor = new ColorSensorV3(I2C.Port.kMXP);

    void initialize() {
        autoXPID.setTolerance(Constants.autoPositionToleranceMeters);
        autoXPID.reset();//driver.getPose().getX());
        
        autoYPID.setTolerance(Constants.autoPositionToleranceMeters);
        autoYPID.reset();//driver.getPose().getY());

        autoTurnPID.enableContinuousInput(-Math.PI, Math.PI);
        autoTurnPID.setTolerance(Constants.autoRotationToleranceRadians);
        autoTurnPID.reset();
    }

    public boolean timerInterval_Auto(double min, double max) {
        if (timer.get() > min && timer.get() < max) {
            return true;
        } else {
            return false;
        }
    }

    /**
     * Drives to a position given meters from origin (starting position) and radians
     * @param endPose
     */
    public void driveToPosition(Pose2d endPose) {
        Pose2d startPose = driver.getPose();
        SmartDashboard.putNumber("inputX", Units.metersToInches(startPose.getX()));//Units.metersToFeet(startPose.getX()));
        SmartDashboard.putNumber("inputY", Units.metersToInches(startPose.getY()));//Units.metersToFeet(startPose.getY()));
        SmartDashboard.putNumber("inputRot", startPose.getRotation().getRadians());
        SmartDashboard.putNumber("outputX", Units.metersToInches(endPose.getX()));
        SmartDashboard.putNumber("outputY", Units.metersToInches(endPose.getY()));
        SmartDashboard.putNumber("outputRot", endPose.getRotation().getRadians());
        double xSpeed = autoXPID.calculate(startPose.getX(), endPose.getX());
        double ySpeed = autoYPID.calculate(startPose.getY(), endPose.getY()); 
        double rotSpeed = autoTurnPID.calculate(startPose.getRotation().getRadians(), endPose.getRotation().getRadians());
        SmartDashboard.putNumber("xSpeed", xSpeed);
        SmartDashboard.putNumber("ySpeed", ySpeed);
        SmartDashboard.putNumber("rotSpeed", rotSpeed);

        double[] fieldOriented = driver.fieldOrient(xSpeed, ySpeed);
        driver.swerveDrive(fieldOriented[0], fieldOriented[1], rotSpeed);
        
    }

    public boolean atTargetPosition() {
        boolean at = (autoXPID.atSetpoint() && autoYPID.atSetpoint() && autoTurnPID.atSetpoint()) ? true : false;
        if (at) {
            autoXPID.reset();//driver.getPose().getX());
            autoYPID.reset();//driver.getPose().getY());
            autoTurnPID.reset();
        }
        return at;
    }

    public void angleArmToPosition(double angle) {
        shooter.moveArmPID(angle);
    }

    public boolean armAtPosition() {
        return shooter.atPosition();
    }

    
    // public void shootNote() {
    //     shooter.shootNote(1, 1);
    // }

    public void intake(double input) {
        shooter.shootNote(-0.1);
        shooter.collectNote(input);
    }

    Timer timer = new Timer();
    
    public boolean hasNote() {
        return noteSensor.getRed() > 400;
    }

    public boolean delayFlag = true;
    public double delayStopTime = 0;
    public boolean delay(double delay) {
        if (delayFlag) {
            delayStopTime = timer.get() + delay;
            delayFlag = false;
        }
        if (timer.get() > delayStopTime) {
            delayFlag = true;
            return true;
        } else {
            return false;
        }
    }

    public void prepNote() {
        shooter.collectNote(-0.1);
    }

    // boolean shootFlag = true;
    // public void shoot(double input, double time) {
    //     double timeEnd = 0;
    //     if (shootFlag) {
    //         shootFlag = false;
    //         timeEnd = timer.get() + time;
    //     }
    //     if (timer.get() < timeEnd) {
    //         shooter.shootNote(input);
    //     } else {shootFlag = true;}
    // }

    // boolean intakeFlag;
    // public void intake(double input, double time) {
    //     double timeEnd = 0;
    //     if (intakeFlag) {
    //         intakeFlag = false;
    //         timeEnd = timer.get() + time;
    //     }
    //     if (timer.get() < timeEnd) {
    //         shooter.collectNote(input);
    //     } else {intakeFlag = true;}
    // }
    
    public void armToPosition(double position) {
        shooter.moveArmPID(position);
    }

    public Pose2d getPose() {
        return driver.getPose();
    }

    public Pose2d setDesiredPose(double x, double y, double rot) {
        return new Pose2d(x, y, new Rotation2d(rot));
    }

    public void stopDrive(){
        driver.swerveDrive(0, 0, 0);
    }

    public void stopIntake(){
        shooter.collectNote(0);
    }

    public void stopShoot(){
        shooter.shootNote(-0.1);
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
