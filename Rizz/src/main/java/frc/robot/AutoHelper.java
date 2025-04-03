package frc.robot;

//import com.kauailabs.navx.frc.AHRS;
import com.studica.frc.AHRS;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.systems.Climber;
import frc.robot.systems.Driver;
import frc.robot.systems.Elevator;
import frc.robot.systems.Arm;
import frc.robot.systems.LimeLight;

public class AutoHelper {

    Driver driver;
    Climber climber;
    AHRS navx;
    Timer timer;
    Elevator elevator;
    Arm arm;
    LimeLight limelight;

    public AutoHelper(Driver driver, Climber climber, AHRS navx, Timer timer, Elevator elevator, Arm arm, LimeLight limelight) {
        this.driver = driver;
        this.climber = climber;
        this.navx = navx;
        this.timer = timer;
        this.elevator = elevator;
        this.arm = arm;
        this.limelight = limelight;
    }

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

    public void resetSteerEncoders(){
        driver.frontLeftSteer.setPosition(0);
        driver.frontRightSteer.setPosition(0);
        driver.backLeftSteer.setPosition(0);
        driver.backRightSteer.setPosition(0);
    }


    public void autoResetPIDs() {
        driver.resetAutoPIDs();
    }

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

    public void autoElevatorLift(double speed){
       elevator.elevatorLift(speed);
        // elevator.moveToHeight(Constants.L1_HEIGHT);
    }

    public void autoArmLift(double speed){
       // arm.armAngle(35);
        arm.armMotor.set(speed);
    }

    public void autoOpenClaw(){
        arm.openClaw();
    }

    public void autoCloseClaw(){
        arm.closeClaw();
    }

    public void autoLimelightAlign(){

    }


    public void thirdReefPos() {
        if (!(arm.getArmAngleRad() < Constants.armL4 + 0.3 && arm.getArmAngleRad() > Constants.armL4 - 0.3)) {
            arm.moveArmPID(Constants.armL4);
            lastSetPositionArm = arm.getArmAngleRad();
            ArmPositionPresets = true;
        } else if (!(elevator.getElevatorHeight() < Constants.L4_HEIGHT + 0.0002 && elevator.getElevatorHeight() > Constants.L4_HEIGHT - 0.0002)) {
            elevator.elevatorPIDLift(Constants.L4_HEIGHT);
            lastSetPositionElevator = elevator.getElevatorHeight();
            elevatorPositionPresets = true;
        } else {
            return;
        }
    }

    double speed = 0;
    double lastSetPositionArm = 0;
    double lastSetPositionElevator = 0;
    boolean ArmPositionPresets = false;
    boolean elevatorPositionPresets = false;
    
}
