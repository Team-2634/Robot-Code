package frc.robot;

import frc.robot.systems.Climber;
import frc.robot.systems.Driver;
import frc.robot.systems.Shooter;

public class AutoHelper {

    Driver driver;
    Shooter shooter;
    Climber climber;

    public AutoHelper(Driver driver, Shooter shooter, Climber climber) {
        this.driver = driver;
        this.shooter = shooter;
        this.climber = climber;
    }

    public boolean timerInterval_Auto(double min, double max) {
        if (timerAuto.get() > min && timerAuto.get() < max) {
            return true;
        } else {
            return false;
        }
    }

    public boolean driveSwerve_EncoderIf_FwdAndBwd(double targetX) {
        targetX = -targetX;

        double currentDistanceX;
        currentDistanceX = encoderLeftFrontDriveDisplacement_Meteres;
        double outPutX = 0;

        double toleranc = 0.05;
        double xSpeed = 0.45;
        double xSpeed_Rev = -0.30;
        if (Math.abs(targetX - currentDistanceX) > toleranc) {
            if (currentDistanceX < targetX) {
                outPutX = xSpeed;
                swerveDrive(outPutX, 0, 0);
                System.out.println("behind target");
                return false;
            }
            if (currentDistanceX > targetX) {
                outPutX = -xSpeed;
                swerveDrive(outPutX, 0, 0);
                System.out.println("in front of target");
                return false;
            }
            return false;
        } else {
            swerveDrive(0, 0, 0);
            return true;
        }
    }

    public boolean driveSwerve_EncoderIf_turnOnSpot(double targetYaw_inRad) {
        double currentRoationYaw_inRad;
        currentRoationYaw_inRad = botYaw_angleRad;
        double outPutRad = 0;

        double tolerance = 0.2;
        double RotSpeed = 25; // rads per sec
        if (Math.abs(targetYaw_inRad - currentRoationYaw_inRad) > tolerance) {
            if (currentRoationYaw_inRad < targetYaw_inRad) {
                outPutRad = RotSpeed;
                swerveDrive(0, 0, outPutRad);
                return false;
            } else if (currentRoationYaw_inRad > targetYaw_inRad) {
                outPutRad = -RotSpeed;
                swerveDrive(0, 0, outPutRad);
                return false;
            }
            return false;
        } else {
            swerveDrive(0, 0, 0);
            return true;
        }
    }


}
