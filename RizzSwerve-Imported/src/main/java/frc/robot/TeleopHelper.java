package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import java.lang.Math;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.systems.Climber;
import frc.robot.systems.Driver;
import frc.robot.systems.Limelight;
import frc.robot.systems.LimelightHelpers;
import frc.robot.systems.Shooter;

public class TeleopHelper {
    
    Driver driver;
    Shooter shooter;
    Climber climber;
    AHRS navx;
    Limelight limelight;

    public TeleopHelper(Driver driver, Shooter shooter, Climber climber, AHRS navx, Limelight limelight) {
        this.driver = driver;
        this.shooter = shooter;
        this.climber = climber;
        this.navx = navx;
        this.limelight = limelight;
    }

    final XboxController xbox = new XboxController(0);

    public void drive(double XSpeed, double YSpeed, double TurnSpeed, boolean boost, boolean goLimelight, boolean disableFieldOrient) {
        
        if (goLimelight) {
            TurnSpeed = limelightRotate();
        }

        if (boost) {
            XSpeed /= 3;
            YSpeed /= 3;
            TurnSpeed /= 3;
        }

        if (!disableFieldOrient) {
            double[] speedsFieldOriented = driver.fieldOrient(XSpeed, YSpeed);
            XSpeed = speedsFieldOriented[0];
            YSpeed = speedsFieldOriented[1];
        
        }
        XSpeed *= Constants.XdriveSensitivity;
        YSpeed *= Constants.YdriveSensitivity;
        TurnSpeed *= Constants.turningSensitivity;
        
        driver.swerveDrive(XSpeed, YSpeed, TurnSpeed);
    }

    PIDController rotatePID = new PIDController(Constants.kpBotRotate, Constants.kiBotRotate, Constants.kdBotRotate);
    
    public double limelightRotate() {
        return rotatePID.calculate(limelight.tx, 0);
    }

    public double calculateArmAngle(){

        final double floorToSpeaker = Constants.targetToSpeaker + Constants.floorToTarget;
        final double armToTag = Constants.floorToTarget - Constants.floorToLimelight;

        double limelightToTargetAngle = (Constants.limelightAngle + limelight.ty) * Constants.degToRad;
        
        double botToSpeakerVertical = floorToSpeaker - Constants.floorToLimelight;
        double botToSpeakerHorizontal = armToTag / Math.tan(limelightToTargetAngle);

        double limelightToSpeakerAngle = Math.atan(botToSpeakerVertical / botToSpeakerHorizontal);
        double limelightToSpeakerLength = Math.sqrt(Math.pow(botToSpeakerHorizontal, 2) + Math.pow(botToSpeakerVertical, 2));

        //Angle between limelightToSpeaker and shooterToSpeaker
        double angleThree = Math.asin((Constants.armLength * Math.sin(Constants.shooterToSpeakerAngle)) / limelightToSpeakerLength);

        //Angle between limelightToSpeaker and shooterToSpeaker
        double angleFour = Math.PI - angleThree - Constants.shooterToSpeakerAngle; 

        //Opposite of angleFour
        double angleArm = Math.PI - limelightToSpeakerAngle - angleFour;

        //Account for arm angle offset
        return angleArm - Constants.shooterToSpeakerAngle;
    }

    /*
    * subwoofer Depth = 36 
    * botToLimelight =  12
    * 
    * wallToLimelight = 48 inch
    */
    


    public void shoot(double input) {
        shooter.shootNote(input);
        // if (input) {
        //     shooter.shootNote(Constants.shootSpeed);
        // } else {
        //     shooter.shootNote(0);
        // }
    } 

    public void shootRoutine(boolean hold) {
        if (hold) {
            shooter.shootNoteRoutine();
        }
    }

    int currentState = 0;
    public void setArmState(boolean stateUp, boolean stateDown) {
        if (stateUp && currentState < 2) {
            currentState++;
        }

        if (stateDown && currentState > 0) {
            currentState--;
        }

        switch (currentState) {
            case 0:
                shooter.moveArmPID(Constants.pickupPosition);
                break;
            case 1:
                shooter.moveArmPID(Constants.closeSpeakerPosition);
                break;
            case 2:
                shooter.moveArmPID(Constants.ampPosition);
                break;

            default:
                currentState = 0;
                break;
        
        }
    }

    public void intake(double input, boolean yButton) {
        
         if (yButton) {
            shooter.collectNote(-0.2);
            shooter.shootNote(-0.2);
        } else {
            shooter.collectNote(input/3);
        }
    }


    public void arm(boolean up, boolean down) {
        if (up && !shooter.isHardStoppedHigh()) {
            shooter.moveArm(Constants.armSpeed);
        } else if (down && !shooter.isHardStoppedLow()) {
            shooter.moveArm(-Constants.armSpeed);
        } else {
            shooter.moveArm(0);
        }
    }

    public void climb(boolean aButton, boolean bButton) {
        if (aButton && !climber.isHardStoppedHigh()) {
            climber.climb(Constants.climbSpeed);
        } else if (bButton && !climber.isHardStoppedLow()) {
            climber.climb(-Constants.climbSpeed);
        } else {
            climber.climb(0);
        }
    }

    public void panic(boolean AAAA) {
        if (AAAA) {
            SmartDashboard.putBoolean("PANIC", AAAA);
            driver.panicReset();
        }
    }

    public void limelightArmAngle(boolean xButton){
        limelight.updateLimelight();

        if(xButton){
            LimelightHelpers.setLEDMode_ForceOn("");
            if(limelight.tv){
                shooter.moveArmPID(calculateArmAngle());
            }
        } else {
            LimelightHelpers.setLEDMode_ForceOff("");
        }

    }

    // public boolean detectTarget(boolean xButton){
    //     return limelight.tv;
    // }

    /**
     * Get axis and remove deadzone from controller input
     * @param axisInput axis ID
     * @return
     */
    
}
