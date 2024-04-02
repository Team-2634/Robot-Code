package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.ColorSensorV3;

import java.lang.Math;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.systems.Climber;
import frc.robot.systems.Driver;
import frc.robot.systems.Limelight;
import frc.robot.systems.LimelightHelpers;
import frc.robot.systems.LimelightHelpers.LimelightTarget_Fiducial;
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
        
        // if (goLimelight) {
        //     limelight.updateLimelight();
        //     TurnSpeed = limelightRotate();
        // }

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
        SmartDashboard.putNumber("limelightToTargetAngle", Constants.limelightAngle + limelight.ty);

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
        return angleArm - Constants.shooterToSpeakerAngle -0.07;
    }

    /*
    * subwoofer Depth = 36 
    * botToLimelight =  12
    * 
    * wallToLimelight = 48 inch
    */
    


    public void shoot(double input) {
        if (currentState == 2) {
            input /= 2;
        }
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
    boolean defenseFlag= false;
    public void setArmState(boolean stateUp, boolean stateDown, boolean defense) {
        if (stateUp && currentState < 2) {
            currentState++;
        }

        if (stateDown && currentState > 0) {
            currentState--;
        }

        if (defense) {
            defenseFlag = !defenseFlag;
        }

        if (defenseFlag) {
            shooter.moveArmPID(Constants.feedPosition);
        } else {
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
        
    }

    ColorSensorV3 sensor = new ColorSensorV3(I2C.Port.kMXP);
    public void intake(double input, boolean yButton, double shoot) {

        if (yButton) {
            shooter.collectNote(-0.2);
            shooter.shootNote(-0.2);
        // } else if (sensor.getRed() > 300 && shoot < 0.2) {
        //     input = 0;
        } else {
            shooter.collectNote(input * 0.6);
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

    LimelightTarget_Fiducial target = new LimelightTarget_Fiducial();

    public void limelightArmAngle(boolean xButton){
        limelight.updateLimelight();

        if(xButton){ //&& (target.fiducialID == 4 && DriverStation.getAlliance().get() == Alliance.Red) || (target.fiducialID == 7 && DriverStation.getAlliance().get() == Alliance.Blue)){
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
