package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.systems.Climber;
import frc.robot.systems.Driver;
import frc.robot.systems.Limelight;
import frc.robot.systems.LimelightHelpers;
import frc.robot.systems.Shooter;

public class Auto {

    Timer timer;

    /**
     * returns x position of desired field object accounting for starting alliance
     * 
     * @param number 0 is speaker, 1-3 are alliance notes ordered
     * from closest to amp side to furthest, 4-8 are mid notes ordered
     * from closest to amp side to furthest
     * @return x position of object
     */
    double getObjectPositionX(int number) {
        if (DriverStation.getAlliance().get() == Alliance.Blue || true) {
            double[] objectXBlue = {
                Units.inchesToMeters(51),
                
                Units.inchesToMeters(114),
                Units.inchesToMeters(114),
                Units.inchesToMeters(114),
                
                Units.inchesToMeters(326.6),
                Units.inchesToMeters(326.6),
                Units.inchesToMeters(326.6),
                Units.inchesToMeters(326.6),
                Units.inchesToMeters(326.6),
            };
            return objectXBlue[number];
        } else {
            double[] objectXRed = {
                Units.inchesToMeters(599),

                Units.inchesToMeters(536),
                Units.inchesToMeters(536),
                Units.inchesToMeters(536),
                
                Units.inchesToMeters(326.6),
                Units.inchesToMeters(326.6),
                Units.inchesToMeters(326.6),
                Units.inchesToMeters(326.6),
                Units.inchesToMeters(326.6),
            };
            return objectXRed[number];
        }
    }

    /**
     * returns y position of desired field object accounting for starting alliance
     * 
     * @param number 0 is speaker, 1-3 are alliance notes ordered
     * from closest to amp side to furthest, 4-8 are mid notes ordered
     * from closest to amp side to furthest
     * @return y position of object
     */
    double getObjectPositionY(int number) {
        if (DriverStation.getAlliance().get() == Alliance.Blue||true) {
            double[] objectYBlue = {
                Units.inchesToMeters(219),

                Units.inchesToMeters(276),
                Units.inchesToMeters(219),
                Units.inchesToMeters(162),
                
                Units.inchesToMeters(30),
                Units.inchesToMeters(96),
                Units.inchesToMeters(162),
                Units.inchesToMeters(228),
                Units.inchesToMeters(294),
            };
            return objectYBlue[number];
        } else {
            double[] objectYRed = {
                Units.inchesToMeters(219),

                Units.inchesToMeters(276),
                Units.inchesToMeters(219),
                Units.inchesToMeters(162),
                
                Units.inchesToMeters(30),
                Units.inchesToMeters(96),
                Units.inchesToMeters(162),
                Units.inchesToMeters(228),
                Units.inchesToMeters(294),
            };
            return objectYRed[number];
        }
    } 

    Pose2d getWaypoint(int number) {
        if (DriverStation.getAlliance().get() == Alliance.Blue || true) {
            Pose2d[] positionBlue = {
                new Pose2d(Units.inchesToMeters(30), Units.inchesToMeters(254), Rotation2d.fromDegrees(60)),
                new Pose2d(Units.inchesToMeters(51), Units.inchesToMeters(219), Rotation2d.fromDegrees(0)),
                new Pose2d(Units.inchesToMeters(30), Units.inchesToMeters(184), Rotation2d.fromDegrees(-60)),

                new Pose2d(Units.inchesToMeters(120), Units.inchesToMeters(120), Rotation2d.fromDegrees(0))

            };

            return positionBlue[number];

        } else {
            Pose2d[] positionRed = {
                new Pose2d(Units.inchesToMeters(620), Units.inchesToMeters(254), Rotation2d.fromDegrees(120)),
                new Pose2d(Units.inchesToMeters(599), Units.inchesToMeters(219), Rotation2d.fromDegrees(180)),
                new Pose2d(Units.inchesToMeters(620), Units.inchesToMeters(184), Rotation2d.fromDegrees(-120)),
                
                new Pose2d(Units.inchesToMeters(530), Units.inchesToMeters(120), Rotation2d.fromDegrees(180))

            };
            
            return positionRed[number];
        }
    }

    double fixAngle(double angle) {
        if (DriverStation.getAlliance().get() == Alliance.Blue || true) {
            return angle;
        } else {
            return 180 - angle;
        }
    }

    AutoHelper autoHelper;
    public Auto(Driver driver, Shooter shooter, Climber climber, AHRS navx, Timer timer, Limelight limelight) {
        this.autoHelper = new AutoHelper(driver, shooter, climber, navx, timer, limelight);
        this.timer = timer;
    }

    void restartTimer() {
        timer.reset();
        timer.start();    
    }

    boolean delayFinished = false;
    boolean driveFinished = false;
    boolean armFinished = false;
    boolean intakeFinished = false;
    boolean shootFinished = false;
    int counter = 0;

    /* 
            case x:
                autoHelper.angleArmToPosition(Constants.closeSpeakerPosition);
                autoHelper.intake(0.5);
                autoHelper.revShooter();
                autoHelper.prepNote(0.3);  
                autoHelper.driveToPosition(autoHelper.setDesiredPose(getObjectPositionX(2), getObjectPositionY(2), Math.toRadians(0)));

                if (autoHelper.delay(0.5)) {delayFinished = true;}
                if (autoHelper.hasNote()) {intakeFinished = true;}                
                if (autoHelper.noteRoutineFlag) {shootFinished = true;}
                if (autoHelper.armAtPosition()) {armFinished = true;}
                if (autoHelper.atTargetPosition()) {driveFinished = true;}
                if (driveFinished && intakeFinished) {counter += 1; resetFlags();}
                break;
    */
    public void resetFlags() {
        delayFinished = false; 
        intakeFinished = false; 
        shootFinished = false; 
        armFinished = false; 
        driveFinished = false;
    }

    public void autoSpeakerFourNote() {
        SmartDashboard.putNumber("Auto Phase:",counter);

        switch (counter) {
            case 0:
                autoHelper.angleArmToPosition(Constants.closeSpeakerPosition);
                autoHelper.revShooter();

                SmartDashboard.putBoolean("Auto State 0: arm", armFinished);
                
                if (autoHelper.delay(1)) {delayFinished = true;}
                if (autoHelper.armAtPosition()) {armFinished = true;}
                if (armFinished && delayFinished) {counter += 1; resetFlags();}
                break;
            
            case 1:
                autoHelper.angleArmToPosition(Constants.closeSpeakerPosition);
                autoHelper.revShooter();
                autoHelper.intake(0.5);

                if (autoHelper.delay(0.5)) {shootFinished = true;}
                SmartDashboard.putBoolean("Auto State 1: shoot", shootFinished);
                
                if (shootFinished) {counter += 1; resetFlags();}
                break;

            case 2:
                counter++;
                autoHelper.stopIntake();
                autoHelper.stopShoot();

                SmartDashboard.putBoolean("Auto State 2: stop", true);

                // if (autoHelper.intakeFlag) {shootFinished = true;}
                // if (shootFinished) {counter += 1; shootFinished = false;}
                break;

            case 3:
                autoHelper.angleArmToPosition(Constants.pickupPosition);
                autoHelper.driveToPosition(autoHelper.setDesiredPose(getObjectPositionX(2), getObjectPositionY(2), Math.toRadians(0)));
                autoHelper.intake(0.5);

                if (autoHelper.atTargetPosition()) {driveFinished = true;}
                if (autoHelper.armAtPosition()) {armFinished = true;}
                if (autoHelper.hasNote()) {intakeFinished = true;}
                SmartDashboard.putBoolean("Auto State 3: drive", driveFinished);
                SmartDashboard.putBoolean("Auto State 3: arm", armFinished);
                SmartDashboard.putBoolean("Auto State 3: intake", intakeFinished);
                
                if ((armFinished && driveFinished) || intakeFinished) {counter += 1; resetFlags();}
                break;

            case 4:
                counter++;
                break;

            case 5:
                autoHelper.revShooter();
                autoHelper.prepNote(0.3);  
                autoHelper.angleArmToPosition(Constants.closeSpeakerPosition);
                autoHelper.driveToPosition(autoHelper.setDesiredPose(getObjectPositionX(0), getObjectPositionY(0), Math.toRadians(fixAngle(0))));

                if (autoHelper.armAtPosition()) {armFinished = true;}
                if (autoHelper.delay(0.8)) {intakeFinished = true;}
                if (autoHelper.atTargetPosition()) {driveFinished = true;}
                SmartDashboard.putBoolean("Auto State 3: drive", driveFinished);
                SmartDashboard.putBoolean("Auto State 3: arm", armFinished);
                SmartDashboard.putBoolean("Auto State 3: intake", intakeFinished);
                
                if (intakeFinished && driveFinished) {counter += 1; resetFlags();}
                break;

            case 6:
                autoHelper.intake(0.5);
                autoHelper.revShooter();
                autoHelper.angleArmToPosition(Constants.closeSpeakerPosition);
                autoHelper.stopDrive();

                if (autoHelper.delay(0.5)) {shootFinished = true;}
                SmartDashboard.putBoolean("Auto State 5: shoot", shootFinished);
                
                if (shootFinished) {counter += 1; resetFlags();}
                break;
                
            case 7:
                autoHelper.intake(0.5);
                autoHelper.angleArmToPosition(Constants.pickupPosition);
                autoHelper.driveToPosition(autoHelper.setDesiredPose(getObjectPositionX(1), getObjectPositionY(1), Math.toRadians(fixAngle(40))));
                
                if (autoHelper.atTargetPosition()) {driveFinished = true;}
                if (autoHelper.hasNote()) {intakeFinished = true;}
                if (autoHelper.delay(3)) {delayFinished = true;}
                SmartDashboard.putBoolean("Auto State 6: drive", driveFinished);
                SmartDashboard.putBoolean("Auto State 6: intake", intakeFinished);
                SmartDashboard.putBoolean("Auto State 6: delay", delayFinished);
                
                if (intakeFinished || driveFinished || delayFinished) {counter += 1; resetFlags();}
                break;

            case 8:
                autoHelper.revShooter();
                autoHelper.prepNote(0.3);  
                autoHelper.angleArmToPosition(Constants.closeSpeakerPosition);
                autoHelper.driveToPosition(autoHelper.setDesiredPose(getObjectPositionX(0), getObjectPositionY(0), Math.toRadians(fixAngle(0))));
                
                if (autoHelper.atTargetPosition()) {driveFinished = true;}
                if (autoHelper.armAtPosition()) {armFinished = true;}
                SmartDashboard.putBoolean("Auto State 7: drive", driveFinished);
                SmartDashboard.putBoolean("Auto State 7: arm", armFinished);
                
                if (armFinished && driveFinished) {counter += 1; resetFlags();}
                break;

            case 9:
                autoHelper.intake(0.5);
                autoHelper.revShooter();
                autoHelper.angleArmToPosition(Constants.closeSpeakerPosition);
                autoHelper.stopDrive();

                if (autoHelper.delay(0.5)) {shootFinished = true;}
                SmartDashboard.putBoolean("Auto State 8: shoot", shootFinished);
                
                if (shootFinished) {counter += 1; resetFlags();}
                break;

            case 10:
                autoHelper.intake(0.5);
                autoHelper.angleArmToPosition(Constants.pickupPosition);
                autoHelper.driveToPosition(autoHelper.setDesiredPose(getObjectPositionX(3), getObjectPositionY(3), Math.toRadians(fixAngle(-40))));
                
                if (autoHelper.atTargetPosition()) {driveFinished = true;}
                if (autoHelper.hasNote()) {intakeFinished = true;}
                if (autoHelper.delay(3)) {delayFinished = true;}
                SmartDashboard.putBoolean("Auto State 9: drive", driveFinished);
                SmartDashboard.putBoolean("Auto State 9: intake", intakeFinished);
                SmartDashboard.putBoolean("Auto State 9: delay", delayFinished);

                if (intakeFinished || driveFinished || delayFinished) {counter += 1; resetFlags();}
                break;

            case 11:
                autoHelper.revShooter();
                autoHelper.prepNote(0.3);  
                autoHelper.angleArmToPosition(Constants.closeSpeakerPosition);
                autoHelper.driveToPosition(autoHelper.setDesiredPose(getObjectPositionX(0), getObjectPositionY(0), Math.toRadians(fixAngle(0))));
                
                if (autoHelper.atTargetPosition()) {driveFinished = true;}
                if (autoHelper.armAtPosition()) {armFinished = true;}
                SmartDashboard.putBoolean("Auto State 10: drive", driveFinished);
                SmartDashboard.putBoolean("Auto State 10: arm", armFinished);
                
                if (armFinished && driveFinished) {counter += 1; resetFlags();}
                break;

            case 12:
                autoHelper.intake(0.5);
                autoHelper.revShooter();
                autoHelper.angleArmToPosition(Constants.closeSpeakerPosition);
                autoHelper.stopDrive();

                if (autoHelper.delay(0.5)) {shootFinished = true;}
                SmartDashboard.putBoolean("Auto State 11: shoot", shootFinished);
                
                if (shootFinished) {counter += 1; resetFlags();}
                break;

            default:
                autoHelper.stopDrive();
                autoHelper.stopIntake();
                autoHelper.stopShoot();
                break;
        }
    }

    public void autoSpeakerLimelight() {
        SmartDashboard.putNumber("Auto Phase:",counter);

        switch (counter) {
            case 0:
                autoHelper.angleArmToPosition(Constants.closeSpeakerPosition);
                autoHelper.revShooter();

                SmartDashboard.putBoolean("Auto State 0: arm", armFinished);
                
                if (autoHelper.delay(0.5)) {delayFinished = true;}
                if (autoHelper.armAtPosition()) {armFinished = true;}
                if (armFinished && delayFinished) {counter += 1; armFinished = false; delayFinished = false;}
                break;
            
            case 1:
                autoHelper.angleArmToPosition(Constants.closeSpeakerPosition);
                autoHelper.revShooter();
                autoHelper.intake(0.5);

                SmartDashboard.putBoolean("Auto State 1: shoot", shootFinished);

                if (autoHelper.delay(0.5)) {shootFinished = true;}
                if (shootFinished) {counter += 1; shootFinished = false;}
                break;

            case 2:
                counter++;
                autoHelper.stopIntake();
                autoHelper.stopShoot();

                SmartDashboard.putBoolean("Auto State 2: stop", shootFinished);

                // if (autoHelper.intakeFlag) {shootFinished = true;}
                // if (shootFinished) {counter += 1; shootFinished = false;}
                break;

            case 3:
                autoHelper.angleArmToPosition(Constants.pickupPosition);
                autoHelper.driveToPosition(autoHelper.setDesiredPose(getObjectPositionX(2), getObjectPositionY(2), Math.toRadians(0)));
                autoHelper.intake(0.5);

                SmartDashboard.putBoolean("Auto State 3: drive", driveFinished);
                SmartDashboard.putBoolean("Auto State 3: arm", armFinished);
                SmartDashboard.putBoolean("Auto State 3: intake", intakeFinished);

                if (autoHelper.atTargetPosition()) {driveFinished = true;}
                if (autoHelper.armAtPosition()) {armFinished = true;}
                if (autoHelper.hasNote()) {intakeFinished = true;}
                if ((armFinished && driveFinished) || intakeFinished) {counter += 1; driveFinished = false; armFinished = false; intakeFinished = false;}
                break;

            case 4:
                counter++;
                break;

            case 5:
                autoHelper.revShooter();
                autoHelper.prepNote(0.3);  
                autoHelper.limelightMove(false);

                // SmartDashboard.putBoolean("Auto State 3: drive", driveFinished);
                // SmartDashboard.putBoolean("Auto State 3: arm", armFinished);
                // SmartDashboard.putBoolean("Auto State 3: intake", intakeFinished);

                // if (autoHelper.atTargetPosition()) {driveFinished = true;}
                // if (autoHelper.armAtPosition()) {armFinished = true;}
                if (autoHelper.delay(0.8)) {delayFinished = true;}
                if (autoHelper.armAtPosition()) {armFinished = true;}
                if (delayFinished && armFinished) {counter += 1; driveFinished = false; armFinished = false; delayFinished = false;}
                break;

            case 6:
                autoHelper.intake(0.5);
                autoHelper.revShooter();
                autoHelper.limelightMove(false);

                SmartDashboard.putBoolean("Auto State 5: shoot", shootFinished);
                
                if (autoHelper.delay(0.5)) {shootFinished = true;}
                if (shootFinished) {counter += 1; shootFinished = false;}
                break;
                
            case 7:
                autoHelper.intake(0.5);
                autoHelper.angleArmToPosition(Constants.pickupPosition);
                autoHelper.driveToPosition(autoHelper.setDesiredPose(getObjectPositionX(1), getObjectPositionY(1), Math.toRadians(fixAngle(90))));
                SmartDashboard.putBoolean("Auto State 6: drive", driveFinished);

                if (autoHelper.atTargetPosition()) {driveFinished = true;}
                if (autoHelper.hasNote()) {intakeFinished = true;}
                if (intakeFinished || driveFinished) {counter += 1; intakeFinished = false; driveFinished = false;}
                break;

            case 8:
                autoHelper.revShooter();
                autoHelper.prepNote(0.3);  
                autoHelper.limelightMove(false);

                SmartDashboard.putBoolean("Auto State 7: drive", driveFinished);
                SmartDashboard.putBoolean("Auto State 7: arm", armFinished);

                if (LimelightHelpers.getTX("") < 5) {driveFinished = true;}
                if (autoHelper.armAtPosition()) {armFinished = true;}
                if (armFinished && driveFinished) {counter += 1; armFinished = false; driveFinished = false;}
                break;

            case 9:
                autoHelper.intake(0.5);
                autoHelper.revShooter();
                autoHelper.angleArmToPosition(autoHelper.getLimelightAngle());
                autoHelper.stopDrive();

                SmartDashboard.putBoolean("Auto State 8: shoot", shootFinished);
                
                if (autoHelper.delay(0.5)) {shootFinished = true;}
                if (shootFinished) {counter += 1; shootFinished = false;}
                break;

            case 10:
                autoHelper.intake(0.5);
                autoHelper.angleArmToPosition(Constants.pickupPosition);
                autoHelper.driveToPosition(autoHelper.setDesiredPose(getObjectPositionX(3), getObjectPositionY(3), Math.toRadians(fixAngle(-90))));
                SmartDashboard.putBoolean("Auto State 9: drive", driveFinished);

                if (autoHelper.atTargetPosition()) {driveFinished = true;}
                if (autoHelper.hasNote()) {intakeFinished = true;}
                if (intakeFinished || driveFinished) {counter += 1; intakeFinished = false; driveFinished = false;}
                break;

            case 11:
                autoHelper.revShooter();
                autoHelper.prepNote(0.3);  
                autoHelper.limelightMove(true);

                SmartDashboard.putBoolean("Auto State 10: drive", driveFinished);
                SmartDashboard.putBoolean("Auto State 10: arm", armFinished);

                if (LimelightHelpers.getTX("") < 5) {driveFinished = true;}
                if (autoHelper.armAtPosition()) {armFinished = true;}
                if (armFinished && driveFinished) {counter += 1; armFinished = false; driveFinished = false;}
                break;

            case 12:
                autoHelper.intake(0.5);
                autoHelper.revShooter();
                autoHelper.angleArmToPosition(autoHelper.getLimelightAngle());
                autoHelper.stopDrive();

                SmartDashboard.putBoolean("Auto State 11: shoot", shootFinished);
                
                if (autoHelper.delay(0.5)) {shootFinished = true;}
                if (shootFinished) {counter += 1; shootFinished = false;}
                break;
    
            default:
                autoHelper.stopDrive();
                autoHelper.stopIntake();
                autoHelper.stopShoot();
                break;
        }
    }

    /**
     * start at speaker, score starting note, pickup and score note directly in front
     */
    public void autoSpeakerTwoNote() {
        SmartDashboard.putNumber("Auto Phase:",counter);

        switch (counter) {
            case 0:
                autoHelper.angleArmToPosition(Constants.closeSpeakerPosition);

                SmartDashboard.putBoolean("Auto State 0: arm", armFinished);

                if (autoHelper.armAtPosition()) {armFinished = true;}
                if (armFinished) {counter += 1; armFinished = false;}
                break;
            
            case 1:
                autoHelper.angleArmToPosition(Constants.closeSpeakerPosition);
                autoHelper.shootNoteRoutine();

                SmartDashboard.putBoolean("Auto State 1: shoot", shootFinished);

                if (autoHelper.noteRoutineFlag) {shootFinished = true;}
                if (shootFinished) {counter += 1; shootFinished = false;}
                break;

            case 2:
                counter++;
                autoHelper.stopIntake();
                autoHelper.stopShoot();

                SmartDashboard.putBoolean("Auto State 2: stop", shootFinished);

                // if (autoHelper.intakeFlag) {shootFinished = true;}
                // if (shootFinished) {counter += 1; shootFinished = false;}
                break;

            case 3:
                autoHelper.angleArmToPosition(Constants.pickupPosition);
                // autoHelper.driveToPosition(autoHelper.setDesiredPose(getObjectPositionX(2), getObjectPositionY(2), Math.toRadians(0)));
                // autoHelper.intake(true);

                SmartDashboard.putBoolean("Auto State 3: drive", driveFinished);
                SmartDashboard.putBoolean("Auto State 3: arm", armFinished);
                SmartDashboard.putBoolean("Auto State 3: intake", intakeFinished);

                // if (autoHelper.atTargetPosition()) {driveFinished = true;}
                if (autoHelper.armAtPosition()) {armFinished = true;}
                // if (autoHelper.hasNote()) {intakeFinished = true;}
                if (armFinished) {counter += 1; driveFinished = false; armFinished = false; intakeFinished = false;}
                break;

            case 4:
                autoHelper.angleArmToPosition(Constants.pickupPosition);
                autoHelper.driveToPosition(autoHelper.setDesiredPose(getObjectPositionX(2), getObjectPositionY(2), Math.toRadians(fixAngle(0))));
                autoHelper.intake(0.5);

                // SmartDashboard.putBoolean("Auto State 3: drive", driveFinished);
                // SmartDashboard.putBoolean("Auto State 3: arm", armFinished);
                // SmartDashboard.putBoolean("Auto State 3: intake", intakeFinished);

                if (autoHelper.atTargetPosition()) {driveFinished = true;}
                // if (autoHelper.armAtPosition()) {armFinished = true;}
                if (autoHelper.hasNote() ) {intakeFinished = true;}
                if (intakeFinished || driveFinished) {counter += 1; driveFinished = false; armFinished = false; intakeFinished = false;}
                break;

            case 5:
                autoHelper.prepNote(0.3);  
                
                // SmartDashboard.putBoolean("Auto State 3: drive", driveFinished);
                // SmartDashboard.putBoolean("Auto State 3: arm", armFinished);
                // SmartDashboard.putBoolean("Auto State 3: intake", intakeFinished);

                // if (autoHelper.atTargetPosition()) {driveFinished = true;}
                // if (autoHelper.armAtPosition()) {armFinished = true;}
                if (autoHelper.delay(0.3)) {intakeFinished = true;}
                if (intakeFinished) {counter += 1; driveFinished = false; armFinished = false; intakeFinished = false;}
                break;

            case 6:
                autoHelper.angleArmToPosition(Constants.closeSpeakerPosition);
                autoHelper.driveToPosition(autoHelper.setDesiredPose(getObjectPositionX(0), getObjectPositionY(0), Math.toRadians(fixAngle(0))));

                SmartDashboard.putBoolean("Auto State 4: drive", driveFinished);
                SmartDashboard.putBoolean("Auto State 4: arm", armFinished);
                
                if (autoHelper.atTargetPosition()) {driveFinished = true;}
                if (autoHelper.armAtPosition()) {armFinished = true;}
                if (driveFinished && armFinished) {counter += 1; driveFinished = false; armFinished = false;}
                break;

            case 7:
                autoHelper.angleArmToPosition(Constants.closeSpeakerPosition);
                autoHelper.shootNoteRoutine();
                autoHelper.stopDrive();

                SmartDashboard.putBoolean("Auto State 5: shoot", shootFinished);
                
                if (autoHelper.noteRoutineFlag) {shootFinished = true;}
                if (shootFinished) {counter += 1; shootFinished = false;}
                break;

            case 8:
                autoHelper.driveToPosition(autoHelper.setDesiredPose(Units.inchesToMeters(180), Units.inchesToMeters(211), Math.toRadians(fixAngle(0))));
                
                SmartDashboard.putBoolean("Auto State 6: drive", driveFinished);

                if (autoHelper.atTargetPosition()) {driveFinished = true;}
                if (driveFinished) {counter += 1; driveFinished = false;}
                break;

            default:
                autoHelper.stopDrive();
                autoHelper.stopIntake();
                autoHelper.stopShoot();
                break;
        }
    }

    public void autoCloseTwoNote() {
        SmartDashboard.putNumber("Auto Phase:",counter);

        switch (counter) {
            case 0:
                autoHelper.angleArmToPosition(Constants.closeSpeakerPosition);

                SmartDashboard.putBoolean("Auto State 0: arm", armFinished);

                if (autoHelper.armAtPosition()) {armFinished = true;}
                if (armFinished) {counter += 1; armFinished = false;}
                break;
            
            case 1:
                autoHelper.angleArmToPosition(Constants.closeSpeakerPosition);
                autoHelper.shootNoteRoutine();

                SmartDashboard.putBoolean("Auto State 1: shoot", shootFinished);

                if (autoHelper.noteRoutineFlag) {shootFinished = true;}
                if (shootFinished) {counter += 1; shootFinished = false;}
                break;

            case 2:
                counter++;
                autoHelper.stopIntake();
                autoHelper.stopShoot();

                SmartDashboard.putBoolean("Auto State 2: stop", shootFinished);

                // if (autoHelper.intakeFlag) {shootFinished = true;}
                // if (shootFinished) {counter += 1; shootFinished = false;}
                break;

            case 3:
                autoHelper.angleArmToPosition(Constants.pickupPosition);
                // autoHelper.driveToPosition(autoHelper.setDesiredPose(getObjectPositionX(2), getObjectPositionY(2), Math.toRadians(0)));
                // autoHelper.intake(true);

                SmartDashboard.putBoolean("Auto State 3: drive", driveFinished);
                SmartDashboard.putBoolean("Auto State 3: arm", armFinished);
                SmartDashboard.putBoolean("Auto State 3: intake", intakeFinished);

                // if (autoHelper.atTargetPosition()) {driveFinished = true;}
                if (autoHelper.armAtPosition()) {armFinished = true;}
                if (autoHelper.hasNote()) {intakeFinished = true;}
                if (armFinished) {counter += 1; driveFinished = false; armFinished = false; intakeFinished = false;}
                break;

            case 4:
                autoHelper.angleArmToPosition(Constants.pickupPosition);
                autoHelper.driveToPosition(autoHelper.setDesiredPose(getObjectPositionX(1), getObjectPositionY(1), Math.toRadians(10)));
                autoHelper.intake(0.5);

                // SmartDashboard.putBoolean("Auto State 3: drive", driveFinished);
                // SmartDashboard.putBoolean("Auto State 3: arm", armFinished);
                // SmartDashboard.putBoolean("Auto State 3: intake", intakeFinished);

                // if (autoHelper.atTargetPosition()) {driveFinished = true;}
                // if (autoHelper.armAtPosition()) {armFinished = true;}
                if (autoHelper.hasNote()) {intakeFinished = true;}
                if (intakeFinished) {counter += 1; driveFinished = false; armFinished = false; intakeFinished = false;}
                break;

            case 5:
                autoHelper.prepNote(0.3);  
                
                // SmartDashboard.putBoolean("Auto State 3: drive", driveFinished);
                // SmartDashboard.putBoolean("Auto State 3: arm", armFinished);
                // SmartDashboard.putBoolean("Auto State 3: intake", intakeFinished);

                // if (autoHelper.atTargetPosition()) {driveFinished = true;}
                // if (autoHelper.armAtPosition()) {armFinished = true;}
                if (autoHelper.delay(0.3)) {intakeFinished = true;}
                if (intakeFinished) {counter += 1; driveFinished = false; armFinished = false; intakeFinished = false;}
                break;

            case 6:
                autoHelper.angleArmToPosition(Constants.closeSpeakerPosition);
                autoHelper.driveToPosition(getWaypoint(0));

                SmartDashboard.putBoolean("Auto State 4: drive", driveFinished);
                SmartDashboard.putBoolean("Auto State 4: arm", armFinished);
                
                if (autoHelper.atTargetPosition()) {driveFinished = true;}
                if (autoHelper.armAtPosition()) {armFinished = true;}
                if (driveFinished && armFinished) {counter += 1; driveFinished = false; armFinished = false;}
                break;

            case 7:
                autoHelper.angleArmToPosition(Constants.closeSpeakerPosition);
                autoHelper.shootNoteRoutine();
                autoHelper.stopDrive();

                SmartDashboard.putBoolean("Auto State 5: shoot", shootFinished);
                
                if (autoHelper.noteRoutineFlag) {shootFinished = true;}
                if (shootFinished) {counter += 1; shootFinished = false;}
                break;

            case 8:
                autoHelper.driveToPosition(autoHelper.setDesiredPose(Units.inchesToMeters(180), Units.inchesToMeters(211), 0));
                
                SmartDashboard.putBoolean("Auto State 6: drive", driveFinished);

                if (autoHelper.atTargetPosition()) {driveFinished = true;}
                if (driveFinished) {counter += 1; driveFinished = false;}
                break;

            default:
                autoHelper.stopDrive();
                autoHelper.stopIntake();
                autoHelper.stopShoot();
                break;
        }
    }
    
    public void autoFarTwoNote() {
        SmartDashboard.putNumber("Auto Phase:",counter);

        switch (counter) {
            case 0:
                autoHelper.angleArmToPosition(Constants.closeSpeakerPosition);

                SmartDashboard.putBoolean("Auto State 0: arm", armFinished);

                if (autoHelper.armAtPosition()) {armFinished = true;}
                if (armFinished) {counter += 1; armFinished = false;}
                break;
            
            case 1:
                autoHelper.angleArmToPosition(Constants.closeSpeakerPosition);
                autoHelper.shootNoteRoutine();

                SmartDashboard.putBoolean("Auto State 1: shoot", shootFinished);

                if (autoHelper.noteRoutineFlag) {shootFinished = true;}
                if (shootFinished) {counter += 1; shootFinished = false;}
                break;

            case 2:
                counter++;
                autoHelper.stopIntake();
                autoHelper.stopShoot();

                SmartDashboard.putBoolean("Auto State 2: stop", shootFinished);

                // if (autoHelper.intakeFlag) {shootFinished = true;}
                // if (shootFinished) {counter += 1; shootFinished = false;}
                break;

            case 3:
                autoHelper.angleArmToPosition(Constants.pickupPosition);
                // autoHelper.driveToPosition(autoHelper.setDesiredPose(getObjectPositionX(2), getObjectPositionY(2), Math.toRadians(0)));
                // autoHelper.intake(true);

                SmartDashboard.putBoolean("Auto State 3: drive", driveFinished);
                SmartDashboard.putBoolean("Auto State 3: arm", armFinished);
                SmartDashboard.putBoolean("Auto State 3: intake", intakeFinished);

                // if (autoHelper.atTargetPosition()) {driveFinished = true;}
                if (autoHelper.armAtPosition()) {armFinished = true;}
                // if (autoHelper.hasNote()) {intakeFinished = true;}
                if (armFinished) {counter += 1; driveFinished = false; armFinished = false; intakeFinished = false;}
                break;

            case 4:
                autoHelper.angleArmToPosition(Constants.pickupPosition);
                autoHelper.driveToPosition(getWaypoint(3));
                // autoHelper.intake(0.5);

                // SmartDashboard.putBoolean("Auto State 3: drive", driveFinished);
                // SmartDashboard.putBoolean("Auto State 3: arm", armFinished);
                // SmartDashboard.putBoolean("Auto State 3: intake", intakeFinished);

                // if (autoHelper.atTargetPosition()) {driveFinished = true;}
                // if (autoHelper.armAtPosition()) {armFinished = true;}
                if (autoHelper.atTargetPosition()) {driveFinished = true;}
                if (driveFinished) {counter += 1; driveFinished = false; armFinished = false; intakeFinished = false;}
                break;

            case 5:
                autoHelper.angleArmToPosition(Constants.pickupPosition);
                autoHelper.driveToPosition(autoHelper.setDesiredPose(getObjectPositionX(8), getObjectPositionY(8), Math.toRadians(fixAngle(0))));
                autoHelper.intake(0.5);

                // SmartDashboard.putBoolean("Auto State 3: drive", driveFinished);
                // SmartDashboard.putBoolean("Auto State 3: arm", armFinished);
                // SmartDashboard.putBoolean("Auto State 3: intake", intakeFinished);

                if (autoHelper.atTargetPosition()) {driveFinished = true;}
                // if (autoHelper.armAtPosition()) {armFinished = true;}
                if (autoHelper.hasNote()) {intakeFinished = true;}
                if (intakeFinished || driveFinished) {counter += 1; driveFinished = false; armFinished = false; intakeFinished = false;}
                break;

            case 6:
                autoHelper.angleArmToPosition(Constants.closeSpeakerPosition);
                autoHelper.driveToPosition(getWaypoint(3));
                // autoHelper.intake(0.5);

                // SmartDashboard.putBoolean("Auto State 3: drive", driveFinished);
                // SmartDashboard.putBoolean("Auto State 3: arm", armFinished);
                // SmartDashboard.putBoolean("Auto State 3: intake", intakeFinished);

                // if (autoHelper.atTargetPosition()) {driveFinished = true;}
                // if (autoHelper.armAtPosition()) {armFinished = true;}
                if (autoHelper.atTargetPosition()) {driveFinished = true;}
                if (driveFinished) {counter += 1; driveFinished = false; armFinished = false; intakeFinished = false;}
                break;

            case 7:
                autoHelper.angleArmToPosition(Constants.closeSpeakerPosition);
                autoHelper.driveToPosition(getWaypoint(3));
                // autoHelper.intake(0.5);

                // SmartDashboard.putBoolean("Auto State 3: drive", driveFinished);
                // SmartDashboard.putBoolean("Auto State 3: arm", armFinished);
                // SmartDashboard.putBoolean("Auto State 3: intake", intakeFinished);

                // if (autoHelper.atTargetPosition()) {driveFinished = true;}
                // if (autoHelper.armAtPosition()) {armFinished = true;}
                if (autoHelper.atTargetPosition()) {driveFinished = true;}
                if (driveFinished) {counter += 1; driveFinished = false; armFinished = false; intakeFinished = false;}
                break;

            case 8:
                autoHelper.angleArmToPosition(Constants.closeSpeakerPosition);
                autoHelper.shootNoteRoutine();
                autoHelper.stopDrive();

                SmartDashboard.putBoolean("Auto State 5: shoot", shootFinished);
                
                if (autoHelper.noteRoutineFlag) {shootFinished = true;}
                if (shootFinished) {counter += 1; shootFinished = false;}
                break;

            case 9:
                autoHelper.angleArmToPosition(Constants.closeSpeakerPosition);
                autoHelper.driveToPosition(getWaypoint(3));

                SmartDashboard.putBoolean("Auto State 5: shoot", shootFinished);
                
                if (autoHelper.noteRoutineFlag) {shootFinished = true;}
                if (shootFinished) {counter += 1; shootFinished = false;}
                break;

            default:
                autoHelper.stopDrive();
                autoHelper.stopIntake();
                autoHelper.stopShoot();
                autoHelper.reset();
                break;
        }
    }

    public void autoBlueCloseRedFar() {
        SmartDashboard.putNumber("Auto Phase:",counter);

        switch (counter) {
            case 0:
                autoHelper.angleArmToPosition(Constants.closeSpeakerPosition);

                SmartDashboard.putBoolean("Auto State 0: arm", armFinished);

                if (autoHelper.armAtPosition()) {armFinished = true;}
                if (armFinished) {counter += 1; armFinished = false;}
                break;
            
            case 1:
                autoHelper.angleArmToPosition(Constants.closeSpeakerPosition);
                autoHelper.shootNoteRoutine();

                SmartDashboard.putBoolean("Auto State 1: shoot", shootFinished);

                if (autoHelper.noteRoutineFlag) {shootFinished = true;}
                if (shootFinished) {counter += 1; shootFinished = false;}
                break;

            case 2:
                autoHelper.stopIntake();
                autoHelper.stopShoot();

                SmartDashboard.putBoolean("Auto State 2: stop", shootFinished);

                if (autoHelper.delay(10)) {delayFinished = true;}
                if (delayFinished) {counter += 1; shootFinished = false;}
                break;

            case 3:
                autoHelper.angleArmToPosition(Constants.pickupPosition);
                autoHelper.driveToPosition(autoHelper.setDesiredPose(1, -0.65, -60));
                // autoHelper.intake(0.5);

                // SmartDashboard.putBoolean("Auto State 3: drive", driveFinished);
                // SmartDashboard.putBoolean("Auto State 3: arm", armFinished);
                // SmartDashboard.putBoolean("Auto State 3: intake", intakeFinished);

                // if (autoHelper.atTargetPosition()) {driveFinished = true;}
                // if (autoHelper.armAtPosition()) {armFinished = true;}
                if (autoHelper.atTargetPosition()) {driveFinished = true;}
                if (driveFinished) {counter += 1; driveFinished = false; armFinished = false; intakeFinished = false;}
                break;

            
            default:
                autoHelper.stopDrive();
                autoHelper.stopIntake();
                autoHelper.stopShoot();
                break;
        }
    }

    public void autoBlueFarRedClose() {
        SmartDashboard.putNumber("Auto Phase:",counter);

        switch (counter) {
            case 0:
                autoHelper.angleArmToPosition(Constants.closeSpeakerPosition);

                SmartDashboard.putBoolean("Auto State 0: arm", armFinished);

                if (autoHelper.armAtPosition()) {armFinished = true;}
                if (armFinished) {counter += 1; armFinished = false;}
                break;
            
            case 1:
                autoHelper.angleArmToPosition(Constants.closeSpeakerPosition);
                autoHelper.shootNoteRoutine();

                SmartDashboard.putBoolean("Auto State 1: shoot", shootFinished);

                if (autoHelper.noteRoutineFlag) {shootFinished = true;}
                if (shootFinished) {counter += 1; shootFinished = false;}
                break;

            case 2:
                autoHelper.stopIntake();
                autoHelper.stopShoot();

                SmartDashboard.putBoolean("Auto State 2: stop", shootFinished);

                if (autoHelper.delay(10)) {delayFinished = true;}
                if (delayFinished) {counter += 1; shootFinished = false;}
                break;

            case 3:
                autoHelper.angleArmToPosition(Constants.pickupPosition);
                autoHelper.driveToPosition(autoHelper.setDesiredPose(1, 0.65, 60));
                // autoHelper.intake(0.5);

                // SmartDashboard.putBoolean("Auto State 3: drive", driveFinished);
                // SmartDashboard.putBoolean("Auto State 3: arm", armFinished);
                // SmartDashboard.putBoolean("Auto State 3: intake", intakeFinished);

                // if (autoHelper.atTargetPosition()) {driveFinished = true;}
                // if (autoHelper.armAtPosition()) {armFinished = true;}
                if (autoHelper.atTargetPosition()) {driveFinished = true;}
                if (driveFinished) {counter += 1; driveFinished = false; armFinished = false; intakeFinished = false;}
                break;

            
            default:
                autoHelper.stopDrive();
                autoHelper.stopIntake();
                autoHelper.stopShoot();
                break;
        }
    }

    // public void autoAmpTwoNote() {
    //     SmartDashboard.putNumber("auto",counter);
    //     switch (counter) {
    //         case 0:
    //             autoHelper.angleArmToPosition(Constants.closeSpeakerPosition);

    //             if (autoHelper.armAtPosition()) {armFinished = true;}
    //             if (armFinished) {counter += 1; armFinished = false;}
    //             break;
            
    //         case 1:
    //             autoHelper.shooter.shootNoteRoutine();

    //             if (!autoHelper.hasNote()) {shootFinished = true;}
    //             if (shootFinished) {counter += 1; shootFinished = false;}
    //             break;

    //         case 2:
    //             autoHelper.angleArmToPosition(Constants.pickupPosition);
    //             autoHelper.driveToPosition(autoHelper.setDesiredPose(noteX[1], noteY[1], 0));
    //             autoHelper.intake(true);

    //             if (autoHelper.atTargetPosition()) {driveFinished = true;}
    //             if (autoHelper.armAtPosition()) {armFinished = true;}
    //             if (autoHelper.hasNote()) {intakeFinished = true;}
    //             if (driveFinished && armFinished && intakeFinished) {counter += 1; driveFinished = false; armFinished = false; intakeFinished = false;}
    //             break;

    //         case 3:
    //             autoHelper.angleArmToPosition(Constants.closeSpeakerPosition);
    //             autoHelper.driveToPosition(autoHelper.setDesiredPose(speakerPosX, speakerPosY, 0));

    //             if (autoHelper.atTargetPosition()) {driveFinished = true;}
    //             if (autoHelper.armAtPosition()) {armFinished = true;}
    //             if (driveFinished && armFinished) {counter += 1; driveFinished = false; armFinished = false;}
    //             break;

    //         case 4:
    //             autoHelper.shooter.shootNoteRoutine();
                
    //             if (!autoHelper.hasNote()) {shootFinished = true;}
    //             if (shootFinished) {counter += 1; shootFinished = false;}
    //             break;

    //         case 5:
    //             autoHelper.driveToPosition(autoHelper.setDesiredPose(Units.inchesToMeters(180), Units.inchesToMeters(211), counter));;
                
    //             if (autoHelper.atTargetPosition()) {driveFinished = true;}
    //             if (driveFinished) {counter += 1; driveFinished = false;}
    //             break;

    //         default:
    //             autoHelper.stopDrive();
    //             break;
    //     }
    // }

    // public void autoSupport() {
    //     switch (counter) {
    //         case 0:
    //             autoHelper.angleArmToPosition(Constants.closeSpeakerPosition);

    //             if (autoHelper.armAtPosition()) {armFinished = true;}
    //             if (armFinished) {counter += 1; armFinished = false;}
    //             break;
            
    //         case 1:
    //             autoHelper.shooter.shootNoteRoutine();

    //             if (!autoHelper.hasNote()) {shootFinished = true;}
    //             if (shootFinished) {counter += 1; shootFinished = false;}
    //             break;

    //         case 2:
    //             autoHelper.angleArmToPosition(Constants.pickupPosition);
    //             autoHelper.driveToPosition(autoHelper.setDesiredPose(noteX[1], noteY[1], 0));
    //             autoHelper.intake(true);

    //             if (autoHelper.atTargetPosition()) {driveFinished = true;}
    //             if (autoHelper.armAtPosition()) {armFinished = true;}
    //             if (autoHelper.hasNote()) {intakeFinished = true;}
    //             if (driveFinished && armFinished && intakeFinished) {counter += 1; driveFinished = false; armFinished = false; intakeFinished = false;}
    //             break;

    //         case 3:
    //             autoHelper.angleArmToPosition(Constants.closeSpeakerPosition);
    //             autoHelper.driveToPosition(autoHelper.setDesiredPose(speakerPosX, speakerPosY, 0));

    //             if (autoHelper.atTargetPosition()) {driveFinished = true;}
    //             if (autoHelper.armAtPosition()) {armFinished = true;}
    //             if (driveFinished && armFinished) {counter += 1; driveFinished = false; armFinished = false;}
    //             break;

    //         case 4:
    //             autoHelper.shooter.shootNoteRoutine();
                
    //             if (!autoHelper.hasNote()) {shootFinished = true;}
    //             if (shootFinished) {counter += 1; shootFinished = false;}
    //             break;

    //         case 5:
    //             autoHelper.driveToPosition(autoHelper.setDesiredPose(Units.inchesToMeters(180), Units.inchesToMeters(211), counter));;
                
    //             if (autoHelper.atTargetPosition()) {driveFinished = true;}
    //             if (driveFinished) {counter += 1; driveFinished = false;}
    //             break;

    //         default:
    //             autoHelper.stopDrive();
    //             break;
    //     }
    // }

    public void autoProgramTest() {
        SmartDashboard.putNumber("auto",counter);
        switch (counter) {
            case 0:
                autoHelper.driveToPosition(autoHelper.setDesiredPose(1, 0, 0));

                if (autoHelper.atTargetPosition()) {driveFinished = true;}
                if (driveFinished) {counter += 1; driveFinished = false;}
                break;

            case 1:
                autoHelper.driveToPosition(autoHelper.setDesiredPose(1, 0, Math.PI));
                if (autoHelper.atTargetPosition()) {driveFinished = true;}  

                // autoHelper.shootNote();
                // if (!autoHelper.hasNote()) {shootFinished = true;}
                if (driveFinished /*&& shootFinished*/) {counter += 1; driveFinished = shootFinished = false;}
                break;

            case 2:
                autoHelper.driveToPosition(autoHelper.setDesiredPose(1, 1, 0));
                
                if (autoHelper.atTargetPosition()) {driveFinished = true;}
                if (driveFinished) {counter += 1; driveFinished = false;}
                break;

            case 3: 
                autoHelper.driveToPosition(autoHelper.setDesiredPose(1, 1, -Math.PI/2));

                if (autoHelper.atTargetPosition()) {driveFinished = true;}
                if (driveFinished) {counter += 1; driveFinished = false;}
                break;

            case 4:
                autoHelper.driveToPosition(autoHelper.setDesiredPose(0, 0, 0));

                if (autoHelper.atTargetPosition()) {driveFinished = true;}
                if (driveFinished) {counter += 1; driveFinished = false;}
                break;

            // case 2:
            //     autoHelper.driveToPosition(autoHelper.setDesiredPose(-0.5, -0.5, -Math.PI/2));

            //     if (autoHelper.atTargetPosition()) {driveFinished = true;}
            //     if (driveFinished) {counter += 1; driveFinished = false;}
            //     break;

            // case 3:
            //     autoHelper.driveToPosition(autoHelper.setDesiredPose(1, 1, Math.PI/2));

            //     if (autoHelper.atTargetPosition()) {driveFinished = true;}
            //     if (driveFinished) {counter += 1; driveFinished = false;}
            //     break;

            

            default: autoHelper.stopDrive();
                break;
        }
    }
    
    
    // public void autoMidBalance() {
    //     if (autoHelper.timerInterval_Auto(0, 2.5)){
    //         autoHelper.autoDriveRotate(Math.PI-0.01); // face nodes
    //         autoHelper.resetDriveEncoders();
    //     }else if (autoHelper.timerInterval_Auto(2.51, 3)){
    //         autoHelper.driveSwerve_EncoderIf_FwdAndBwd(0.21); //drive forwards to thingy
    //         //swerveDrive(-0.2, 0, 0);
    //     }else if (autoHelper.timerInterval_Auto(3.01, 6)){
    //         autoHelper.driveSwerve_EncoderIf_FwdAndBwd(0);
    //         // armRotate_encoderIf_upAndDown(-1.80); //lift arm
    //     }else if (autoHelper.timerInterval_Auto(6.01, 8)){
    //         // armExtend_encoderIf_outAndIn(0.75); // extend arm
    //     }else if (autoHelper.timerInterval_Auto(8.01, 8.5)){
    //         // dSolenoidClaw.set(Value.kForward); //open claw
    //     }else if (autoHelper.timerInterval_Auto(8.51, 10)){
    //         // armExtend_encoderIf_outAndIn(0); //retract
    //         // dSolenoidClaw.set(Value.kReverse); //close claw
    //     }else if (autoHelper.timerInterval_Auto(10.01, 12)){
    //         // armRotate_encoderIf_upAndDown(-0.1); //lower arm
    //     }else if (autoHelper.timerInterval_Auto(12.01, 15)){
    //         autoHelper.driveSwerve_EncoderIf_FwdAndBwd(-2.3); //drive backwards past line
    //     }else { //STOP!!!
    //         autoHelper.driveSwerve_EncoderIf_FwdAndBwd(0);
    //         // armRotate.tankDrive(0, 0);      
    //         // armTalonExtenstion.set(0);
    //     }
    // }

    //     public void autoTopAndBottom() {
    //     if (autoHelper.timerInterval_Auto(0, 2)){
    //         autoHelper.autoDriveRotate(Math.PI-0.01); // face nodes
    //     }else if (autoHelper.timerInterval_Auto(2.01, 3.50)){
    //         autoHelper.driveSwerve_EncoderIf_FwdAndBwd(0.21); //drive forwards to thingy
    //         autoHelper.resetDriveEncoders();
    //         //swerveDrive(-0.3, 0, 0);
    //     }else if (autoHelper.timerInterval_Auto(3.51, 6)){
    //         autoHelper.driveSwerve_EncoderIf_FwdAndBwd(0);
    //         // armRotate_encoderIf_upAndDown(-1.80); //lift arm
    //     }else if (autoHelper.timerInterval_Auto(6.01, 8)){
    //         // armExtend_encoderIf_outAndIn(0.75); // extend arm
    //     }else if (autoHelper.timerInterval_Auto(8.01, 8.5)){
    //         // dSolenoidClaw.set(Value.kForward); //open claw
    //     }else if (autoHelper.timerInterval_Auto(8.51, 10)){
    //         // armExtend_encoderIf_outAndIn(0); //retract
    //         // dSolenoidClaw.set(Value.kReverse); //close claw
    //     }else if (autoHelper.timerInterval_Auto(10.01, 12)){
    //         // armRotate_encoderIf_upAndDown(-0.1); //lower arm
    //     }else if (autoHelper.timerInterval_Auto(12.01, 15)){
    //         autoHelper.driveSwerve_EncoderIf_FwdAndBwd(-4.3); //drive backwards past line
    //     }else { //STOP!!!
    //         autoHelper.driveSwerve_EncoderIf_FwdAndBwd(0);
    //         // armRotate.tankDrive(0, 0);      
    //         // armTalonExtenstion.set(0);
    //     }
    // }
    




}