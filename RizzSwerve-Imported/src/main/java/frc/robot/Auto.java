package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.systems.Climber;
import frc.robot.systems.Driver;
import frc.robot.systems.Limelight;
import frc.robot.systems.Shooter;

public class Auto {

    Timer timer;

    double[] noteX = {
        Units.inchesToMeters(114),
        Units.inchesToMeters(114),
        Units.inchesToMeters(114),

        Units.inchesToMeters(326.6),
        Units.inchesToMeters(326.6),
        Units.inchesToMeters(326.6),
        Units.inchesToMeters(326.6),
        Units.inchesToMeters(326.6),
    };

    double[] noteY = {
        Units.inchesToMeters(267),
        Units.inchesToMeters(219),
        Units.inchesToMeters(162),

        Units.inchesToMeters(30),
        Units.inchesToMeters(96),
        Units.inchesToMeters(162),
        Units.inchesToMeters(228),
        Units.inchesToMeters(294),
    };

    double speakerPosX = Units.inchesToMeters(41);
    double speakerPosY = Units.inchesToMeters(231);

    AutoHelper autoHelper;
    public Auto(Driver driver, Shooter shooter, Climber climber, AHRS navx, Timer timer, Limelight limelight) {
        this.autoHelper = new AutoHelper(driver, shooter, climber, navx, timer, limelight);
        this.timer = timer;
    }

    void restartTimer() {
        timer.reset();
        timer.start();    
    }

    public isRed = false;
    public invertIfRed(double inputY) {
        if (isRed) {
            return -inputY;
        } else return inputY;
    }

    boolean driveFinished = false;
    boolean armFinished = false;
    boolean intakeFinished = false;
    boolean shootFinished = false;
    int counter = 0;
    /**
     * start at speaker, score starting note, pickup and score note directly in front
     */
    public void autoSpeakerTwoNote() {
        SmartDashboard.putNumber("auto",counter);
        switch (counter) {
            case 0:
                autoHelper.angleArmToPosition(Constants.ampPosition);

                if (autoHelper.armAtPosition()) {armFinished = true;}
                if (armFinished) {counter += 1; armFinished = false;}
                break;
            
            case 1:
                autoHelper.shooter.shootNoteRoutine();

                if (!autoHelper.hasNote()) {shootFinished = true;}
                if (shootFinished) {counter += 1; shootFinished = false;}
                break;

            case 2:
                autoHelper.angleArmToPosition(Constants.pickupPosition);
                autoHelper.driveToPosition(autoHelper.setDesiredPose(noteX[0], invertIfRed(noteY[0]), Math.toRadians(45)));
                autoHelper.intake(true);

                if (autoHelper.atTargetPosition()) {driveFinished = true;}
                if (autoHelper.armAtPosition()) {armFinished = true;}
                if (autoHelper.hasNote()) {intakeFinished = true;}
                if (driveFinished && armFinished && intakeFinished) {counter += 1; driveFinished = false; armFinished = false; intakeFinished = false;}
                break;

            case 3:
                autoHelper.angleArmToPosition(Constants.closeSpeakerPosition);
                autoHelper.driveToPosition(autoHelper.setDesiredPose(speakerPosX, invertIfRed(speakerPosY), 0));

                if (autoHelper.atTargetPosition()) {driveFinished = true;}
                if (autoHelper.armAtPosition()) {armFinished = true;}
                if (driveFinished && armFinished) {counter += 1; driveFinished = false; armFinished = false;}
                break;

            case 4:
                autoHelper.shooter.shootNoteRoutine();
                
                if (!autoHelper.hasNote()) {shootFinished = true;}
                if (shootFinished) {counter += 1; shootFinished = false;}
                break;

            case 5:
                autoHelper.driveToPosition(autoHelper.setDesiredPose(Units.inchesToMeters(180), Units.inchesToMeters(211), counter));;
                
                if (autoHelper.atTargetPosition()) {driveFinished = true;}
                if (driveFinished) {counter += 1; driveFinished = false;}
                break;

            default:
                autoHelper.stop();
                break;
        }
    }

    public void autoAmpTwoNote() {
        SmartDashboard.putNumber("auto",counter);
        switch (counter) {
            case 0:
                autoHelper.angleArmToPosition(Constants.closeSpeakerPosition);

                if (autoHelper.armAtPosition()) {armFinished = true;}
                if (armFinished) {counter += 1; armFinished = false;}
                break;
            
            case 1:
                autoHelper.shooter.shootNoteRoutine();

                if (!autoHelper.hasNote()) {shootFinished = true;}
                if (shootFinished) {counter += 1; shootFinished = false;}
                break;

            case 2:
                autoHelper.angleArmToPosition(Constants.pickupPosition);
                autoHelper.driveToPosition(autoHelper.setDesiredPose(noteX[1], noteY[1], 0));
                autoHelper.intake(true);

                if (autoHelper.atTargetPosition()) {driveFinished = true;}
                if (autoHelper.armAtPosition()) {armFinished = true;}
                if (autoHelper.hasNote()) {intakeFinished = true;}
                if (driveFinished && armFinished && intakeFinished) {counter += 1; driveFinished = false; armFinished = false; intakeFinished = false;}
                break;

            case 3:
                autoHelper.angleArmToPosition(Constants.closeSpeakerPosition);
                autoHelper.driveToPosition(autoHelper.setDesiredPose(speakerPosX, speakerPosY, 0));

                if (autoHelper.atTargetPosition()) {driveFinished = true;}
                if (autoHelper.armAtPosition()) {armFinished = true;}
                if (driveFinished && armFinished) {counter += 1; driveFinished = false; armFinished = false;}
                break;

            case 4:
                autoHelper.shooter.shootNoteRoutine();
                
                if (!autoHelper.hasNote()) {shootFinished = true;}
                if (shootFinished) {counter += 1; shootFinished = false;}
                break;

            case 5:
                autoHelper.driveToPosition(autoHelper.setDesiredPose(Units.inchesToMeters(180), Units.inchesToMeters(211), counter));;
                
                if (autoHelper.atTargetPosition()) {driveFinished = true;}
                if (driveFinished) {counter += 1; driveFinished = false;}
                break;

            default:
                autoHelper.stop();
                break;
        }
    }

    public void autoSupport() {
        switch (counter) {
            case 0:
                autoHelper.angleArmToPosition(Constants.closeSpeakerPosition);

                if (autoHelper.armAtPosition()) {armFinished = true;}
                if (armFinished) {counter += 1; armFinished = false;}
                break;
            
            case 1:
                autoHelper.shooter.shootNoteRoutine();

                if (!autoHelper.hasNote()) {shootFinished = true;}
                if (shootFinished) {counter += 1; shootFinished = false;}
                break;

            case 2:
                autoHelper.angleArmToPosition(Constants.pickupPosition);
                autoHelper.driveToPosition(autoHelper.setDesiredPose(noteX[1], noteY[1], 0));
                autoHelper.intake(true);

                if (autoHelper.atTargetPosition()) {driveFinished = true;}
                if (autoHelper.armAtPosition()) {armFinished = true;}
                if (autoHelper.hasNote()) {intakeFinished = true;}
                if (driveFinished && armFinished && intakeFinished) {counter += 1; driveFinished = false; armFinished = false; intakeFinished = false;}
                break;

            case 3:
                autoHelper.angleArmToPosition(Constants.closeSpeakerPosition);
                autoHelper.driveToPosition(autoHelper.setDesiredPose(speakerPosX, speakerPosY, 0));

                if (autoHelper.atTargetPosition()) {driveFinished = true;}
                if (autoHelper.armAtPosition()) {armFinished = true;}
                if (driveFinished && armFinished) {counter += 1; driveFinished = false; armFinished = false;}
                break;

            case 4:
                autoHelper.shooter.shootNoteRoutine();
                
                if (!autoHelper.hasNote()) {shootFinished = true;}
                if (shootFinished) {counter += 1; shootFinished = false;}
                break;

            case 5:
                autoHelper.driveToPosition(autoHelper.setDesiredPose(Units.inchesToMeters(180), Units.inchesToMeters(211), counter));;
                
                if (autoHelper.atTargetPosition()) {driveFinished = true;}
                if (driveFinished) {counter += 1; driveFinished = false;}
                break;

            default:
                autoHelper.stop();
                break;
        }
    }

    public void autoProgramTest() {
        SmartDashboard.putNumber("auto",counter);
        switch (counter) {
            case 0:
                autoHelper.driveToPosition(autoHelper.setDesiredPose(1, 1, 0));

                if (autoHelper.atTargetPosition()) {driveFinished = true;}
                if (driveFinished) {counter += 1; driveFinished = false;}
                break;

            case 1:
                autoHelper.driveToPosition(autoHelper.setDesiredPose(-1, -1, 0));
                if (autoHelper.atTargetPosition()) {driveFinished = true;}  

                // autoHelper.shootNote();
                // if (!autoHelper.hasNote()) {shootFinished = true;}
                if (driveFinished /*&& shootFinished*/) {counter += 1; driveFinished = shootFinished = false;}
                break;

            case 2:
                autoHelper.driveToPosition(autoHelper.setDesiredPose(3, 2, 0));
                autoHelper.angleArmToPosition(68);
                
                if (autoHelper.atTargetPosition()) {driveFinished = true;}
                if (driveFinished) {counter += 1; driveFinished = false;}
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

            // case 4:
            //     if () {counter += 1;}
            //     break;
            // case 5:
            //     if () {counter += 1;}
            //     break;
            // case 6:
            //     if () {counter += 1;}
            //     break;

            default: autoHelper.stop();
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