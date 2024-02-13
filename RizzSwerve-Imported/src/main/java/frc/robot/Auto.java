package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.systems.Climber;
import frc.robot.systems.Driver;
import frc.robot.systems.Shooter;

public class Auto {

    Timer timer;

    AutoHelper autoHelper;
    public Auto(Driver driver, Shooter shooter, Climber climber, AHRS navx, Timer timer) {
        this.autoHelper = new AutoHelper(driver, shooter, climber, navx, timer);
        this.timer = timer;
    }

    void restartTimer() {
        timer.reset();
        timer.start();    
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
    
    int counter = 0;
    public void autoProgramTest() {
        switch (counter) {
            case 0:
                if (autoHelper.driveToPosition(autoHelper.setDesiredPose(1, 1, Math.PI/2))) {counter += 1;}
                break;
            case 1:
                if (autoHelper.driveToPosition(autoHelper.setDesiredPose(0, 0, 0))) {counter += 1;}
                break;
            case 2:
                if (autoHelper.driveToPosition(autoHelper.setDesiredPose(0, 0, 0))) {counter += 1;}
                break;
            case 3:
                if (autoHelper.driveToPosition(autoHelper.setDesiredPose(0, 0, 0))) {counter += 1;}
                break;
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




}
