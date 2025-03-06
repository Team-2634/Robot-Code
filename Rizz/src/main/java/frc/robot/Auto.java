/**
 * Team 2634
 * 2/21/2025
 * Auto Code
 * The following auto code plays around with swerve drive
 */

package frc.robot;

import com.studica.frc.AHRS;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.systems.Driver;
import frc.robot.systems.Elevator;

public class Auto {

    Timer timer;

    AutoHelper autoHelper;
    public Auto(Driver driver, AHRS navx, Timer timer, Elevator elevator) {
        this.autoHelper = new AutoHelper(driver, timer, elevator);
        this.timer = timer;
    }

    void restartTimer() {
        timer.reset();
        timer.start();    
    }
    
    // public void autoTest(){

    //     if (timer.get() < 8 ){

    //         if (autoHelper.timerInterval_Auto(0, 1)){
    //             autoHelper.resetDriveEncoders();
    //             autoHelper.resetSteerEncoders();
    //             autoHelper.autoResetPIDs();
    //         }
    //         else if (autoHelper.timerInterval_Auto(1,5)){
    //             autoHelper.driver.swerveDrive(0.10, 0, 0); 
    //             //autoHelper.autoDriveByDistance(1,1);
    //         }

    //         else if (autoHelper.timerInterval_Auto(5,8)){
    //             autoHelper.driver.swerveDrive(-0.10, 0.05, 0); 
    //         }

    //     }

    //     else {
    //         autoHelper.driver.swerveDrive(0, 0, 0); //STOP
    //     }

    // }

    public void autoMiddle() {

        if (timer.get() < 15) { //eventually add limelight target
    
            if (autoHelper.timerInterval_Auto(0, 1)) {

                autoHelper.resetDriveEncoders();
                autoHelper.resetSteerEncoders();
                autoHelper.resetElevatorEncoders();
                autoHelper.autoResetPIDs();

            } else if (autoHelper.timerInterval_Auto(1, 3)) {

                autoHelper.driver.swerveDrive(0, 0,  0.25); // moves forward to the reef
                //autoHelper.elevator.elevatorLift(0.05);
                

            } else if (autoHelper.timerInterval_Auto(3.1, 10)) {

                autoHelper.driver.swerveDrive(0, 0, 0);
    
                // Target angle (arm should move down to 45 degrees)
                // double targetAngle = 45;  
                // targetAngle = Math.max(0, Math.min(targetAngle, 90)); 
                // autoHelper.elevator.armAngle(targetAngle); 
    
            }
        } 
        else {
            autoHelper.driver.swerveDrive(0, 0, 0); // STOP
        }

    }

    public void autoLeft() {
        if (timer.get() < 15) {
            
            if (autoHelper.timerInterval_Auto(0, 1)) {
                autoHelper.resetDriveEncoders();
                autoHelper.resetSteerEncoders();
                autoHelper.autoResetPIDs();
            }
            else if (autoHelper.timerInterval_Auto(1, 2)) {
                autoHelper.driver.swerveDrive(0.25, 0, 0); // Drive Forward
            }
            else if (autoHelper.timerInterval_Auto(2, 3)) {
                autoHelper.driver.swerveDrive(0, 0, 0.25); // Rotate towards Reef
            }
            else if (autoHelper.timerInterval_Auto(3, 4)) {
                autoHelper.driver.swerveDrive(0.10, 0, 0); // Drive towards reef, while elevator lifts up
                autoHelper.elevator.elevatorLift(0.05);
            }
            else if (autoHelper.timerInterval_Auto(4, 6)) {
                autoHelper.driver.swerveDrive(0, 0, 0); 
                // Stops At reef, Arm Code Goes here
                // Release Coral
            }
            else if (autoHelper.timerInterval_Auto(6, 8)) {
                autoHelper.driver.swerveDrive(0, 0, 0);
            }
            
        } 
        else {
            autoHelper.driver.swerveDrive(0, 0, 0); // STOP
        }
    }

    public void autoRight(){

        if (timer.get() < 15) {
            
            if (autoHelper.timerInterval_Auto(0, 1)) {
                autoHelper.resetDriveEncoders();
                autoHelper.resetSteerEncoders();
                autoHelper.autoResetPIDs();
                
            }
            else if (autoHelper.timerInterval_Auto(1, 2)) {
                autoHelper.driver.swerveDrive(0.10, 0, 0); // Drive Forward
            }
            else if (autoHelper.timerInterval_Auto(2, 3)) {
                autoHelper.driver.swerveDrive(0, 0, -0.20); // Rotate towards Reef
            }
            else if (autoHelper.timerInterval_Auto(3, 4)) {
                autoHelper.driver.swerveDrive(0.10, 0, 0); // Drive towards reef, while elevator lifts up
                autoHelper.elevator.elevatorLift(0.05);
                //autoHelper.elevator.moveToL4();
            }
            else if (autoHelper.timerInterval_Auto(4, 6)) {
                autoHelper.driver.swerveDrive(0, 0, 0); 
                // Stops At reef, Arm Code Goes here
                // Release Coral
            }
            else if (autoHelper.timerInterval_Auto(6, 8)) {
                autoHelper.driver.swerveDrive(0, 0, 0);
            }
            
        } 
        else {
            autoHelper.driver.swerveDrive(0, 0, 0); // STOP
        }

    }

    // int counter = 0;

    // public void autoProgramTest() {
    //     SmartDashboard.putNumber("auto",counter);
    //     switch (counter) {
    //         case 0:
    //             autoHelper.driveToPosition(autoHelper.setDesiredPose(1, 0, 0));

    //             if (autoHelper.atTargetPosition()) {driveFinished = true;}
    //             if (driveFinished) {counter += 1; driveFinished = false;}
    //             break;

    //         case 1:
    //             autoHelper.driveToPosition(autoHelper.setDesiredPose(1, 0, Math.PI));
    //             if (autoHelper.atTargetPosition()) {driveFinished = true;}

    //         default: autoHelper.stopDrive();
    //             break;
    //     }
    // }
}