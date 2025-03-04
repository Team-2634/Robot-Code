/**
 * Team 2634
 * 2/21/2025
 * Auto Code
 * The following auto code plays around with swerve drive
 */

package frc.robot;

import com.studica.frc.AHRS;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.systems.Climber;
import frc.robot.systems.Driver;
import frc.robot.systems.Elevator;
public class Auto {

    Timer timer;

    AutoHelper autoHelper;
    public Auto(Driver driver, Climber climber, AHRS navx, Timer timer, Elevator elevator) {
        this.autoHelper = new AutoHelper(driver, climber, navx, timer);
        this.timer = timer;
    }

    void restartTimer() {
        timer.reset();
        timer.start();    
    }
    
    public void autoTest(){

        if (timer.get() < 8 ){

            if (autoHelper.timerInterval_Auto(0, 1)){
                autoHelper.resetDriveEncoders();
                autoHelper.resetSteerEncoders();
                autoHelper.autoResetPIDs();
            }
            else if (autoHelper.timerInterval_Auto(1,5)){
                autoHelper.driver.swerveDrive(0.10, 0, 0); 
            }

            else if (autoHelper.timerInterval_Auto(5,8)){
                autoHelper.driver.swerveDrive(-0.10, 0.05, 0); 
            }

        }

        else {
            autoHelper.driver.swerveDrive(0, 0, 0); //STOP
        }

    }

    public void autoMiddle(){
         
        if (timer.get() < 15 ){

            if (autoHelper.timerInterval_Auto(0, 1)){
                autoHelper.resetDriveEncoders();
                autoHelper.resetSteerEncoders();
                autoHelper.autoResetPIDs();
            }
            else if (autoHelper.timerInterval_Auto(1, 3)){
                autoHelper.driver.swerveDrive(0.25, 0, 0); //moves forward to the reef
            }
            else if (autoHelper.timerInterval_Auto(4.1, 10)){
                autoHelper.driver.swerveDrive(0, 0, 0);
                /* 
                Arm Code here: Extend Arm, Release Coral into L4, Take off an Algae
                 */ 
            }
        }
            
        else {
            autoHelper.driver.swerveDrive(0, 0, 0); //STOP
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
                autoHelper.driver.swerveDrive(0.4, 0, 0); // Drive Forward
            }
            else if (autoHelper.timerInterval_Auto(2, 3)) {
                autoHelper.driver.swerveDrive(0, 0, 0.20); // Rotate towards Reef
            }
            else if (autoHelper.timerInterval_Auto(3, 4)) {
                autoHelper.driver.swerveDrive(0.10, 0, 0); // Drive towards reef
                //Elevator and Arm Both Lift Up
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

    
}