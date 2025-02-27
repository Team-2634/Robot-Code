/**
 * Team 2634
 * 2/21/2025
 * Auto Code
 * The following auto code plays around with swerve drive
 */

package frc.robot;

import com.studica.frc.AHRS;

//import com.kauailabs.navx.frc.AHRS;

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
    
    public void autoTest(){

        if (timer.get() < 5 ){

            if (autoHelper.timerInterval_Auto(0, 1)){
                autoHelper.resetDriveEncoders();
                autoHelper.resetSteerEncoders();
                autoHelper.autoResetPIDs();
            }
            else if (autoHelper.timerInterval_Auto(1, 5)){
                autoHelper.driver.swerveDrive(0.25, 0, 0);
            }
            // else if (autoHelper.timerInterval_Auto(5.1, 7)){
            //     autoHelper.driver.swerveDrive(0.25, 0.25, 0);
            // }
            // else if (autoHelper.timerInterval_Auto(8, 15)){
            //     autoHelper.driver.swerveDrive(0.25, 0, 0);
            // }

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
            else if (autoHelper.timerInterval_Auto(1, 5)){
                autoHelper.driver.swerveDrive(0.25, 0, 0); //moves forward
            }
            else if (autoHelper.timerInterval_Auto(5.1, 10)){
                autoHelper.driver.swerveDrive(0.25, 0.25, 0);    
            }
        }
            
        else {
            autoHelper.driver.swerveDrive(0, 0, 0); //STOP
        }

    }
        
}
    
    


