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
import frc.robot.systems.LimeLight;
import frc.robot.systems.Arm;

public class Auto {

    Timer timer;

    AutoHelper autoHelper;
    public Auto(Driver driver, Climber climber, AHRS navx, Timer timer, Elevator elevator, Arm arm, LimeLight limelight) {
        this.autoHelper = new AutoHelper(driver, climber, navx, timer, elevator, arm, limelight);
        this.timer = timer;
    }

    void restartTimer() {
        timer.reset();
        timer.start();    
    }
    
    // public void autoMiddle(){
         
    //     if (timer.get() < 15 ){

    //         if (autoHelper.timerInterval_Auto(0, 1)){
    //             autoHelper.resetDriveEncoders();
    //             autoHelper.resetSteerEncoders();
    //             autoHelper.autoResetPIDs();
    //         }
    //         else if (autoHelper.timerInterval_Auto(1, 3)){
    //             autoHelper.driver.swerveDrive(0.25, 0, 0); //moves forward to the reef
    //         }
    //         else if (autoHelper.timerInterval_Auto(4.1, 10)){
    //             autoHelper.driver.swerveDrive(0, 0, 0);
    //             /* 
    //             Arm Code here: Set Arm @ Correct Angle, Release Coral into L3, Take off an Algae
    //              */ 
    //         }
    //     }
            
    //     else {
    //         autoHelper.driver.swerveDrive(0, 0, 0); //STOP
    //     }

    // }

        public void autoLeftBlueAlliance(){

            if (timer.get() < 15){ 

                if (autoHelper.timerInterval_Auto(0, 0.25)){
                    autoHelper.resetDriveEncoders();
                    autoHelper.resetSteerEncoders();
                    autoHelper.autoResetPIDs();
                }
                else if (autoHelper.timerInterval_Auto(0.26, 1.26)){  //move forward
                    autoHelper.autoCloseClaw();
                  
                    autoHelper.driver.swerveDrive(0, 0, 0);
                    autoHelper.autoArmLift(-0.25);
                }
                else if (autoHelper.timerInterval_Auto(1.27, 4.27)){
                    autoHelper.autoArmLift(0);
                    autoHelper.autoElevatorLift(0.50);
                    autoHelper.driver.swerveDrive(0.25, 0, 0);

                }
                else if (autoHelper.timerInterval_Auto(4.28, 5.28)){
                    autoHelper.autoElevatorLift(0);
                    autoHelper.driver.swerveDrive(0, 0, 0);

                }
                else if (autoHelper.timerInterval_Auto(5.29, 6.29)){
                    autoHelper.autoOpenClaw();
                }
               
             
            }   
        }
   
}