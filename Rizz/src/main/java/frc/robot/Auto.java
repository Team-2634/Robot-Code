/**
 * Team 2634
 * 2/21/2025
 * Auto Code
 * The following auto code plays around with swerve drive
 */

package frc.robot;

import com.studica.frc.AHRS;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    boolean delayFinished = false;
    boolean driveFinished = false;
    boolean armFinished = false;
    boolean elevatorFinished = false;
    boolean clawFinished = false;
    int counter = 0;

    public void resetFlags() {
        delayFinished = false; 
        armFinished = false; 
        elevatorFinished = false; 
        clawFinished = false; 
        driveFinished = false;
    }

    public void autoDriveByDistanceTest() {
        switch (counter) {
            case 0:
                autoHelper.resetDriveEncoders();
                autoHelper.resetSteerEncoders();
                autoHelper.autoResetPIDs();
                restartTimer();
                counter++;
                break;
    
            case 1:

                autoHelper.driver.driveToPosition(autoHelper.driver.setDesiredPose(1, 0, 0)); // Move forward 1 meter  
                if (autoHelper.driver.atTargetPosition()) {driveFinished = true;} SmartDashboard.putBoolean("driveFinished", driveFinished);
                if (driveFinished) {counter += 1; driveFinished = false;}
                break;
    
            case 2:
                autoHelper.driver.swerveDrive(0, 0, 0); // Stop movement
                break;
        }
    }

}
    
// public void autoMiddle(){ // 1-coral Auto (on L1) without Limelight

//     if (timer.get() < 15){ 

//         if (autoHelper.timerInterval_Auto(0, 0.15)){
//             autoHelper.resetDriveEncoders();
//             autoHelper.resetSteerEncoders();
//             autoHelper.autoResetPIDs();
//         }
//         else if (autoHelper.timerInterval_Auto(0.16, 1.16)){ 
//             autoHelper.autoCloseClaw();
//             autoHelper.driver.swerveDrive(0, 0, 0);
//             autoHelper.autoArmLift(-0.255);
//         }
//         else if (autoHelper.timerInterval_Auto(1.17, 3.17)){
//             autoHelper.autoArmLift(0);
//             autoHelper.autoElevatorLift(0.45);
//             autoHelper.driver.swerveDrive(0.24, 0, 0);
//         }
//         else if (autoHelper.timerInterval_Auto(3.18, 3.21)){
//             autoHelper.autoElevatorLift(0);
//             autoHelper.driver.swerveDrive(0, 0, 0);
//         }
//         else if (autoHelper.timerInterval_Auto(3.22, 3.9)){
//             autoHelper.autoOpenClaw();
//         }
//         else if (autoHelper.timerInterval_Auto(4, 5)){
//             autoHelper.autoArmLift(0.25);
//         }
//         else if (autoHelper.timerInterval_Auto(5.1, 5.9)){
//             autoHelper.autoArmLift(0);
//             autoHelper.driver.swerveDrive(-0.10, 0, 0);
//         }
//         else if (autoHelper.timerInterval_Auto(6, 9)){
//             autoHelper.driver.swerveDrive(0, 0, 0);
//         }
                
//     }   

// }