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
                System.out.println("Case 1");
                autoHelper.driver.driveToPosition(autoHelper.driver.setDesiredPose(2, 0, 0)); // Move forward 1 meter  
                if (autoHelper.driver.atTargetPosition()) {driveFinished = true;} SmartDashboard.putBoolean("driveFinished", driveFinished);
                if (driveFinished) {counter += 1; driveFinished = false;}
                break;
    
            case 2:
                System.out.println("Case 2");
                autoHelper.driver.swerveDrive(0, 0, 0); // Stop movement
                break;
        }
    }

    public void autoLimelightTest(){

        switch(counter){
            case 0:
                autoHelper.resetDriveEncoders();
                autoHelper.resetSteerEncoders();
                autoHelper.autoResetPIDs();
                restartTimer();
                counter++;
                break;
            
            case 1: 
                System.out.println("Case 1");
                
                //autoHelper.driver.rotateToAprilTag(); // Align with AprilTag before driving - Irrelevant since drive to april tag should fully align now

                autoHelper.driver.driveToAprilTag(0.5); 

                if (autoHelper.driver.atTargetPosition()) {
                    autoHelper.driver.swerveDrive(0, 0, 0); 
                    driveFinished = true;
                }

                if(driveFinished){
                    counter++;
                    driveFinished = false;
                    System.out.println("Auto Finished");
                    break;
                }

        }

    }

    public void autoSidesBlueAlliance(){ //One Piece Auto on L4

        /**
         * Auto Pseudocode //still need to implement Limelight into the driveToPosition function
         * 
         * Claw Closes
         * Arm Down 
         * Drive Forward (Align with Limelight), while elevator lifts to L4
         * Align Arm to L4
         * Claw Opens
         * Drive to Coral Station
         */

        switch(counter) {
            case 0: //First Step of Auto: Claw Closes and Arm Lifts Down
                autoHelper.resetDriveEncoders();
                autoHelper.resetSteerEncoders();
                autoHelper.autoResetPIDs();
                restartTimer();

                if (clawFinished = false){
                    autoHelper.autoCloseClaw();
                    clawFinished = true;
                }
                
                if (armFinished = false){
                    autoHelper.autoArmLift(-0.255);
                }

                if (autoHelper.arm.atTargetArmPositionL0()){
                    autoHelper.autoArmLift(0);
                    armFinished = true;
                }

                if (clawFinished && armFinished){
                    counter++;
                    clawFinished = false;
                    armFinished = false;
                }
                break;
        
            case 1: //Second Step of Auto: Drives towards the reef, while elevator lifts to L4 Height
                System.out.println("Case 1");
                autoHelper.driver.driveToPosition(autoHelper.driver.setDesiredPose(2.54, 0, 0)); // Moved forward 1.35m

                if (autoHelper.driver.atTargetPosition()) {
                    autoHelper.driver.swerveDrive(0, 0, 0);
                    driveFinished = true;
                }

                if (elevatorFinished = false) { 
                    autoHelper.autoElevatorLift(0.5); 
                }
                
                if (autoHelper.elevator.atTargetElevatorPositionL4()) {
                    autoHelper.autoElevatorLift(0);
                    elevatorFinished = true;
                }
    
                if (driveFinished && elevatorFinished) {
                    counter++;
                    driveFinished = false;
                    elevatorFinished = false;
                }
                break;
    
            case 2: //Third step of Auto: Arm Lifts Up to L4
                System.out.println("Case 2");

                if (armFinished = false){
                    autoHelper.autoArmLift(0.5);
                }

                if (autoHelper.arm.atTargetArmPositionL4()){
                    autoHelper.autoArmLift(0);
                    armFinished = true;
                }

                if (armFinished){
                    counter++;
                    armFinished = false;
                }
                break;
            
            case 3: //Fourth Step of Auto: Coral Drops Into L4, drives back "0.9 m"
                System.out.println("Case 3"); 

                if (clawFinished = false){
                    autoHelper.autoOpenClaw();
                    clawFinished = true;
                }

                if (clawFinished = true){
                    autoHelper.driver.driveToPosition(autoHelper.driver.setDesiredPose(-0.9, 0, 0));
                }

                if (autoHelper.driver.atTargetPosition()) {
                    autoHelper.driver.swerveDrive(0, 0, 0);
                    driveFinished = true;
                }   

                if (elevatorFinished = false) { 
                    autoHelper.autoElevatorLift(-0.5); 
                }
                
                if (autoHelper.elevator.atTargetElevatorPositionL0()) {
                    autoHelper.autoElevatorLift(0);
                    elevatorFinished = true;
                }


                if (driveFinished && elevatorFinished && clawFinished) {
                    counter++;
                    driveFinished = false;
                    elevatorFinished = false;
                    clawFinished = false;
                }
                break;
            
            case 4: //Fifth Step of Auto: Arm Returns to its Intake Position
                System.out.println("Case 4");
                if (armFinished = false){
                    autoHelper.autoArmLift(0.5);
                }

                if (autoHelper.arm.atTargetArmPositionIntake()){
                    autoHelper.autoArmLift(0);
                    armFinished = true;
                }

                break;

            default:
                autoHelper.driver.swerveDrive(0, 0, 0);
                autoHelper.autoArmLift(0);
                autoHelper.autoElevatorLift(0);
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