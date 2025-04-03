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
import java.util.concurrent.TimeUnit;

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
    double matchTime = 0;
    boolean firstTime = true;
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
                autoHelper.driver.driveToPosition(autoHelper.driver.setDesiredPose(2.2, 0, 0)); // Move forward 1 meter  
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

                autoHelper.driver.driveToAprilTag(true); 

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
                
                if (autoHelper.elevator.atTargetElevatorPositionInTake()) {
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
    


    //Framework for Middle Auto Blue
    
    public void autoSidesBlueAllianceMiddle(){ //One Piece Auto on L4

        /**
         * Auto Pseudocode //still need to implement Limelight into the driveToPosition function
         * 
         * Claw Closes
         * Arm Down 
         * Drive Forward (Align with Limelight), while elevator lifts to L4
         * Align Arm to L4
         * 
         * Claw Opens
         * Drive to Coral Station
         */

        switch(counter) {
            case 0: //First Step of Auto: Claw Closes and Arm Lifts Down(This Part of the code stays the same, it is universal.)
                autoHelper.resetDriveEncoders();
                autoHelper.resetSteerEncoders();
                autoHelper.autoResetPIDs();
                restartTimer();

                if (clawFinished == false){
                    autoHelper.autoCloseClaw();
                    clawFinished = true;
                }
                
                if (armFinished == false){
                    autoHelper.autoArmLift(-0.255); //Will get jammed if starts below 0.
                }

                if (autoHelper.arm.atTargetArmPositionL0()){
                    autoHelper.autoArmLift(0);
                    armFinished = true;
                }

                if (clawFinished && armFinished) {
                    finishCheckAll(); 
                } 

                break;
        
            case 1: //Second Step of Auto: Drives 82 in(2.083 M) Forward, Move elevator up to L4, Move arm up to L4
                System.out.println("Case 1");
                
                autoHelper.driver.driveToPosition(autoHelper.driver.setDesiredPose(2.083, 0, 0)); // Moved forward 2.083m

                if (autoHelper.driver.atTargetPosition()) {
                    autoHelper.driver.swerveDrive(0, 0, 0);
                    driveFinished = true;
                }

                if (elevatorFinished == false) { 
                    autoHelper.autoElevatorLift(0.5); 
                }                
                if (autoHelper.elevator.atTargetElevatorPositionL4()) {
                    autoHelper.autoElevatorLift(0);
                    elevatorFinished = true;
                }


                if (armFinished == false){
                    autoHelper.autoArmLift(0.5);
                }
                if (autoHelper.arm.atTargetArmPositionL4()){
                    autoHelper.autoArmLift(0);
                    armFinished = true;
                }
    
                if (driveFinished && elevatorFinished && armFinished) {
                    finishCheckAll(); 
                } 

                break;
    
            case 2: //Third step of Auto: Read april tag, use limelight to adjust accordingly.
                System.out.println("Case 2");
                autoHelper.driver.driveToAprilTag(true);

                
                if (autoHelper.driver.atTargetPosition()) {
                    autoHelper.driver.swerveDrive(0, 0, 0);
                    driveFinished = true;
                }

                if (driveFinished) {
                    finishCheckAll();
                }

                break;
            
            case 3: //Fourth step of auto: Open claw, dropping coral, retract all elevator and arm.
                System.out.println("Case 3"); 

                if (clawFinished == false){
                    autoHelper.autoOpenClaw();
                    clawFinished = true;
                }

                if (elevatorFinished == false) { 
                    autoHelper.autoElevatorLift(-0.5); 
                }                
                if (autoHelper.elevator.atTargetElevatorPositionInTake()) {
                    autoHelper.autoElevatorLift(0);
                    elevatorFinished = true;
                }
                
                if (armFinished == false) {
                    autoHelper.autoArmLift(-0.5);
                }
                if (autoHelper.arm.atTargetArmPositionIntake()) {
                    autoHelper.autoArmLift(0);
                    armFinished = true;
                }

                if (clawFinished && elevatorFinished && armFinished) {
                    finishCheckAll(); 
                }            
                break;
            
            case 4: //Fifth Step of Auto: Move right 92 inches, move forward 196 inches, rotate at -180*
                System.out.println("Case 4");

                autoHelper.driver.driveToPosition(autoHelper.driver.setDesiredPose(4.978, 2.3368, -180));

                if (autoHelper.driver.atTargetPosition()) {
                    autoHelper.driver.swerveDrive(0, 0, 0);
                    driveFinished = true;
                }

                if (elevatorFinished == false) { 
                    autoHelper.autoElevatorLift(0.5); 
                }                
                if (autoHelper.elevator.atTargetElevatorPositionL4()) {
                    autoHelper.autoElevatorLift(0);
                    elevatorFinished = true;
                }

                if (driveFinished && elevatorFinished) {
                    finishCheckAll();
                }

                break;

            case 5: //Read april tag and drive to the feeder.
                System.out.println("Case 5");

                //autoHelper.driver.driveToAprilTag(Constants.feederDistanceOffset1st, true); {Save this for Later, for now exact measurements.}
                
                /** This case is a nothing burger for now, if we get limelight working for this, will explore back. */

                finishCheckAll();

                break;
                
            
            case 6: //Seventh step of Auto: Receive second piece of coral from feeder, close claw
                System.out.println("Case 6");
                
                if (!clawFinished){

                    if (firstTime) {
                        matchTime = Timer.getMatchTime(); //first iteration should set the time to match time.
                        firstTime = false;
                    }
                    
                    if(Timer.getMatchTime() > matchTime + 2) {
                        autoHelper.autoCloseClaw();
                        clawFinished = true;
                    }
                }
                    //1. On the first iteration of loop , get the match time(How long the code is running). Set the match time to a variable. Constantly check the time each loop

                if (clawFinished) {
                    finishCheckAll();
                }

                break;
            
            case 7: //Eight step of auto: Move forward 90 inches, lift arm and elevator to L4
                System.out.println("Case 7");
        
                autoHelper.driver.driveToPosition(autoHelper.driver.setDesiredPose(2.286, 0, 0));

                if (autoHelper.driver.atTargetPosition()) {
                    autoHelper.driver.swerveDrive(0, 0, 0);
                    driveFinished = true;
                }

                if (armFinished == false){
                    autoHelper.autoArmLift(0.5);
                }
                if (autoHelper.arm.atTargetArmPositionL4()){
                    autoHelper.autoArmLift(0);
                    armFinished = true;
                }

                if (armFinished && driveFinished) {
                    finishCheckAll();
                }

                break;

            case 8: //9th step of code: Read April Tag Again to adjust for reef but for the 2nd time.

            System.out.println("Case 8");

                autoHelper.driver.driveToAprilTag(true);
                
                if (autoHelper.driver.atTargetPosition()) {
                    autoHelper.driver.swerveDrive(0, 0, 0);
                    driveFinished = true;
                }

                if (driveFinished) {
                    finishCheckAll();
                }

                break;
            
            case 9: //10th step of auto: Open claw, dropping coral.
                System.out.println("Case 9");

                if (clawFinished == false){
                    autoHelper.autoOpenClaw();
                    clawFinished = true;
                }

                if (elevatorFinished == false) { 
                    autoHelper.autoElevatorLift(-0.5); 
                }
                
                if (clawFinished && elevatorFinished) {
                    finishCheckAll(); 
                }   
                //TODO move elevator + arm down 1 case
                
                break;

            case 10: //11th Step of Auto: Lower Elevator & Arm, Move 96 inches backwards while opening claw to go to feeder
                System.out.println("Case 10");

                autoHelper.driver.driveToPosition(autoHelper.driver.setDesiredPose(-2.438, 0, 0));
                
                if (autoHelper.driver.atTargetPosition()) {
                    autoHelper.driver.swerveDrive(0, 0, 0);
                    driveFinished = true;
                }

                if (autoHelper.elevator.atTargetElevatorPositionInTake()) {
                    autoHelper.autoElevatorLift(0);
                    elevatorFinished = true;
                }
                
                if (armFinished == false) {
                    autoHelper.autoArmLift(-0.5);
                }

                if (autoHelper.arm.atTargetArmPositionIntake()) {
                    autoHelper.autoArmLift(0);
                    armFinished = true;
                }         

                if (clawFinished == false) {
                    autoHelper.autoOpenClaw();
                    clawFinished = true;
                }

                if (driveFinished && clawFinished && elevatorFinished && armFinished) {
                    finishCheckAll();
                }

                break;

            case 11: //12th Step of Auto: Receive second piece of coral from feeder, close claw
                System.out.println("Case 11");

                if (!clawFinished){

                    if (firstTime) {
                        matchTime = Timer.getMatchTime(); //first iteration should set the time to match time.
                        firstTime = false;
                    }
                    
                    if(Timer.getMatchTime() > matchTime + 2) {
                        autoHelper.autoCloseClaw();
                        clawFinished = true;
                    }
                }

                if (clawFinished) {
                    finishCheckAll();
                }

                break;

            case 12: //13th Step of Auto: Move forward 90 inches, lift arm and elevator to L4 AGAIN

                System.out.println("Case 12");
        
                autoHelper.driver.driveToPosition(autoHelper.driver.setDesiredPose(2.286, 0, 0));

                if (autoHelper.driver.atTargetPosition()) {
                    autoHelper.driver.swerveDrive(0, 0, 0);
                    driveFinished = true;
                }

                if (armFinished == false){
                    autoHelper.autoArmLift(0.5);
                }
                if (autoHelper.arm.atTargetArmPositionL4()){
                    autoHelper.autoArmLift(0);
                    armFinished = true;
                }

                if (elevatorFinished == false) { 
                    autoHelper.autoElevatorLift(0.5); 
                }                
                if (autoHelper.elevator.atTargetElevatorPositionL4()) {
                    autoHelper.autoElevatorLift(0);
                    elevatorFinished = true;
                }

                if (elevatorFinished && armFinished && driveFinished) {
                    finishCheckAll();
                }

                break;

            case 13: //14th Step of Auto: Scan and Scan some more
                System.out.println("Case 13");

                autoHelper.driver.driveToAprilTag(true);

                if (autoHelper.driver.atTargetPosition()) {
                    autoHelper.driver.swerveDrive(0, 0, 0);
                    driveFinished = true;
                }

                if (driveFinished) {
                    finishCheckAll();
                }

                break;

            case 14: //15th Step of Auto: Opening claw, bringing elevator and arm back down
                System.out.println("Case 14");

                if (clawFinished == false){
                    autoHelper.autoOpenClaw();
                    clawFinished = true;
                }

                if (elevatorFinished == false) { 
                    autoHelper.autoElevatorLift(-0.5); 
                }            
                if (autoHelper.elevator.atTargetElevatorPositionInTake()) {
                    autoHelper.autoElevatorLift(0);
                    elevatorFinished = true;
                }
                
                if (armFinished == false) {
                    autoHelper.autoArmLift(-0.5);
                }
                if (autoHelper.arm.atTargetArmPositionIntake()) {
                    autoHelper.autoArmLift(0);
                    armFinished = true;
                }

                if (clawFinished && elevatorFinished && armFinished) {
                    finishCheckAll(); 
                }            
                break;

            case 15: //16th and final step of Auto: Move robot 96 inches backwards

                autoHelper.driver.driveToPosition(autoHelper.driver.setDesiredPose(-2.438, 0, 0));
                
                if (autoHelper.driver.atTargetPosition()) {
                    autoHelper.driver.swerveDrive(0, 0, 0);
                    driveFinished = true;
                }

                if (clawFinished == false) {
                    autoHelper.autoOpenClaw();
                    clawFinished = true;
                }

                if (driveFinished && clawFinished) {
                    finishCheckAll();
                }
                
                break;

            default:
                autoHelper.driver.swerveDrive(0, 0, 0);
                autoHelper.autoArmLift(0);
                autoHelper.autoElevatorLift(0);
                break;

        }
    }

    public void finishCheckAll (){
          
            counter++;
            driveFinished = false;
            elevatorFinished = false;
            clawFinished = false;
            armFinished = false;
            firstTime = true;
    
    }
    
    public void autoMiddleDriveByPosition(){

        switch(counter){

            case 0: 
                autoHelper.resetDriveEncoders();
                autoHelper.resetSteerEncoders();
                autoHelper.autoResetPIDs();
                restartTimer();

                if (clawFinished = false){
                    autoHelper.autoCloseClaw();
                    clawFinished = true;
                }
                
                if (armFinished = false){
                    autoHelper.autoArmLift(-0.3);
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

            case 1: 
                System.out.println("Case 1");
                autoHelper.driver.driveToPosition(autoHelper.driver.setDesiredPose(2.54, 0, 0)); // Moved forward 1.35m

                if (autoHelper.driver.atTargetPosition()) {
                    autoHelper.driver.swerveDrive(0, 0, 0);
                    driveFinished = true;
                }

                if (elevatorFinished = false) { 
                    autoHelper.autoElevatorLift(-0.5); 
                }
                
                if (autoHelper.elevator.atTargetElevatorPositionL1()) {
                    autoHelper.autoElevatorLift(0);
                    elevatorFinished = true;
                }
    
                if (driveFinished && elevatorFinished) {
                    counter++;
                    driveFinished = false;
                    elevatorFinished = false;
                }
                break;

            case 2: 
                System.out.println("Case 2");
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

                if (clawFinished) {
                    counter++;
                    clawFinished = false;
                }
                break;

            default:
                autoHelper.driver.swerveDrive(0, 0, 0);
                autoHelper.autoArmLift(0);
                autoHelper.autoElevatorLift(0);
                break;
            
            }
        }

        public void autoSidesDriveByPosition(){

            switch(counter){
    
                case 0: 
                    autoHelper.resetDriveEncoders();
                    autoHelper.resetSteerEncoders();
                    autoHelper.autoResetPIDs();
                    restartTimer();
    
                    if (clawFinished = false){
                        autoHelper.autoCloseClaw();
                        clawFinished = true;
                    }
                    
                    if (armFinished = false){
                        autoHelper.autoArmLift(-0.3);
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
    
                case 1: 
                    System.out.println("Case 1");
                    autoHelper.driver.driveToPosition(autoHelper.driver.setDesiredPose(3.54, 0, 0)); // Moved forward 1.35m
    
                    if (autoHelper.driver.atTargetPosition()) {
                        autoHelper.driver.swerveDrive(0, 0, 0);
                        driveFinished = true;
                    }
    
                    if (elevatorFinished = false) { 
                        autoHelper.autoElevatorLift(-0.5); 
                    }
                    
                    if (autoHelper.elevator.atTargetElevatorPositionL1()) {
                        autoHelper.autoElevatorLift(0);
                        elevatorFinished = true;
                    }
        
                    if (driveFinished && elevatorFinished) {
                        counter++;
                        driveFinished = false;
                        elevatorFinished = false;
                    }
                    break;
    
                case 2: 
                    System.out.println("Case 2");
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
    
                    if (clawFinished) {
                        counter++;
                        clawFinished = false;
                    }
                    break;
    
                default:
                    autoHelper.driver.swerveDrive(0, 0, 0);
                    autoHelper.autoArmLift(0);
                    autoHelper.autoElevatorLift(0);
                    break;
                
                }
            }

    public void autoL4Test(){

        if (timer.get() < 15){

            if (autoHelper.timerInterval_Auto(0, 0.25)){
                autoHelper.resetDriveEncoders();
                autoHelper.resetSteerEncoders();
                autoHelper.autoResetPIDs();
                autoHelper.autoCloseClaw();
            }

            else if (autoHelper.timerInterval_Auto(0.25, 0.99)){
                autoHelper.autoArmLift(-0.5);
            }

            else if (autoHelper.timerInterval_Auto(1, 5.91)){
                autoHelper.autoArmLift(0);
                armFinished = true;
                          
                if (armFinished){
                    autoHelper.driver.driveToPosition(autoHelper.driver.setDesiredPose(2.2, 0, 0));

                    if (autoHelper.driver.atTargetPosition()){
                        autoHelper.driver.swerveDrive(0, 0, 0);
                        driveFinished = true;
                    }
                
                    if (driveFinished){
                        autoHelper.autoElevatorLift(-0.85); 
                    }
                }

            }

            else if (autoHelper.timerInterval_Auto(5.92, 10.92)){

                if (autoHelper.timerInterval_Auto(5.92, 7.92)){
                    autoHelper.autoElevatorLift(-0.85);
                    elevatorFinished = true;
                }

                if (elevatorFinished){
                    autoHelper.autoElevatorLift(0);
                    autoHelper.autoArmLift(-0.5);
                    armFinished = true;
                }

                if (armFinished){
                    autoHelper.autoOpenClaw();
                    driveFinished = false;
                    elevatorFinished = false;
                    armFinished = false;
                }


            }
            
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
    //             autoHelper.autoArmLift(-0.26);
    //         }
    //         else if (autoHelper.timerInterval_Auto(1.17, 3.17)){
    //             autoHelper.autoArmLift(0);
    //             autoHelper.autoElevatorLift(-0.50);
    //             autoHelper.driver.swerveDrive(0.302, 0, 0);
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
    