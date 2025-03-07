package frc.robot;

import com.studica.frc.AHRS; 

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.systems.Climber;
import frc.robot.systems.Driver;
import frc.robot.systems.Elevator;


public class TeleopHelper {
    
    Driver driver;
    Climber climber;
    AHRS navx;
    Elevator elevator;
    public TeleopHelper(Driver driver, Climber climber, AHRS navx, Elevator elevator) {
        this.driver = driver;
        this.climber = climber;
        this.navx = navx;
        this.elevator = elevator;
    }

    /**
     * Drives given speed with options
     * @param XSpeed double, From -1 to 1 
     * @param YSpeed double, From -1 to 1
     * @param TurnSpeed double, From -1 to 1
     * @param disableFieldOrient Boolean, toggle field oriented controls
     */

    public void drive(double XSpeed, double YSpeed, double TurnSpeed, boolean disableFieldOrient) {

        if (!disableFieldOrient) {
            double[] speedsFieldOriented = Driver.fieldOrient(XSpeed, YSpeed, navx);
            XSpeed = speedsFieldOriented[0];
            YSpeed = speedsFieldOriented[1];
        }
        
        XSpeed *= Constants.XdriveSensitivity;
        YSpeed *= Constants.YdriveSensitivity;
        TurnSpeed = TurnSpeed * Constants.turningSensitivity;
        
        driver.swerveDrive(XSpeed, YSpeed, TurnSpeed);
    }

        //  public void liftElevatorFromBumper(boolean input){
//      if(input) {
//          elevator.elevatorLiftUp(0.05);
//      }
//      else if(input != true) {
//          elevator.elevatorLiftUp(0); //just incase it doesnt stop after you press Left bumper
//      }

//  }

    // public void liftElevatorFromBumper() {
    //     if (xbox1.getRightTriggerAxis() > 0.2) {
    //         elevator.elevatorLift(0.05); // Moves up
    //     } 
    //     else if (xbox1.getRightTriggerAxis() > 0.2) {
    //         elevator.elevatorLift(-0.05); // Moves down
    //     } 
    //     else {
    //         elevator.elevatorLift(0); // Stops elevator
    //     }
    // }

    private int currentLevel = 1;  // Start at L1
    private boolean rtPressed = false;
    private boolean ltPressed = false;

    public void elevatorControl(double rightTrigger, double leftTrigger) {
        double rtValue = rightTrigger;
        double ltValue = leftTrigger;
    
        
        if (rtValue > 0.5 && !rtPressed) { 
            if (currentLevel < 4) { // Max level is L4
                currentLevel++;
                moveToCurrentLevel();
            }
            rtPressed = true; 
        } else if (rtValue < 0.2) {
            rtPressed = false; 
        }
    
        if (ltValue > 0.5 && !ltPressed) {
            if (currentLevel > 1) { // Min level is L1
                currentLevel--;
                moveToCurrentLevel();
            }
            ltPressed = true;
        } else if (ltValue < 0.2) {
            ltPressed = false;
        }
    }

    // Moves elevator based on current level
    private void moveToCurrentLevel() {
        switch (currentLevel) {
            case 1: elevator.moveToL1(); break;
            case 2: elevator.moveToL2(); break;
            case 3: elevator.moveToL3(); break;
            case 4: elevator.moveToL4(); break;
        }
    }
    
    public void moveArm(XboxController xbox) {
    
        if (xbox.getAButtonPressed()) { 
            elevator.armAngle(35); // Moves arm to 35° downward
        } 
        else if (xbox.getBButtonPressed()) { 
            elevator.armAngle(0);  // Moves arm back up to 0°
        }

        if (xbox.getLeftBumperButtonPressed()) {
            elevator.closeClaw();
        } 

        if (xbox.getRightBumperButtonPressed()) {  
            elevator.openClaw();
        
        }
    }
 }
