package frc.robot;

import com.studica.frc.AHRS; 

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.systems.Climber;
import frc.robot.systems.Driver;
import frc.robot.systems.Elevator;
import frc.robot.systems.Arm;


public class TeleopHelper {
    
    Driver driver;
    Climber climber;
    AHRS navx;
    Elevator elevator;
    Arm arm;
    public TeleopHelper(Driver driver, Climber climber, AHRS navx, Elevator elevator, Arm arm) {
        this.driver = driver;
        this.climber = climber;
        this.navx = navx;
        this.elevator = elevator;
        this.arm = arm;
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

    public void moveElevator(XboxController xbox1) {
        if (xbox1.getRightTriggerAxis() > 0.2) {
            elevator.elevatorLift(0.35); // Moves up
        } 
        else if (xbox1.getLeftTriggerAxis() > 0.2) {
            elevator.elevatorLift(-0.35); // Moves down
        } 
        else {
            elevator.elevatorLift(0); // Stops elevator
        }
    }

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
    
    public void moveClamp(XboxController xbox) {

        if (xbox.getLeftBumperButtonPressed()) {
            arm.closeClaw();
            arm.closeClaw();
        } 

        if (xbox.getRightBumperButtonPressed()) {  
            arm.openClaw();
        
        }
    }

    public void moveArm(XboxController xbox) {

        // if (xbox.getAButtonPressed() && arm.isHardStoppedHigh() && arm.isHardStoppedLow()) {
        //     arm.moveArmPID(Constants.armLowPosition);
        // } else if (xbox.getBButtonPressed() && arm.isHardStoppedHigh() && arm.isHardStoppedLow()) {
        //     arm.moveArmPID(Constants.arm60);
        // } else if (xbox.getXButtonPressed() && arm.isHardStoppedHigh() && arm.isHardStoppedLow()) {
        //     arm.moveArmPID(Constants.arm35);
        // } else if (xbox.getYButtonPressed() && arm.isHardStoppedHigh() && arm.isHardStoppedLow()) {
        //     arm.moveArmPID(Constants.armIntake);
        // } else {
        //     arm.moveArm(0);
        // }
    }

    double speed = 0;

    public void armTestTest(XboxController xbox) {
        if (xbox.getXButton()) {
            speed = -0.1;
        } else {
            speed = 0;
        }

        SmartDashboard.putNumber("arm speed", speed);
        arm.moveArm(speed);

    }

 }