package frc.robot;

import com.studica.frc.AHRS; 

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.systems.Arm;
import frc.robot.systems.Climber;
import frc.robot.systems.Driver;
import frc.robot.systems.Elevator;
import frc.robot.systems.Arm;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.systems.LimeLight;


public class TeleopHelper {
    
    Driver driver;
    Climber climber;
    AHRS navx;
    Elevator elevator;
    Arm arm;
    LimeLight limelight;

    public TeleopHelper(Driver driver, Climber climber, AHRS navx, Elevator elevator, Arm arm, LimeLight limelight) {
        this.driver = driver;
        this.climber = climber;
        this.navx = navx;
        this.elevator = elevator;
        this.arm = arm;
        this.limelight = limelight;
    }

    /**
     * Drives given speed with options
     * @param XSpeed double, From -1 to 1 
     * @param YSpeed double, From -1 to 1
     * @param TurnSpeed double, From -1 to 1
     * @param slowAmount double, slows down drive for more control
     * @param resetNavx boolean, zero's navx to reset field orient
     */
    

    public void drive(double XSpeed, double YSpeed, double TurnSpeed, double slowMode, boolean resetNavx) {
        double slowAmount = 1.0;

        if(Math.abs(slowMode) < 0.2) {
            slowAmount = 0.7;
        }

        double[] speedsFieldOriented = Driver.fieldOrient(XSpeed, YSpeed); //The Navx given to firledOrient is the teleophelper's Navx not the Driver's Navx
        XSpeed = speedsFieldOriented[0];
        YSpeed = speedsFieldOriented[1];


        if (resetNavx) {
            navx.zeroYaw();
        }
        
        
        XSpeed *= (Constants.XdriveSensitivity * slowAmount);
        YSpeed *= (Constants.YdriveSensitivity * slowAmount);
        TurnSpeed = TurnSpeed * Constants.turningSensitivity;
        
        driver.swerveDrive(XSpeed, YSpeed, TurnSpeed);
    }

    public void moveElevator(XboxController xbox1) { 
        if (xbox1.getLeftTriggerAxis() > 0.2) {
            elevator.elevatorLift(Constants.elevatorSpeed); // Moves up
            lastSetPositionElevator = elevator.getElevatorHeight();
        } 
        else if (xbox1.getRightTriggerAxis() > 0.2) {
            elevator.elevatorLift(-Constants.elevatorSpeed); // Moves down
            lastSetPositionElevator = elevator.getElevatorHeight();
        } 
        else if (!elevatorPositionPresets){
            elevator.elevatorPIDLift(lastSetPositionElevator);
        } else {
            return;
        }
    }

    // private int currentLevel = 1;  // Start at L1
    // private boolean rtPressed = false;
    // private boolean ltPressed = false;

    // public void elevatorControl(double rightTrigger, double leftTrigger) {
    //     double rtValue = rightTrigger;
    //     double ltValue = leftTrigger;
    
        
    //     if (rtValue > 0.5 && !rtPressed) { 
    //         if (currentLevel < 4) { // Max level is L4
    //             currentLevel++;
    //             moveToCurrentLevel();
    //         }
    //         rtPressed = true; 
    //     } else if (rtValue < 0.2) {
    //         rtPressed = false; 
    //     }
    
    //     if (ltValue > 0.5 && !ltPressed) {
    //         if (currentLevel > 1) { // Min level is L1
    //             currentLevel--;
    //             moveToCurrentLevel();
    //         }
    //         ltPressed = true;
    //     } else if (ltValue < 0.2) {
    //         ltPressed = false;
    //     }

    // }

    // Moves elevator based on current level
    // private void moveToCurrentLevel() {
    //     switch (currentLevel) {
    //         case 1: elevator.moveToL1(); break;
    //         case 2: elevator.moveToL2(); break;
    //         case 3: elevator.moveToL3(); break;
    //         case 4: elevator.moveToL4(); break;
    //     }
    // }
    
    public void moveClamp(XboxController xbox) {

        if (xbox.getRightBumperButtonPressed()) {
            arm.openClaw();
        } 

        if (xbox.getLeftBumperButtonPressed()) {  
            arm.closeClaw();
        
        }
    }

    public void moveClimber(XboxController xbox) {
        if (xbox.getRawAxis(1) > 0.2 || xbox.getRawAxis(1) < -0.2) {
            climber.moveClimb(-xbox.getRawAxis(1) * 0.2);
        } else {
            climber.moveClimb(0); // Stops elevator
        }
    }

    public void moveElevatorArm(XboxController xbox) {
        //System.out.println("move elevator arm method from helper!!!!!!!!!!!!!!!!!!!");

        if (xbox.getAButton()) {
            lowTrayPos(); //optimize all these
        } else if (xbox.getXButton()) {
            firstReefPos();
        } else if (xbox.getYButton()) {
            secondReefPos();
        } else if (xbox.getBButtonPressed()) {
            thirdReefPos();
        } else {
            elevatorPositionPresets = false;
            ArmPositionPresets = false;
            return;
        }
    }

    public void lowTrayPos() {

        if (!(arm.getArmAngleRad() < Constants.armL0to3 + 0.3 && arm.getArmAngleRad() > Constants.armL0to3 - 0.3)) {
            arm.moveArmPID(Constants.armL0to3);
            ArmPositionPresets = true;
            lastSetPositionArm = arm.getArmAngleRad();
        } else if (!(elevator.getElevatorHeight() < Constants.L1_HEIGHT + 0.0002 && elevator.getElevatorHeight() > Constants.L1_HEIGHT - 0.0002)) {
            elevator.elevatorPIDLift(Constants.L1_HEIGHT);
            lastSetPositionElevator = elevator.getElevatorHeight();
            elevatorPositionPresets = true;
        } else {
            return;
        }

    }

    public void firstReefPos() {

        if (!(arm.getArmAngleRad() < Constants.armL0to3 + 0.3 && arm.getArmAngleRad() > Constants.armL0to3 - 0.3)) {
        // if (false) {
            arm.moveArmPID(Constants.armL0to3);
            lastSetPositionArm = arm.getArmAngleRad();
            ArmPositionPresets = true;
        } else if (!(elevator.getElevatorHeight() < Constants.L2_HEIGHT + 0.2 && elevator.getElevatorHeight() > Constants.L2_HEIGHT - 0.2)) {
            elevator.elevatorPIDLift(Constants.L2_HEIGHT);
            lastSetPositionElevator = elevator.getElevatorHeight();
            elevatorPositionPresets = true;
            System.out.println("elevator is moving");
        } else {
            return;
        }

    }

    public void secondReefPos() {

        if (!(arm.getArmAngleRad() < Constants.armL0to3 + 0.3 && arm.getArmAngleRad() > Constants.armL0to3 - 0.3)) {
            arm.moveArmPID(Constants.armL0to3);
            lastSetPositionArm = arm.getArmAngleRad();
            ArmPositionPresets = true;
        } else if (!(elevator.getElevatorHeight() < Constants.L3_HEIGHT + 0.0002 && elevator.getElevatorHeight() > Constants.L3_HEIGHT - 0.0002)) {
            elevator.elevatorPIDLift(Constants.L3_HEIGHT);
            lastSetPositionElevator = elevator.getElevatorHeight();
            elevatorPositionPresets = true;
        } else {
            return;
        }

    }

    public void thirdReefPos() {
        if (!(arm.getArmAngleRad() < Constants.armL4 + 0.3 && arm.getArmAngleRad() > Constants.armL4 - 0.3)) {
            arm.moveArmPID(Constants.armL4);
            lastSetPositionArm = arm.getArmAngleRad();
            ArmPositionPresets = true;
        } else if (!(elevator.getElevatorHeight() < Constants.L4_HEIGHT + 0.0002 && elevator.getElevatorHeight() > Constants.L4_HEIGHT - 0.0002)) {
            elevator.elevatorPIDLift(Constants.L4_HEIGHT);
            lastSetPositionElevator = elevator.getElevatorHeight();
            elevatorPositionPresets = true;
        } else {
            return;
        }
    }

    double speed = 0;
    double lastSetPositionArm = 0;
    double lastSetPositionElevator = 0;
    boolean ArmPositionPresets = false;
    boolean elevatorPositionPresets = false;

    public void moveArm(XboxController xbox) {

        if (xbox.getRawAxis(5) > 0.2 || xbox.getRawAxis(5) < -0.2) {
            arm.moveArm(-xbox.getRawAxis(5) * 0.4);
            lastSetPositionArm = arm.getArmAngleRad();
        } else if (!ArmPositionPresets) {
            arm.moveArmPID(lastSetPositionArm);
        } else {
            return;
        }

        SmartDashboard.putNumber("arm speed", speed);
    }


    // The B Button is a testing button for now, controls will be changed later
    // Only one of these functions should be called at any time

    public void testAutoDrivebyDistance(XboxController controller){
        if(controller.getBButton()) {
            driver.driveToPosition(driver.setDesiredPose(0, 1, 0));
        }
    }

    public void moveToAprilTag(XboxController controller){
        if(controller.getBButton()) {
            driver.driveToAprilTag(true);
        }

    }

 }