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
        if (xbox1.getRightTriggerAxis() > 0.2 && elevator.getElevatorHeight() < Constants.elevatorHighHardstop) {
            elevator.elevatorLift(Constants.elevatorSpeed); // Moves up
        } 
        else if (xbox1.getLeftTriggerAxis() > 0.2 && elevator.getElevatorHeight() > Constants.elevatorLowHardstop) {
            elevator.elevatorLift(-Constants.elevatorSpeed); // Moves down
        } 
        else {
            elevator.elevatorLift(0); // Stops elevator
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
            arm.closeClaw();
        } 

        if (xbox.getLeftBumperButtonPressed()) {  
            arm.openClaw();
        
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

        if (xbox.getAButton()) {
            lowTrayPos();
        } else if (xbox.getXButton()) {
            firstReefPos();
        } else if (xbox.getYButton()) {
            secondReefPos();
        } else if (xbox.getBButtonPressed()) {
            thirdReefPos();
        }
    }

    public void lowTrayPos() {

        if (arm.getArmAngleRad() < Constants.arm35DegreeInRadians + 0.2 && arm.getArmAngleRad() > Constants.arm35DegreeInRadians - 0.2) {
            arm.moveArmPID(Constants.arm35DegreeInRadians);
        } else if (elevator.getElevatorHeight() < Constants.L1_HEIGHT + 0.03 && elevator.getElevatorHeight() > Constants.L1_HEIGHT - 0.03) {
            elevator.elevatorPIDLift(Constants.L1_HEIGHT);
        } else {
            return;
        }

    }

    public void firstReefPos() {

        if (arm.getArmAngleRad() < Constants.arm35DegreeInRadians + 0.2 && arm.getArmAngleRad() > Constants.arm35DegreeInRadians - 0.2) {
            arm.moveArmPID(Constants.arm35DegreeInRadians);
        } else if (elevator.getElevatorHeight() < Constants.L2_HEIGHT + 0.03 && elevator.getElevatorHeight() > Constants.L2_HEIGHT - 0.03) {
            elevator.elevatorPIDLift(Constants.L2_HEIGHT);
        } else {
            return;
        }

    }

    public void secondReefPos() {

        if (arm.getArmAngleRad() < Constants.arm35DegreeInRadians + 0.2 && arm.getArmAngleRad() > Constants.arm35DegreeInRadians - 0.2) {
            arm.moveArmPID(Constants.arm35DegreeInRadians);
        } else if (elevator.getElevatorHeight() < Constants.L3_HEIGHT + 0.03 && elevator.getElevatorHeight() > Constants.L3_HEIGHT - 0.03) {
            elevator.elevatorPIDLift(Constants.L3_HEIGHT);
        } else {
            return;
        }

    }

    public void thirdReefPos() {

    }

    

    double speed = 0;

    public void armTestTest(XboxController xbox) {
        if (xbox.getXButton()) {
            speed = -Constants.armSpeed;
        } else if (xbox.getBButton()) {
            speed = Constants.armSpeed;
        } else {
            speed = 0;
        }

        SmartDashboard.putNumber("arm speed", speed);
        arm.moveArm(speed);

    }

    public void rotateAlignToAprilTag(XboxController controller) {
        if(controller.getBButton()) {
            if(limelight.Xoffset() >= 0.1) {
                drive(0, 0, -0.1, false);
            }
            else if(limelight.Xoffset() <= -0.1) {
                drive(0, 0, 0.1, false);

            }
        }
    }
    
    public void moveToAprilTag(XboxController controller){
        limelight.moveToAprilTag(controller);
    }
    

 }




