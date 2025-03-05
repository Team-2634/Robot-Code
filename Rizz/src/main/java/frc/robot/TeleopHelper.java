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
    
    final XboxController xbox1 = new XboxController(0); //Controller For Swerve Drive
    final XboxController xbox2 = new XboxController(1); //Controller For Arm/Elevator

    //boolean xboxElevatorInput = xbox1.getLeftBumperButton();
    

    public void drive(double XSpeed, double YSpeed, double TurnSpeed) {
        double[] speedsFieldOriented = Driver.fieldOrient(XSpeed, YSpeed, navx);
        XSpeed = speedsFieldOriented[0] * Constants.XdriveSensitivity;
        YSpeed = speedsFieldOriented[1] * Constants.YdriveSensitivity;
        TurnSpeed = TurnSpeed * Constants.turningSensitivity;
        
        driver.swerveDrive(XSpeed, YSpeed, TurnSpeed);
    }

    public double removeDeadzone(int axisInput) {
        if (Math.abs(xbox1.getRawAxis(axisInput)) < Constants.controllerDeadzone) {
            return 0;
        }
        return xbox1.getRawAxis(axisInput);
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
    //     if (xbox1.getLeftBumperButtonPressed()) {
    //         elevator.elevatorLift(0.05); // Moves up
    //     } 
    //     else if (xbox1.getRightBumperButtonPressed()) {
    //         elevator.elevatorLift(-0.05); // Moves down
    //     } 
    //     else {
    //         elevator.elevatorLift(0); // Stops elevator
    //     }
    // }

    public void liftElevatorFromBumper() { //STILL HAVE TO TEST!
        
        double[] elevatorLevels = {0.1, 0.3, 0.5, 0.7}; //Elevator Lift Height 
        int currentLevel = 0; // Track the elevator level
    
        // Move up
        if (xbox2.getRightBumperPressed()) {
            if (currentLevel < elevatorLevels.length - 1) {
                currentLevel++;
            }
            elevator.elevatorLift(elevatorLevels[currentLevel]);
        } 
        // Move down
        else if (xbox2.getLeftBumperPressed()) {
            if (currentLevel > 0) {
                currentLevel--;
            }
            elevator.elevatorLift(elevatorLevels[currentLevel]);
        }
    }

    public void moveArm(){

        //figure out arm code

       
        }

        

}
