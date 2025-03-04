package frc.robot;

//import com.kauailabs.navx.frc.AHRS;
import com.studica.frc.AHRS; //changed

//testing

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
    
    final XboxController xbox1 = new XboxController(0);
    final XboxController xbox2 = new XboxController(1);


    boolean xboxElevatorInput = xbox1.getLeftBumperButton();

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

    public void liftElevatorFromBumper(boolean input){
        if(xboxElevatorInput) {
            elevator.elevatorLiftUp(0.05);
        }
        else if(xboxElevatorInput != true) {
            elevator.elevatorLiftUp(0); //just incase it doesnt stop after you press Left bumper
        }


    }

}
