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

}
