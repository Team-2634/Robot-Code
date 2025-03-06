package frc.robot;

//import com.kauailabs.navx.frc.AHRS;
import com.studica.frc.AHRS;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.systems.Climber;
import frc.robot.systems.Driver;
import frc.robot.systems.Shooter;


public class Teleop {

    XboxController xboxDrive = new XboxController(0);

    TeleopHelper teleopHelper;
    public Teleop(Driver driver, Shooter shooter, Climber climber, AHRS navx) {
        teleopHelper = new TeleopHelper(driver, shooter, climber, navx);
    }

    public double removeDeadzone(double input) {
        if (Math.abs(input) < Constants.controllerDeadzone) {
            return 0;
        } 
        return input;
    }
    
    public void drive() {
        teleopHelper.drive(teleopHelper.removeDeadzone(1), teleopHelper.removeDeadzone(0), teleopHelper.removeDeadzone(4));
    }

}
