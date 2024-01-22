package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import frc.robot.systems.Climber;
import frc.robot.systems.Driver;
import frc.robot.systems.Shooter;

public class Teleop {
    TeleopHelper teleopHelper;
    public Teleop(Driver driver, Shooter shooter, Climber climber, AHRS navx) {
        teleopHelper = new TeleopHelper(driver, shooter, climber, navx);
    }
    
    
    
    public void drive() {
        teleopHelper.drive(teleopHelper.removeDeadzone(1), teleopHelper.removeDeadzone(0), teleopHelper.removeDeadzone(4));
    }

    

    




}
