package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.systems.Climber;
import frc.robot.systems.Driver;
import frc.robot.systems.Shooter;

public class Teleop {
    XboxController xbox = new XboxController(0);
    TeleopHelper teleopHelper;
    public Teleop(Driver driver, Shooter shooter, Climber climber, AHRS navx) {
        teleopHelper = new TeleopHelper(driver, shooter, climber, navx);
    }
    
    
    
    public void drive() {
        teleopHelper.drive(teleopHelper.getAxisValue(1), teleopHelper.getAxisValue(0), teleopHelper.getAxisValue(4));
    }

    public void shoot() {
        teleopHelper.shoot(teleopHelper.getAxisValue(3));
    }

    public void intake() {
        teleopHelper.intake(teleopHelper.getAxisValue(2), xbox.getLeftBumper());
    }

    // public void arm() {
    //     teleopHelper.arm(xbox.getRightBumper(), xbox.getLeftBumper());
    // }


}
