package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.systems.Climber;
import frc.robot.systems.Driver;
import frc.robot.systems.Limelight;
import frc.robot.systems.Shooter;

public class Teleop {
    XboxController xbox = new XboxController(0);
    TeleopHelper teleopHelper;

    public Teleop(Driver driver, Shooter shooter, Climber climber, AHRS navx, Limelight limelight) {
        teleopHelper = new TeleopHelper(driver, shooter, climber, navx, limelight);
    }
    
    
    
    public void drive() {
        teleopHelper.drive(-teleopHelper.getAxisValue(1), -teleopHelper.getAxisValue(0), -teleopHelper.getAxisValue(4), xbox.getLeftStickButton(), xbox.getXButton());
    }

    public void shoot() {
        teleopHelper.shoot(teleopHelper.getAxisValue(3));
    }

    public void intake() {
        teleopHelper.intake(teleopHelper.getAxisValue(2), xbox.getYButton());
    }

    public void arm() {
        teleopHelper.arm(xbox.getRightBumper(), xbox.getLeftBumper());
    }

    public void climb() {
        teleopHelper.climb(xbox.getAButton(), xbox.getBButton());
    }

    public void panic() {
        teleopHelper.panic(xbox.getStartButton());
    }
    //experemental
    public void shootRoutine() {
        teleopHelper.shootRoutine(xbox.getRightTriggerAxis() > 0.5);
    }

    public void armPID() {
        teleopHelper.setArmState(xbox.getLeftBumperPressed(), xbox.getRightBumperPressed());
    }
}
