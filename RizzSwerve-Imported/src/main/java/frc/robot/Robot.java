package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.systems.Climber;
import frc.robot.systems.Driver;
import frc.robot.systems.Shooter;
import frc.robot.systems.ShuffleDash;

import edu.wpi.first.wpilibj.XboxController;



public class Robot extends TimedRobot {

    AHRS navx = new AHRS();
    Driver driver = new Driver();
    Shooter shooter = new Shooter();
    Climber climber = new Climber();
    Timer matchTimer = new Timer();

    Auto auto = new Auto(driver, shooter, climber, navx, matchTimer);
    Teleop teleop = new Teleop(driver, shooter, climber, navx);

    XboxController control = new XboxController(0);
 

    @Override
    public void robotInit() {

        matchTimer.reset();
        matchTimer.start();
        navx.reset();
        driver.initialize();

    }
    
    @Override
    public void robotPeriodic() {
        ShuffleDash.update("Gyro", navx.getAngle());

        ShuffleDash.intLoop();
        ShuffleDash.boolLoop();
        ShuffleDash.doubleLoop();
        
        driver.updatePose();
    }
    
    @Override
    public void autonomousInit() {
        auto.restartTimer();
        driver.initialize();
    }
    
    @Override
    public void autonomousPeriodic() {
        auto.autoProgramTest();
    }
    
    @Override
    public void teleopInit() {
        
    }
    
    @Override
    public void teleopPeriodic() {
        teleop.drive();
        teleop.intake();
        teleop.shoot();

        driver.electronicStatus();

    }
}
