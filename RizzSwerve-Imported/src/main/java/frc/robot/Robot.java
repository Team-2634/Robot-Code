package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.systems.Climber;
import frc.robot.systems.Driver;
import frc.robot.systems.Shooter;




public class Robot extends TimedRobot {

    AHRS navx = new AHRS();
    Driver driver = new Driver(navx);
    Shooter shooter = new Shooter();
    Climber climber = new Climber();
    Timer matchTimer = new Timer();

    Auto auto = new Auto(driver, shooter, climber, navx, matchTimer);
    Teleop teleop = new Teleop(driver, shooter, climber, navx);
 

    @Override
    public void robotInit() {

        matchTimer.reset();
        matchTimer.start();
        navx.reset();
        driver.initialize();

    }
    
    @Override
    public void robotPeriodic() {
        driver.updatePose();
    }
    
    @Override
    public void autonomousInit() {
        auto.restartTimer();
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
    }
}
