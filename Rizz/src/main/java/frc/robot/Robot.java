package frc.robot;

//import com.kauailabs.navx.frc.AHRS;
import com.studica.frc.AHRS;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.systems.Climber;
import frc.robot.systems.Driver;
import frc.robot.systems.Shooter;
//import edu.wpi.first.wpilibj.SPI;

public class Robot extends TimedRobot {

    Driver driver = new Driver();
    Shooter shooter = new Shooter();
    Climber climber = new Climber();
    Timer matchTimer = new Timer();
    LimeLight limelight = new LimeLight();
    
    AHRS navx = new AHRS(AHRS.NavXComType.kMXP_SPI); 


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
    }
    
    @Override
    public void autonomousInit() {
        auto.restartTimer();
    }
    
    @Override
    public void autonomousPeriodic() {
        auto.moveForwardTest();
        //Hi hans this is for testing
    }
    
    @Override
    public void teleopInit() {
        
    }
    
    @Override
    public void teleopPeriodic() {
        teleop.drive();
    }
}
