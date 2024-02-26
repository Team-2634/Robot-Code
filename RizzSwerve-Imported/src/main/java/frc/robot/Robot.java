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

    Driver driver = new Driver();
    Shooter shooter = new Shooter();
    Climber climber = new Climber();
    Timer matchTimer = new Timer();
    AHRS navx = new AHRS();
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
        driver.electronicStatus();

        ShuffleDash.intLoop();
        ShuffleDash.boolLoop();
    }
    
    @Override
    public void autonomousInit() {
        auto.restartTimer();
    }
    
    @Override
    public void autonomousPeriodic() {
        auto.autoTopAndBottom();
    }
    
    @Override
    public void teleopInit() {
        
    }
    
    @Override
    public void teleopPeriodic() {
        teleop.drive();
    }
}
