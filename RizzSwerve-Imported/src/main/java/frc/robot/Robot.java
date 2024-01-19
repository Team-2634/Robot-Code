package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.systems.Climber;
import frc.robot.systems.Driver;
import frc.robot.systems.Shooter;




public class Robot extends TimedRobot {
    Auto auto;
    Teleop teleop;
    Driver driver;
    Shooter shooter;
    Climber climber;
 
    Timer timer = new Timer();
    AHRS navx = new AHRS();

    @Override
    public void robotInit() {

        this.driver = new Driver();
        this.shooter = new Shooter();
        this.climber = new Climber();
        
        this.auto = new Auto(driver, shooter, climber, navx, timer);
        this.teleop = new Teleop(driver, shooter, climber, navx);
        
        timer.reset();
        timer.start();
        navx.reset();
        driver.resetEncoders();
        driver.resetPIDs();
        driver.setMotorBreaks();
        driver.invertMotors();
        driver.continouousInput();

    }
    
    @Override
    public void robotPeriodic() {
    }
    
    @Override
    public void autonomousInit() {

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

    }
}
