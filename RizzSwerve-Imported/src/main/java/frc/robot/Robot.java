package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.systems.Climber;
import frc.robot.systems.Driver;
import frc.robot.systems.Shooter;




public class Robot extends TimedRobot {

    Driver driver = new Driver();
    Shooter shooter = new Shooter();
    Climber climber = new Climber();
    Timer timer = new Timer();
    AHRS navx = new AHRS();

    Auto auto = new Auto(driver, shooter, climber, navx, timer);
    Teleop teleop = new Teleop(driver, shooter, climber, navx);
 

    @Override
    public void robotInit() {

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
