package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.systems.Climber;
import frc.robot.systems.Driver;
import frc.robot.systems.Shooter;



public class Robot2 extends TimedRobot {
    Auto auto;
    Teleop teleop;
    Driver driver;
    Shooter shooter;
    Climber climber;
 
    Timer timer = new Timer();
    AHRS navx = new AHRS(SPI.Port.kMXP);

    @Override
    public void robotInit() {

        this.driver = new Driver();
        this.shooter = new Shooter();
        this.climber = new Climber();
        
        this.auto = new Auto(driver, shooter, climber);
        this.teleop = new Teleop(driver, shooter, climber);
        
        timer.reset();
        timer.start();

        navx.calibrate();
        navx.reset();
        driver.resetEncoders();
        driver.resetPIDs();
        driver.setMotorBreaks();
        driver.invertMotors();
        driver.continouousInput();

    }
    
    @Override
    public void robotPeriodic() {
        botYaw_angleRad = Math.toRadians(navx.getAngle());
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
