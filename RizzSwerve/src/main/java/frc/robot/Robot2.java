package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;


public class Robot2 extends TimedRobot {
    Auto auto;
    Teleop teleop;
    Driver driver;
    Shooter shooter;
    Climber climber;

    Timer timer = new Timer();

    @Override
    public void robotInit() {
        this.driver = new Driver();
        this.shooter = new Shooter();
        this.climber = new Climber();
        
        this.auto = new Auto(driver, shooter, climber);
        this.teleop = new Teleop(driver, shooter, climber);
    }
    
    @Override
    public void robotPeriodic() {

    }
    
    @Override
    public void autonomousInit() {

    }
    
    @Override
    public void autonomousPeriodic() {

    }
    
    @Override
    public void teleopInit() {
        
    }
    
    @Override
    public void teleopPeriodic() {

    }
}
