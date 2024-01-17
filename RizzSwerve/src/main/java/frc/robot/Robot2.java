package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;


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
        
        timerRobot.reset();
        timerRobot.start();

        navx.calibrate();
        navx.reset();
        resetEncoders();
        resetPIDs();
        setMotorBreaks();
        invertMotors();
        continouousInput();

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
        autoTopAndBottom();
    }
    
    @Override
    public void teleopInit() {
        swerveDrive(contXSpeedField, contYSpeedField, contTurnSpeed);
    }
    
    @Override
    public void teleopPeriodic() {

    }
}
