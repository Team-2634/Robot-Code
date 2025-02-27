package frc.robot;

//import com.kauailabs.navx.frc.AHRS;
import com.studica.frc.AHRS;
import com.github.kwhat.jnativehook.GlobalScreen;
import com.github.kwhat.jnativehook.keyboard.NativeKeyListener;
import com.github.kwhat.jnativehook.*;
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
    KeyListener key = new KeyListener();
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
        try {
            GlobalScreen.registerNativeHook();
        }
        catch (NativeHookException ex) {
            System.err.println("There was a problem registering the native hook.");
            System.err.println(ex.getMessage());
        }
        GlobalScreen.addNativeKeyListener(key);

        teleop.drive();
    }
}
