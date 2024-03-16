package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.systems.Climber;
import frc.robot.systems.Driver;
import frc.robot.systems.Limelight;
import frc.robot.systems.Shooter;




public class Robot extends TimedRobot {

    Limelight limelight = new Limelight();
    AHRS navx = new AHRS();
    Driver driver = new Driver(limelight);
    Shooter shooter = new Shooter();
    Climber climber = new Climber();
    Timer matchTimer = new Timer();

    Auto auto = new Auto(driver, shooter, climber, navx, matchTimer, limelight);
    Teleop teleop = new Teleop(driver, shooter, climber, navx, limelight);
 

    @Override
    public void robotInit() {

        matchTimer.reset();
        matchTimer.start();
        navx.reset();
        driver.initialize();
        shooter.initialize();   
        climber.initialize();
        CameraServer.startAutomaticCapture();
    }
    
    @Override
    public void robotPeriodic() {
        driver.updatePose();
        // SmartDashboard.putNumber("positionFL", driver.readAbsEncoder(0));
        // SmartDashboard.putNumber("positionFR", driver.readAbsEncoder(1));
        // SmartDashboard.putNumber("positionBL", driver.readAbsEncoder(2));
        // SmartDashboard.putNumber("positionBR", driver.readAbsEncoder(3));

        // SmartDashboard.putNumber("positionFL rads", driver.readAbsEncoderRad(0));
        // SmartDashboard.putNumber("positionFR rads", driver.readAbsEncoderRad(1));
        // SmartDashboard.putNumber("positionBL rads", driver.readAbsEncoderRad(2));
        // SmartDashboard.putNumber("positionBR rads", driver.readAbsEncoderRad(3));

        SmartDashboard.putNumber("arm position", shooter.getArmRadians());
        SmartDashboard.putNumber("climb position", climber.getClimbPosition());
        SmartDashboard.putNumber("RArm", shooter.armMotorLeft.getPosition().getValue());
        SmartDashboard.putNumber("LArm", shooter.armMotorRight.getPosition().getValue());
    
        SmartDashboard.putBoolean("null", auto.autoHelper.atTargetPosition());
    
    }
    
    @Override
    public void autonomousInit() {
        auto.restartTimer();
        driver.initialize();
    }
    
    @Override
    public void autonomousPeriodic() {

        auto.isRed = true;

        // auto.autoProgramTest();
        // auto.autoAmpTwoNote();
        auto.autoSpeakerTwoNote();
    }
    
    @Override
    public void teleopInit() {
        
    }
    
    @Override
    public void teleopPeriodic() {
        teleop.drive();
        teleop.intake();
        teleop.shoot();
        teleop.climb();
        // teleop.arm();

        //EXPEREMENTAL STUFF THAT WILL BREAK EVERYTHING
        // teleop.drive();
        teleop.armPID();
        // teleop.intake();
        // teleop.climb();
        // teleop.shootRoutine();
    }
}
