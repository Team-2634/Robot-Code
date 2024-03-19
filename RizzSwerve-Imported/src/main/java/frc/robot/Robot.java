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
import frc.robot.Constants;



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
        auto.autoHelper.timer.start();
    }
    
    @Override
    public void robotPeriodic() {
        driver.updatePose();
        // SmartDashboard.putNumber("positionFL", driver.readAbsEncoder(0));
        // SmartDashboard.putNumber("positionFR", driver.readAbsEncoder(1));
        // SmartDashboard.putNumber("positionBL", driver.readAbsEncoder(2));
        // SmartDashboard.putNumber("positionBR", driver.readAbsEncoder(3));

        //SmartDashboard.putNumber("positionFL rads", driver.readAbsEncoderRad(0));
        //SmartDashboard.putNumber("positionFR rads", driver.readAbsEncoderRad(1));
        //SmartDashboard.putNumber("positionBL rads", driver.readAbsEncoderRad(2));
        //SmartDashboard.putNumber("positionBR rads", driver.readAbsEncoderRad(3));

        // SmartDashboard.putNumber(null, kDefaultPeriod)

        SmartDashboard.putNumber("arm position", shooter.getArmRadians());
        SmartDashboard.putNumber("climb position", climber.getClimbPosition());
        
        double totalAmperage = 
            driver.driveMotorArray[0].getSupplyCurrent().getValue() + 
            driver.driveMotorArray[1].getSupplyCurrent().getValue() + 
            driver.driveMotorArray[2].getSupplyCurrent().getValue() + 
            driver.driveMotorArray[3].getSupplyCurrent().getValue() + 
            driver.steerMotorArray[0].getSupplyCurrent().getValue() + 
            driver.steerMotorArray[1].getSupplyCurrent().getValue() + 
            driver.steerMotorArray[2].getSupplyCurrent().getValue() + 
            driver.steerMotorArray[3].getSupplyCurrent().getValue() + 
            shooter.armMotorLeft.getSupplyCurrent().getValue() +
            shooter.armMotorRight.getSupplyCurrent().getValue() +
            shooter.shooterMotorLeft.getSupplyCurrent().getValue() +
            shooter.shooterMotorRight.getSupplyCurrent().getValue() +
            shooter.intake.getSupplyCurrent().getValue() +
            climber.leftClimb.getSupplyCurrent().getValue() +
            climber.rightClimb.getSupplyCurrent().getValue();

        SmartDashboard.putNumber("TOTAL AMPERAGE", totalAmperage);

    }
    
    @Override
    public void autonomousInit() {
        auto.restartTimer();
        driver.initialize();


        // DO NOT FORGET TO SET STARTING POSITION

        //in front of speaker
        driver.startAuto(null);
    }
    
    @Override
    public void autonomousPeriodic() {

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
        teleop.shoot();
        teleop.intake();
        teleop.climb();
        // teleop.arm();
        teleop.panic();

        //EXPEREMENTAL STUFF THAT WILL BREAK EVERYTHING
        // teleop.drive();
        teleop.armPID();
        // teleop.intake();
        // teleop.climb();
        // teleop.shootRoutine();
    }
}
