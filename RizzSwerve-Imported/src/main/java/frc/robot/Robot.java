package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.systems.Climber;
import frc.robot.systems.Driver;
import frc.robot.systems.Limelight;
import frc.robot.systems.Shooter;
// import frc.robot.systems.Webcam;
import frc.robot.Constants;

import frc.robot.TeleopHelper;



public class Robot extends TimedRobot {

    Limelight limelight = new Limelight();
    AHRS navx = new AHRS();
    // Webcam webcam = new Webcam();
    Driver driver = new Driver(limelight);
    Shooter shooter = new Shooter();
    Climber climber = new Climber();
    Timer matchTimer = new Timer();

    Auto auto = new Auto(driver, shooter, climber, navx, matchTimer, limelight);
    Teleop teleop = new Teleop(driver, shooter, climber, navx, limelight);

    TeleopHelper teleopHelper = new TeleopHelper(driver, shooter, climber, navx, limelight);
 
    PowerDistribution pdBoard = new PowerDistribution();

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

    ColorSensorV3 sensor = new ColorSensorV3(I2C.Port.kMXP);
    
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
        
        // double totalAmperage = 
        //     driver.driveMotorArray[0].getSupplyCurrent().getValue() + 
        //     driver.driveMotorArray[1].getSupplyCurrent().getValue() + 
        //     driver.driveMotorArray[2].getSupplyCurrent().getValue() + 
        //     driver.driveMotorArray[3].getSupplyCurrent().getValue() + 
        //     driver.steerMotorArray[0].getSupplyCurrent().getValue() + 
        //     driver.steerMotorArray[1].getSupplyCurrent().getValue() + 
        //     driver.steerMotorArray[2].getSupplyCurrent().getValue() + 
        //     driver.steerMotorArray[3].getSupplyCurrent().getValue() + 
        //     shooter.armMotorLeft.getSupplyCurrent().getValue() +
        //     shooter.armMotorRight.getSupplyCurrent().getValue() +
        //     shooter.shooterMotorLeft.getSupplyCurrent().getValue() +
        //     shooter.shooterMotorRight.getSupplyCurrent().getValue() +
        //     shooter.intake.getSupplyCurrent().getValue() +
        //     climber.leftClimb.getSupplyCurrent().getValue() +
        //     climber.rightClimb.getSupplyCurrent().getValue();
        // SmartDashboard.putNumber("motorRot0", driver.frontLeftDrive.getPosition().getValue());
        // SmartDashboard.putNumber("motorRot1", driver.frontRightDrive.getPosition().getValue());
        // SmartDashboard.putNumber("motorRot2", driver.backLeftDrive.getPosition().getValue());
        // SmartDashboard.putNumber("motorRot3", driver.backRightDrive.getPosition().getValue());

        SmartDashboard.putNumber("inputX", driver.getPose().getX());
        SmartDashboard.putNumber("inputY", driver.getPose().getY());
        SmartDashboard.putNumber("inputRot", driver.getPose().getRotation().getRadians());
        // SmartDashboard.putBoolean("x good", auto.autoHelper.autoXPID.atSetpoint());
        // SmartDashboard.putBoolean("y good", auto.autoHelper.autoYPID.atSetpoint());
        // SmartDashboard.putBoolean("Rot good", auto.autoHelper.autoTurnPID.atSetpoint());
        // SmartDashboard.putNumber("navx", navx.getAngle());
        // SmartDashboard.putNumber("navx2d", navx.getRotation2d().getDegrees());
    
        SmartDashboard.putBoolean("NOTE", sensor.getRed() > 300);
        SmartDashboard.putNumber("red", sensor.getRed());
        SmartDashboard.putNumber("green", sensor.getGreen());
        SmartDashboard.putNumber("blue", sensor.getBlue());
        SmartDashboard.putNumber("prox", sensor.getProximity());

        // SmartDashboard.putNumber("total amps", pdBoard.getTotalCurrent());
        // SmartDashboard.putNumber("total volts", pdBoard.getVoltage());

        SmartDashboard.putNumber("Angle Value", teleopHelper.calculateArmAngle());
    }
    
    @Override
    public void autonomousInit() {
        auto.restartTimer();
        auto.counter = 0;
        driver.initialize();

        auto.counter = 0;
        // DO NOT FORGET TO SET STARTING POSITION
        Pose2d startSpeakerFrontBlue = new Pose2d(auto.getObjectPositionX(0), auto.getObjectPositionY(0), new Rotation2d());
        Pose2d startSpeakerFrontRed = new Pose2d(auto.getObjectPositionX(0), auto.getObjectPositionY(0), new Rotation2d(Math.PI));

        Pose2d startSpeakerLeftBlue = new Pose2d(auto.getObjectPositionX(0), 0, new Rotation2d(Math.toRadians(60)));
        Pose2d startSpeakerLeftRed = new Pose2d(0, 0, new Rotation2d(Math.toRadians(-120)));

        Pose2d startSpeakerRightBlue = new Pose2d(0, 0, new Rotation2d(Math.toRadians(-60)));
        Pose2d startSpeakerRightRed = new Pose2d(0, 0, new Rotation2d(Math.toRadians(120)));

        Pose2d startOffsideBlue = new Pose2d(0, 0, new Rotation2d());
        Pose2d startOffsideRed = new Pose2d(0, 0, new Rotation2d());

        //in front of speaker
        driver.startAuto(startSpeakerFrontBlue);
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
        teleop.armPID();
        teleop.limelight();
        // teleop.shootRoutine();
    }
}
