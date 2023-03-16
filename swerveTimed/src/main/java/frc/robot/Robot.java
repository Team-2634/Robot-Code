// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Victor was here

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.io.Console;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  //private AHRS navx;
  private final XboxController xbox = new XboxController(0);

  //constuct pid and other stuff
  final double kp = 0.3;
  final double ki = 0.0;
  final double kd = 0.0;

  PIDController pidFrontLeftTurn = new PIDController(kp, ki, kd);
  PIDController pidFrontRightTurn = new PIDController(kp, ki, kd);
  PIDController pidBackLeftTurn = new PIDController(kp, ki, kd);
  PIDController pidBackRightTurn = new PIDController(kp, ki, kd);
  
  public final WPI_TalonFX frontLeftDrive = new WPI_TalonFX(2);
  public final WPI_TalonFX frontRightDrive = new WPI_TalonFX(4);
  public final WPI_TalonFX backLeftDrive = new WPI_TalonFX(6);
  public final WPI_TalonFX backRightDrive= new WPI_TalonFX(8); 

  public final WPI_TalonFX frontLeftSteer = new WPI_TalonFX(1);
  public final WPI_TalonFX frontRightSteer = new WPI_TalonFX(3);
  public final WPI_TalonFX backLeftSteer = new WPI_TalonFX(5);
  public final WPI_TalonFX backRightSteer = new WPI_TalonFX(7); 
/* 
  public final Encoder frontLeftAbsolute = new Encoder(0, 1);
  public final Encoder frontRightAbsolute = new Encoder(0, 1);
  public final Encoder backLeftAbsolute = new Encoder(0, 1);
  public final Encoder backRightAbsolute = new Encoder(0, 1);
  */

  //conversion factors !!!CHANGE THESE!!!
  public final double kWheelDiameterMeters = Units.inchesToMeters(3.75);
  public final double kDriveMotorGearRatio = 1 / 8.45;
  public final double kTurningMotorGearRatio = 1.0 / (150.0/7.0); //motor rotations to wheel rotations conversion factor
  public final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
  public final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI; 
  public final double kTurningEncoderTicksToRad = kTurningEncoderRot2Rad/2048;
  //public final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
  //public final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
  //public final double absEncoderTicks = ;
  //public final double absEncoderTicks2Rad = absEncoderTicks / 2 / Math.PI;

  double driveSensitivity = 0.4; //do not change above 1
  double turningSensitivity = 2;
  double maxSpeedMpS = 5; // metres/sec
/* 
  double frontLeftOffset;
  double frontRightOffset;
  double backLeftOffset;
  double backRightOffset;

*/
  //define location of modules/wheels
  Translation2d m_frontLeftLocation = new Translation2d(0.340, 0.285);
  Translation2d m_frontRightLocation = new Translation2d(0.340, -0.285);
  Translation2d m_backLeftLocation = new Translation2d(-0.340, 0.285);
  Translation2d m_backRightLocation = new Translation2d(-0.340, -0.285);

  //feed the four modules into a kinematics function
  SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
  m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    
    //navx.reset();
    
    frontLeftSteer.setSelectedSensorPosition(0);
    frontRightSteer.setSelectedSensorPosition(0);
    backLeftSteer.setSelectedSensorPosition(0);
    backRightSteer.setSelectedSensorPosition(0);
/* 
    frontLeftOffset = frontLeftAbsolute.getDistance();
    frontRightOffset = frontRightAbsolute.getDistance();
    backLeftOffset = backLeftAbsolute.getDistance();
    backRightOffset = backRightAbsolute.getDistance();
    */
    frontLeftSteer.setNeutralMode(NeutralMode.Brake);
    frontRightSteer.setNeutralMode(NeutralMode.Brake);
    backLeftSteer.setNeutralMode(NeutralMode.Brake);
    backRightSteer.setNeutralMode(NeutralMode.Brake);

    frontLeftSteer.setNeutralMode(NeutralMode.Brake);
    frontRightSteer.setNeutralMode(NeutralMode.Brake);
    backLeftSteer.setNeutralMode(NeutralMode.Brake);
    backRightSteer.setNeutralMode(NeutralMode.Brake);

    frontLeftSteer.setInverted(true);
    frontRightSteer.setInverted(true);
    backLeftSteer.setInverted(true);
    backRightSteer.setInverted(true);

    frontLeftDrive.setInverted(true);
    frontRightDrive.setInverted(true);
    backLeftDrive.setInverted(true);
    backRightDrive.setInverted(true);
    

    //make pids treat values pi radians and -pi radians as the same and have them loop around
    pidFrontLeftTurn.enableContinuousInput(-Math.PI, Math.PI);
    pidFrontRightTurn.enableContinuousInput(-Math.PI, Math.PI);
    pidBackLeftTurn.enableContinuousInput(-Math.PI, Math.PI);
    pidBackRightTurn.enableContinuousInput(-Math.PI, Math.PI);

  }
  
  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  public double removeDeadzone(int axisInput) {
    if (Math.abs(xbox.getRawAxis(axisInput)) < 0.1) {
      return 0;
    }
    return xbox.getRawAxis(axisInput);
  }
  /** This function is called once when teleop is enabled. */
  
  public void swerveDrive(double xSpeed, double ySpeed, double rotSpeed) {
    
    

    //controller inputs are multiplied by max speed to return a fraction of maximum speed and modified further by sensitivity
    //max controller value of 1 returns maximum speed achivable by the robot before being reduced by sensitivity
    //turning is black magic
    

    ChassisSpeeds desiredSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rotSpeed);
    
    //make desiredSpeeds into speeds and angles for each module
    SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(desiredSpeeds);

    //normalize module values to remove impossible speed values
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, maxSpeedMpS);
    
    SwerveModuleState frontLeftModule = moduleStates[0];
    SwerveModuleState frontRightModule = moduleStates[1];
    SwerveModuleState backLeftModule = moduleStates[2];
    SwerveModuleState backRightModule = moduleStates[3];

    //optimize wheel angles (ex. wheel is at 359deg and needs to go to 1deg. wheel will now go 2deg instead of 358deg)

    double frontLeftSensorPos = frontLeftSteer.getSelectedSensorPosition()*kTurningEncoderTicksToRad;
    double frontRightSensorPos = frontRightSteer.getSelectedSensorPosition()*kTurningEncoderTicksToRad;
    double backLeftSensorPos = backLeftSteer.getSelectedSensorPosition()*kTurningEncoderTicksToRad;
    double backRightSensorPos = backRightSteer.getSelectedSensorPosition()*kTurningEncoderTicksToRad;

    var frontLeftCurrentAngle = new Rotation2d(frontLeftSensorPos);
    var frontRightCurrentAngle = new Rotation2d(frontRightSensorPos);
    var backLeftCurrentAngle = new Rotation2d(backLeftSensorPos);
    var backRightCurrentAngle = new Rotation2d(backRightSensorPos);
    
    var frontLeftOptimized = SwerveModuleState.optimize(frontLeftModule, frontLeftCurrentAngle);
    var frontRightOptimized = SwerveModuleState.optimize(frontRightModule, frontRightCurrentAngle);
    var backLeftOptimized = SwerveModuleState.optimize(backLeftModule, backLeftCurrentAngle);
    var backRightOptimized = SwerveModuleState.optimize(backRightModule, backRightCurrentAngle);
    
    double frontLeftTurnPower = pidFrontLeftTurn.calculate(frontLeftSteer.getSelectedSensorPosition()*kTurningEncoderTicksToRad, frontLeftOptimized.angle.getRadians());
    double frontRightTurnPower = pidFrontRightTurn.calculate(frontRightSteer.getSelectedSensorPosition()*kTurningEncoderTicksToRad, frontRightOptimized.angle.getRadians());
    double backLeftTurnPower = pidBackLeftTurn.calculate(backLeftSteer.getSelectedSensorPosition()*kTurningEncoderTicksToRad, backLeftOptimized.angle.getRadians());
    double backRightTurnPower = pidBackRightTurn.calculate(backRightSteer.getSelectedSensorPosition()*kTurningEncoderTicksToRad, backRightOptimized.angle.getRadians());



    //set steer motor power to the pid output of current position in radians and desired position in radians
    //positive is clockwise (right side up)
    frontLeftSteer.set(frontLeftTurnPower);
    frontRightSteer.set(frontRightTurnPower);
    backLeftSteer.set(backLeftTurnPower);
    backRightSteer.set(backRightTurnPower);

    SmartDashboard.putNumber("frontLeftRawSpeed", frontLeftModule.speedMetersPerSecond);
    SmartDashboard.putNumber("frontRightRawSpeed", frontRightModule.speedMetersPerSecond);

    SmartDashboard.putNumber("frontLeftSpeed", frontLeftOptimized.speedMetersPerSecond);
    SmartDashboard.putNumber("frontRightSpeed", frontRightOptimized.speedMetersPerSecond);

    SmartDashboard.putNumber("radsPerMotorTurn", kTurningEncoderRot2Rad);
    SmartDashboard.putNumber("motorTurningGearRatio", kTurningMotorGearRatio);

    SmartDashboard.putNumber("frontLeftPidPosError", pidFrontLeftTurn.getPositionError());
    SmartDashboard.putNumber("frontRightPidPosError", pidFrontRightTurn.getPositionError());
    

    SmartDashboard.putNumber("frontLeftCurrentAngle", frontLeftCurrentAngle.getDegrees());
    SmartDashboard.putNumber("frontRightCurrentAngle", frontRightCurrentAngle.getDegrees());


    SmartDashboard.putNumber("frontLeftSensorPosRadians", frontLeftSensorPos);
    SmartDashboard.putNumber("frontRightSensorPosRadians", frontRightSensorPos);

    SmartDashboard.putNumber("frontLeftDesiredRadiansRaw", frontLeftModule.angle.getRadians());
    SmartDashboard.putNumber("frontRightDesiredRadianRaw", frontRightModule.angle.getRadians());
    // SmartDashboard.putNumber("backLeftDesiredRadiansRaw", backLeftOptimized.angle.getRadians());
    // SmartDashboard.putNumber("backRightDesiredRadiansRaw", backRightOptimized.angle.getRadians());

    SmartDashboard.putNumber("frontLeftDesiredRadians", frontLeftOptimized.angle.getRadians());
    SmartDashboard.putNumber("frontRightDesiredRadians", frontRightOptimized.angle.getRadians());
    // SmartDashboard.putNumber("backLeftDesiredRadians", backLeftOptimized.angle.getRadians());
    // SmartDashboard.putNumber("backRightDesiredRadians", backRightOptimized.angle.getRadians());

    SmartDashboard.putNumber("frontLeftTurnPower", frontLeftTurnPower);
    SmartDashboard.putNumber("frontRightTurnPower", frontRightTurnPower);
    // SmartDashboard.putNumber("backLeftTurnPower", backLeftTurnPower);
    // SmartDashboard.putNumber("backRightTurnPower", backRightTurnPower);    

    SmartDashboard.putNumber("frontLeftTurnPosition", frontLeftSteer.getSelectedSensorPosition());
    SmartDashboard.putNumber("frontRightTurnPosition", frontRightSteer.getSelectedSensorPosition());
    // SmartDashboard.putNumber("backLeftTurnPosition", backLeftSteer.getSelectedSensorPosition());
    // SmartDashboard.putNumber("backRightTurnPosition", backRightSteer.getSelectedSensorPosition()); 

    SmartDashboard.putNumber("frontLeftDesiredSpeed", frontLeftOptimized.speedMetersPerSecond);
    SmartDashboard.putNumber("frontRightDesiredSpeed", frontRightOptimized.speedMetersPerSecond);
    // SmartDashboard.putNumber("backLeftDesiredSpeed", backLeftOptimized.speedMetersPerSecond);
    // SmartDashboard.putNumber("backRightDesiredSpeed", backRightOptimized.speedMetersPerSecond);
    
    SmartDashboard.putNumber("desiredX", xSpeed);
    SmartDashboard.putNumber("desiredY", ySpeed);
    SmartDashboard.putNumber("desiredRot", rotSpeed);

    //set drive power to desired speed div max speed to get value between 0 and 1
    frontLeftDrive.set(frontLeftOptimized.speedMetersPerSecond/maxSpeedMpS);
    frontRightDrive.set(frontRightOptimized.speedMetersPerSecond/maxSpeedMpS);
    backLeftDrive.set(backLeftOptimized.speedMetersPerSecond/maxSpeedMpS);
    backRightDrive.set(backRightOptimized.speedMetersPerSecond/maxSpeedMpS);

    
  }

  @Override
  public void teleopInit() {
    //swerveDrive();

    frontLeftSteer.setSelectedSensorPosition(0);
    frontRightSteer.setSelectedSensorPosition(0);
    backLeftSteer.setSelectedSensorPosition(0);
    backRightSteer.setSelectedSensorPosition(0);

    //frontLeftSteer.configSelectedFeedbackCoefficient(kTurningEncoderRot2Rad);
  }
    
  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    double contXSpeed = removeDeadzone(1) * maxSpeedMpS * driveSensitivity;
    double contYSpeed = removeDeadzone(0) * maxSpeedMpS * driveSensitivity;
    double contTurnSpeed = removeDeadzone(4) * turningSensitivity;

    swerveDrive(contXSpeed, contYSpeed, contTurnSpeed);

    frontLeftSteer.setNeutralMode(NeutralMode.Brake);
    frontRightSteer.setNeutralMode(NeutralMode.Brake);
    backLeftSteer.setNeutralMode(NeutralMode.Brake);
    backRightSteer.setNeutralMode(NeutralMode.Brake);

    frontLeftDrive.setNeutralMode(NeutralMode.Brake);
    frontRightDrive.setNeutralMode(NeutralMode.Brake);
    backLeftDrive.setNeutralMode(NeutralMode.Brake);
    backRightDrive.setNeutralMode(NeutralMode.Brake);
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {

}
}
