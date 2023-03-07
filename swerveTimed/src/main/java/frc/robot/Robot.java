// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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

  private AHRS navx;
  private final XboxController xbox = new XboxController(0);

  //constuct pid and other stuff
  final double kp = 0.5;
  final double ki = 0.05;
  final double kd = 0.05;

  PIDController pidFrontLeftTurn = new PIDController(kp, ki, kd);
  PIDController pidFrontRightTurn = new PIDController(kp, ki, kd);
  PIDController pidBackLeftTurn = new PIDController(kp, ki, kd);
  PIDController pidBackRightTurn = new PIDController(kp, ki, kd);
  
  PIDController pidDrive = new PIDController(kp, ki, kd);
  
  public final WPI_TalonFX frontLeftDrive = new WPI_TalonFX(1);
  public final WPI_TalonFX frontRightDrive = new WPI_TalonFX(3);
  public final WPI_TalonFX backLeftDrive = new WPI_TalonFX(2);
  public final WPI_TalonFX backRightDrive= new WPI_TalonFX(4); 

  public final WPI_TalonFX frontLeftSteer = new WPI_TalonFX(5);
  public final WPI_TalonFX frontRightSteer = new WPI_TalonFX(6);
  public final WPI_TalonFX backLeftSteer = new WPI_TalonFX(7);
  public final WPI_TalonFX backRightSteer = new WPI_TalonFX(8); 

  //conversion factors !!!CHANGE THESE!!!
  public final double kWheelDiameterMeters = Units.inchesToMeters(4);
  public final double kDriveMotorGearRatio = 1 / 5.8462;
  public final double kTurningMotorGearRatio = 1 / 18.0;
  public final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
  public final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
  public final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
  public final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
  public final double kPTurning = 0.5;

  double driveSensitivity = 5;
  double turningSensitivity = 1;
  double maxSpeed = 10;

  //define location of modules/wheels
  Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
  Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
  Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
  Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);

  //feed the four modules into a kinematics function
  SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
  m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    
    
    navx.reset();
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


  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    DifferentialDrive.
    pidFrontLeftTurn.enableContinuousInput(-Math.PI, Math.PI);
    pidFrontLeftTurn.enableContinuousInput(-Math.PI, Math.PI);
    pidFrontLeftTurn.enableContinuousInput(-Math.PI, Math.PI);
    pidFrontLeftTurn.enableContinuousInput(-Math.PI, Math.PI);

    ChassisSpeeds desiredSpeeds = new ChassisSpeeds(xbox.getRawAxis(4)*driveSensitivity, xbox.getRawAxis(3)*driveSensitivity, xbox.getRawAxis(1) * turningSensitivity);
    SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(desiredSpeeds);

    SwerveModuleState frontLeftModule = moduleStates[0];
    SwerveModuleState frontRightModule = moduleStates[1];
    SwerveModuleState backLeftModule = moduleStates[2];
    SwerveModuleState backRightModule = moduleStates[3];

    var frontLeftOptimized = SwerveModuleState.optimize(frontLeftModule, new Rotation2d(frontLeftSteer.getSelectedSensorPosition()/kTurningEncoderRot2Rad));
    var frontRightOptimized = SwerveModuleState.optimize(frontRightModule, new Rotation2d(frontRightSteer.getSelectedSensorPosition()/kTurningEncoderRot2Rad));
    var backLeftOptimized = SwerveModuleState.optimize(backLeftModule, new Rotation2d(backLeftSteer.getSelectedSensorPosition()/kTurningEncoderRot2Rad));
    var backRightOptimized = SwerveModuleState.optimize(backRightModule, new Rotation2d(backRightSteer.getSelectedSensorPosition()/kTurningEncoderRot2Rad));
    
    frontLeftSteer.set(pidFrontLeftTurn.calculate(frontLeftSteer.getSelectedSensorPosition()/kTurningEncoderRot2Rad, frontLeftOptimized.angle.getRadians()));
    frontRightSteer.set(pidFrontRightTurn.calculate(frontRightSteer.getSelectedSensorPosition()/kTurningEncoderRot2Rad, frontRightOptimized.angle.getRadians()));
    backLeftSteer.set(pidBackLeftTurn.calculate(backLeftSteer.getSelectedSensorPosition()/kTurningEncoderRot2Rad, backLeftOptimized.angle.getRadians()));
    backRightSteer.set(pidBackRightTurn.calculate(backRightSteer.getSelectedSensorPosition()/kTurningEncoderRot2Rad, backRightOptimized.angle.getRadians()));


  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

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
    ChassisSpeeds robotSpeeds = new ChassisSpeeds(1.0, 3.0, xbox.getRawAxis(1) * turningSensitivity);
    SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(robotSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, maxSpeed);
    
    SwerveModuleState frontLeft = moduleStates[0];
    SwerveModuleState frontRight = moduleStates[1];
    SwerveModuleState backLeft = moduleStates[2];
    SwerveModuleState backRight = moduleStates[3];

}
}
