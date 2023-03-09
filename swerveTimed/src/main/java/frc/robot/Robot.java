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

  double driveSensitivity = 0.8; //do not change above 1
  double turningSensitivity = 0.8;
  double maxSpeedMpS = 10; // metres/sec

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
    
    frontLeftSteer.setSelectedSensorPosition(0);
    frontRightSteer.setSelectedSensorPosition(0);
    backLeftSteer.setSelectedSensorPosition(0);
    backRightSteer.setSelectedSensorPosition(0);
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
  
  public void swerveDrive() {
    
    //make pids treat values pi radians and -pi radians as the same and have them loop around
    pidFrontLeftTurn.enableContinuousInput(-Math.PI, Math.PI);
    pidFrontRightTurn.enableContinuousInput(-Math.PI, Math.PI);
    pidBackLeftTurn.enableContinuousInput(-Math.PI, Math.PI);
    pidBackRightTurn.enableContinuousInput(-Math.PI, Math.PI);

    //controller inputs are multiplied by max speed to return a fraction of maximum speed and modified further by sensitivity
    //max controller value of 1 returns maximum speed achivable by the robot before being reduced by sensitivity
    //turning is black magic
    double desiredXSpeed = xbox.getRawAxis(4) * maxSpeedMpS * driveSensitivity;
    double desiredYSpeed = xbox.getRawAxis(3) * maxSpeedMpS * driveSensitivity;
    double desiredTurnSpeed = xbox.getRawAxis(1) * turningSensitivity;
    ChassisSpeeds desiredSpeeds = new ChassisSpeeds(desiredXSpeed, desiredYSpeed, desiredTurnSpeed);
    
    //make desiredSpeeds into speeds and angles for each module
    SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(desiredSpeeds);

    //normalize module values to remove impossible speed values
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, maxSpeedMpS);
    
    SwerveModuleState frontLeftModule = moduleStates[0];
    SwerveModuleState frontRightModule = moduleStates[1];
    SwerveModuleState backLeftModule = moduleStates[2];
    SwerveModuleState backRightModule = moduleStates[3];

    //optimize wheel angles (ex. wheel is at 359deg and needs to go to 1deg. wheel will now go 2deg instead of 358deg)
    var frontLeftOptimized = SwerveModuleState.optimize(frontLeftModule, new Rotation2d(frontLeftSteer.getSelectedSensorPosition()/kTurningEncoderRot2Rad));
    var frontRightOptimized = SwerveModuleState.optimize(frontRightModule, new Rotation2d(frontRightSteer.getSelectedSensorPosition()/kTurningEncoderRot2Rad));
    var backLeftOptimized = SwerveModuleState.optimize(backLeftModule, new Rotation2d(backLeftSteer.getSelectedSensorPosition()/kTurningEncoderRot2Rad));
    var backRightOptimized = SwerveModuleState.optimize(backRightModule, new Rotation2d(backRightSteer.getSelectedSensorPosition()/kTurningEncoderRot2Rad));
    
    //set steer motor power to the pid output of current position in radians and desired position in radians
    frontLeftSteer.set(pidFrontLeftTurn.calculate(frontLeftSteer.getSelectedSensorPosition()/kTurningEncoderRot2Rad, frontLeftOptimized.angle.getRadians()));
    frontRightSteer.set(pidFrontRightTurn.calculate(frontRightSteer.getSelectedSensorPosition()/kTurningEncoderRot2Rad, frontRightOptimized.angle.getRadians()));
    backLeftSteer.set(pidBackLeftTurn.calculate(backLeftSteer.getSelectedSensorPosition()/kTurningEncoderRot2Rad, backLeftOptimized.angle.getRadians()));
    backRightSteer.set(pidBackRightTurn.calculate(backRightSteer.getSelectedSensorPosition()/kTurningEncoderRot2Rad, backRightOptimized.angle.getRadians()));

    //set drive power to desired speed div max speed to get value between 0 and 1
    frontLeftDrive.set(frontLeftModule.speedMetersPerSecond/maxSpeedMpS);
    frontRightDrive.set(frontRightModule.speedMetersPerSecond/maxSpeedMpS);
    backLeftDrive.set(backLeftModule.speedMetersPerSecond/maxSpeedMpS);
    backRightDrive.set(backRightModule.speedMetersPerSecond/maxSpeedMpS);
  }

  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    swerveDrive();
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
