// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//NRE POWER SDGSDKHFSD!!!!!!!!!!!!!!!!!!/
// 2048 CPR for talons
package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

public class Robot extends TimedRobot {

  boolean True = true;
  private final WPI_TalonFX leftFront = new WPI_TalonFX(1);
  private final WPI_TalonFX leftBack = new WPI_TalonFX(2);
  private final WPI_TalonFX rightFront = new WPI_TalonFX(3);
  private final WPI_TalonFX rightBack = new WPI_TalonFX(4);

  private final MotorControllerGroup m_leftSide = new MotorControllerGroup(leftBack, leftFront);
  private final MotorControllerGroup m_rightSide = new MotorControllerGroup(rightBack, rightFront);

  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftSide, m_rightSide);

  private final CANSparkMax leftintake = new CANSparkMax(2, MotorType.kBrushless);
  private final CANSparkMax rightintake = new CANSparkMax(7, MotorType.kBrushless);

  private final MotorControllerGroup intake = new MotorControllerGroup(leftintake, rightintake);

  private final Timer timer = new Timer();

  private final XboxController m_stick = new XboxController(0);

  private final DoubleSolenoid dSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
  private final Compressor compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
  
  private final double Scale = 250, offset = -25;
  private final AnalogPotentiometer potentiometer = new AnalogPotentiometer(0, Scale, offset);

  private AHRS navx;  
  
 public Value DSstatus;
 public boolean DSbool;

  public Robot() {
      
    try {
      navx = new AHRS(SPI.Port.kMXP);
    } catch (Exception e) {
      System.out.println(e.getMessage());
    }
    SmartDashboard.putBoolean("Solenoid Status(open = true): ", DSbool);
  }


  @Override
  public void robotInit() {
    double psi = potentiometer.get();
    if (psi <= 119) {
      compressor.enableDigital();
    } else if (psi > 119) {
      compressor.disable();
    }
  }

  @Override
  public void autonomousInit() {
    navx.reset();
    timer.reset();
    timer.start();
  }

  @Override
   
  public void autonomousPeriodic() {
    double robotPitch = navx.getPitch();
    
    m_robotDrive.isSafetyEnabled();

    SmartDashboard.putData(navx);
    SmartDashboard.putNumber("NAVXANGLE", robotPitch);

    leftFront.setNeutralMode(NeutralMode.Brake);
    leftBack.setNeutralMode(NeutralMode.Brake);
    rightFront.setNeutralMode(NeutralMode.Brake);
    rightBack.setNeutralMode(NeutralMode.Brake);



/*
  if (time > 0 && time < 4 ) { 
    m_robotDrive.arcadeDrive(0, -0.50);

  } else if (time < 6 && time > 4) { 
    m_robotDrive.arcadeDrive(0.5, 0);

  } else if (time < 8 && time > 6) {
    m_robotDrive.arcadeDrive(0, -0.5);

  } else {
    m_robotDrive.arcadeDrive(0, 0);
  } */
  
  RobotAngle currentPitch = getPitch(robotPitch);
  double fastLean = 0.40;
  double slowLean = 0.20;
  if (currentPitch == RobotAngle.Balanced) {
    m_robotDrive.arcadeDrive(0,0);
  }
  else if (currentPitch == RobotAngle.Forward){
    m_robotDrive.arcadeDrive(0, fastLean);
  }
  else if (currentPitch == RobotAngle.leaningForward) {
    m_robotDrive.arcadeDrive(0, slowLean);
  }
  else if (currentPitch == RobotAngle.Backward){
    m_robotDrive.arcadeDrive(0, -fastLean);
  }
  else if (currentPitch == RobotAngle.leaningBackward){
    m_robotDrive.arcadeDrive(0, -slowLean);
  }
  }

  enum RobotAngle {
    Forward,
    leaningForward,
    Balanced,
    leaningBackward,
    Backward,
    unset
  }

  // 0.7 1.5 -0.25 nuet
  // 16 back
  // -13.25 front

  double tolerance = 2;
  double tiltBack = 15;
  double tiltFwd = -12;
  double bal = 0;

  public RobotAngle getPitch(double robotPitch) {
    RobotAngle result = RobotAngle.unset;
    if (Math.abs(robotPitch) < tolerance) {
      result = RobotAngle.Balanced;
    } else if (robotPitch > tiltBack) {
      result = RobotAngle.Backward;
    } else if (robotPitch < tiltBack && robotPitch > tolerance) {
      result = RobotAngle.leaningBackward;
    } else if (robotPitch > tiltFwd && robotPitch < -tolerance) {
      result = RobotAngle.leaningForward;
    } else if (robotPitch < tiltFwd) {
      result = RobotAngle.Forward;
    }
    SmartDashboard.putString("Robot Pitch", result.toString());
    return result;
  }

@Override
public void teleopInit() {
  timer.reset();
  timer.start();
}

  @Override
  public void teleopPeriodic() {    
    double time = timer.get();
    DSstatus = dSolenoid.get();
       
    /*
     * if time >= 15 sec
     * get dsol state
     * switch 
     * then switch back
     * reset the timer
     */
  if(time >= 15){
    if(DSstatus==Value.kReverse){
      dSolenoid.set(Value.kForward);
      dSolenoid.set(Value.kReverse);
    }else if(DSstatus==Value.kForward){
      dSolenoid.set(Value.kReverse);
      dSolenoid.set(Value.kForward);
    }
    timer.reset();
  }

  //if ^^^^^^^^^^^^ no work try dis below \/
    /*
  if (DSstatus == Value.kForward ) {
    DSbool = true;
  } else if (DSstatus == Value.kReverse ) {
    DSbool = false;
  }

if(time >= 15){
  if (DSbool = true){
    dSolenoid.set(Value.kReverse);
    dSolenoid.set(Value.kForward);
  } else if (DSbool = false) {
    dSolenoid.set(Value.kForward);
    dSolenoid.set(Value.kReverse);
  }
  timer.reset();
}
*/
    
    double l_speed = m_leftSide.get();
    double r_speed = m_rightSide.get();
    double pItch = navx.getPitch();
    SmartDashboard.putData(navx);
    SmartDashboard.putNumber("NAVXANGLE", pItch);
    SmartDashboard.putNumber("Speed Left", l_speed);
    SmartDashboard.putNumber("SPeed Right", r_speed);

    leftFront.setNeutralMode(NeutralMode.Brake);
    leftBack.setNeutralMode(NeutralMode.Brake);
    rightFront.setNeutralMode(NeutralMode.Brake);
    rightBack.setNeutralMode(NeutralMode.Brake);
    m_robotDrive.arcadeDrive(m_stick.getRawAxis(4) * 0.8, m_stick.getRawAxis(1) * 0.8);

    if (m_stick.getRawButton(1) == true) {
      dSolenoid.set(Value.kForward);
    } else if (m_stick.getRawButton(2) == true) {
      dSolenoid.set(Value.kReverse);
    }

    if (m_stick.getRawButton(6) == true) {
      rightintake.set(-1);
      leftintake.set(1);
    } else if (m_stick.getRawButton(5) == true) {
      leftintake.set(-1);
      rightintake.set(1);
    } else {
      leftintake.set(0);
      rightintake.set(0);
    }
  }
}