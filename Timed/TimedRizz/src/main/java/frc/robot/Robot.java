// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;





public class Robot extends TimedRobot {

  boolean True = true;
  private final WPI_TalonFX leftFront = new WPI_TalonFX(1);
  private final WPI_TalonFX leftBack = new WPI_TalonFX(2);
  private final WPI_TalonFX rightFront = new WPI_TalonFX(3);
  private final WPI_TalonFX rightBack = new WPI_TalonFX(4);
  private final MotorControllerGroup m_leftSide = new MotorControllerGroup(leftBack, leftFront);
  private final MotorControllerGroup m_rightSide = new MotorControllerGroup(rightBack, rightFront);
  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftSide, m_rightSide);
  private final CANSparkMax leftintake = new CANSparkMax(0,MotorType.kBrushless);
  private final CANSparkMax rightintake = new CANSparkMax(0,MotorType.kBrushless);
  private final MotorControllerGroup intake = new MotorControllerGroup(leftintake, rightintake);

  private final Timer timer = new Timer();
  private final XboxController m_stick = new XboxController(0);

  private final DoubleSolenoid dSolenoidLeft = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
  private final Compressor compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
  private final double Scale = 250, offset = -25;
  private final AnalogPotentiometer potentiometer = new AnalogPotentiometer(0, Scale, offset);


  @Override
  public void robotInit() {
    double psi = potentiometer.get();
    if (psi <= 119) {
      compressor.enableDigital();
    }else if (psi > 119) {
      compressor.disable();
    }

  }

  @Override
  public void autonomousInit() {
    timer.reset();  
    timer.start();
      }
  @Override
   
  public void autonomousPeriodic() {

    leftFront.setNeutralMode(NeutralMode.Brake);
    leftBack.setNeutralMode(NeutralMode.Brake);
    rightFront.setNeutralMode(NeutralMode.Brake);
    rightBack.setNeutralMode(NeutralMode.Brake);

    double time = timer.get();

  if (time > 0 && time < 4 ) { 
    m_robotDrive.arcadeDrive(0, -0.50);

  } else if (time < 6 && time > 4) { 
    m_robotDrive.arcadeDrive(0.5, 0);

  } else if (time < 8 && time > 6) {
    m_robotDrive.arcadeDrive(0, -0.5);

  } else {
    m_robotDrive.arcadeDrive(0, 0);
  }
  } 
  
  @Override
  public void teleopPeriodic() {

  leftFront.setNeutralMode(NeutralMode.Brake);
  leftBack.setNeutralMode(NeutralMode.Brake);
  rightFront.setNeutralMode(NeutralMode.Brake);
  rightBack.setNeutralMode(NeutralMode.Brake);
  m_robotDrive.arcadeDrive(m_stick.getRawAxis(4)*0.8, m_stick.getRawAxis(1)*0.8);

   if (m_stick.getRawButton(1) == true) {
    dSolenoidLeft.set(Value.kForward);
   }else if (m_stick.getRawButton(2) == true) {
    dSolenoidLeft.set(Value.kReverse);
   }

   if (m_stick.getRawButton(3) == true) {
    intake.set(kDefaultPeriod);
   }
  }
}