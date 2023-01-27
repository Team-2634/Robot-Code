// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class ArcadeDriveSub extends SubsystemBase {

  private final WPI_TalonFX leftFront = new WPI_TalonFX(1);
  private final WPI_TalonFX leftBack = new WPI_TalonFX(2);
  private final WPI_TalonFX rightFront = new WPI_TalonFX(3);
  private final WPI_TalonFX rightBack = new WPI_TalonFX(4);
  private final MotorControllerGroup m_leftSide = new MotorControllerGroup(leftFront, leftBack);
  private final MotorControllerGroup m_rightSide = new MotorControllerGroup(rightFront, rightBack);

  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftSide, m_rightSide);
  

  public ArcadeDriveSub(){
    
  }

  /** Creates a new ArcadeDriveSub. */
  public void arcadeDrive(double xSpeed, double zRotation) {
      m_robotDrive.arcadeDrive(xSpeed, zRotation);
   
  }


  @Override
  public void periodic() {
}
}