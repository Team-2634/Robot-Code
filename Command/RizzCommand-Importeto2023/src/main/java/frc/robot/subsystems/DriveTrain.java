// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.commands.arcadeDriveCom;

public class DriveTrain extends PIDSubsystem {
  /** Creates a new Frunk_DriveTrain. */
  private DifferentialDrive m_robotDrive;
  private MotorControllerGroup m_leftSide;
  private MotorControllerGroup m_rightSide;

  public DriveTrain(MotorController leftFront, MotorController rightFront, MotorController leftBack, MotorController rightBack) {
    super(
        // The PIDController used by the subsystem
        new PIDController(0, 0, 0));

        m_leftSide = new MotorControllerGroup(leftFront, leftBack);
        m_rightSide = new MotorControllerGroup(rightFront, rightBack);
    
        m_robotDrive = new DifferentialDrive(m_leftSide, m_rightSide);
  }

  @Override
  public void periodic() {
  }

  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return 0;
  }

  public class arcadeDriveSubClass {
    public void arcadeDriveSub(double xSpeed, double zRotation) {
      m_robotDrive.arcadeDrive(xSpeed, zRotation);
    }

    public void setDefaultCommand(arcadeDriveCom arcadeDrivingCom) {
    }
  }
}
