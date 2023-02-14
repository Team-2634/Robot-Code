// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RobotArcadeDriveSub extends SubsystemBase {
  /** Creates a new RobotArcadeDriveSub. */
  DifferentialDrive m_robotDrive = DriveTrain.getRobotDrive();

  public void arcadeDrive(double xSpeed, double zRotation) {
    m_robotDrive.arcadeDrive(xSpeed, zRotation);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
