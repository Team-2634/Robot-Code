// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private MotorController leftIntake;
  private MotorController rightIntake;

  public Intake(MotorController leftIntake, MotorController rightIntake) {
    this.leftIntake = leftIntake;
    this.rightIntake = rightIntake;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
