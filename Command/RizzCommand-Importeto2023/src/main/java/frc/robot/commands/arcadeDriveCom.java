// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class arcadeDriveCom extends CommandBase {
  /** Creates a new arcadeDrive. */
  private final XboxController m_Xstick;
  private final DriveTrain.arcadeDriveSubClass m_robotDrive;

  public arcadeDriveCom(XboxController Xstick, DriveTrain.arcadeDriveSubClass robotDrive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Xstick = Xstick;
    m_robotDrive = robotDrive;
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_robotDrive.arcadeDriveSub(m_Xstick.getRawAxis(4)*0.8, m_Xstick.getRawAxis(1)*0.8);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
