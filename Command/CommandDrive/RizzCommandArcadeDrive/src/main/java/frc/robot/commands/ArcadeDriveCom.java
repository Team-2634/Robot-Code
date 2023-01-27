// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArcadeDriveSub;

public class ArcadeDriveCom extends CommandBase {
  /** Creates a new ArcadeDriveCom.  and define subsys*/

  private final ArcadeDriveSub m_robotDrive;
  private final XboxController m_Xstick;

  public ArcadeDriveCom(ArcadeDriveSub robotDrive, XboxController Xstick) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_robotDrive = robotDrive;
    m_Xstick = Xstick;
    addRequirements(m_robotDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { 
    m_robotDrive.arcadeDrive(m_Xstick.getRawAxis(4)*0.8, m_Xstick.getRawAxis(1)*0.8);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}