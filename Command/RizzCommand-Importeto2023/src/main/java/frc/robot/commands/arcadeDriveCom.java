// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class arcadeDriveCom extends CommandBase {
  /** Creates a new arcadeDrive. */
  private final XboxController m_Xstick;
  private final Constants cont= new Constants();
  private final MotorControllerGroup m_leftSide = new MotorControllerGroup(cont.leftFrontMax, cont.leftBackMax);
  private final MotorControllerGroup m_rightSide = new MotorControllerGroup(cont.rightFrontMax, cont.rightBackMax);
  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftSide, m_rightSide);
  
  public arcadeDriveCom(DriveTrain robotDrive, XboxController Xstick) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Xstick = Xstick;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

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
