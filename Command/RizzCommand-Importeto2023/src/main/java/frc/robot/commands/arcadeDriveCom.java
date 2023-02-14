package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RobotArcadeDriveSub;

public class arcadeDriveCom extends CommandBase {
  private final XboxController m_Xstick;
  private final RobotArcadeDriveSub m_robotDrive;

  public arcadeDriveCom(RobotArcadeDriveSub robotDrive, XboxController Xstick) {
    m_Xstick = Xstick;
    m_robotDrive = robotDrive;
    addRequirements(m_robotDrive);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    m_robotDrive.arcadeDrive(m_Xstick.getRawAxis(4) * 0.8, m_Xstick.getRawAxis(1) * 0.8);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
