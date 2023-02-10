package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class arcadeDriveCom extends CommandBase {
  private final XboxController m_Xstick;
  private final DriveTrain.arcadeDriveSubClass m_robotDrive;

  public arcadeDriveCom(DriveTrain.arcadeDriveSubClass robotDrive, XboxController Xstick) {
    m_Xstick = Xstick;
    m_robotDrive = robotDrive;
    // we need to add the requierments of:
    addRequirements(m_robotDrive);
    // HERE BE ERRORS^^^^^^^
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    m_robotDrive.arcadeDriveSub(m_Xstick.getRawAxis(4) * 0.8, m_Xstick.getRawAxis(1) * 0.8);
    // here is where we say take input of XboxController
    // the Id of the controller will always be 0,
    // there is a constant creating for it but here we are just passing in a
    // controller and will set the id/constants in the Container
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
