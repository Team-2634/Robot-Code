package frc.robot;

import frc.robot.subsystems.DriveTrain;
import frc.robot.commands.arcadeDriveCom;

public class RizzlerRobotContainer {
  // The robot's subsystems and commands are defined here...
  final Constants cont = new Constants();  
  private final DriveTrain Rizzler_DriveTrain = new DriveTrain(cont.leftFrontFX, cont.rightFrontFX, cont.leftBackFX, cont.rightBackFX);
  private final DriveTrain.arcadeDriveSubClass m_robotArcadeDrive = Rizzler_DriveTrain.new arcadeDriveSubClass();
  private final arcadeDriveCom arcadeDrivingCom = new arcadeDriveCom(cont.m_Xstick, m_robotArcadeDrive);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RizzlerRobotContainer() {
    // Configure the button bindings
    m_robotArcadeDrive.setDefaultCommand(arcadeDrivingCom);
  }
}
