package frc.robot;

import frc.robot.subsystems.DriveTrain;
import frc.robot.commands.arcadeDriveCom;

public class FrunkRobotContainer {
  // The robot's subsystems and commands are defined here...
  final Constants cont = new Constants();  
  private final DriveTrain Frunk_DriveTrain = new DriveTrain(cont.leftFrontMax, cont.rightFrontMax, cont.leftBackMax, cont.rightBackMax);
  private final DriveTrain.arcadeDriveSubClass m_robotArcadeDrive = Frunk_DriveTrain.new arcadeDriveSubClass();
  private final arcadeDriveCom arcadeDrivingCom = new arcadeDriveCom(cont.m_Xstick, m_robotArcadeDrive);
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public FrunkRobotContainer() {
    // Configure the button bindings
    m_robotArcadeDrive.setDefaultCommand(arcadeDrivingCom);
  }
}
