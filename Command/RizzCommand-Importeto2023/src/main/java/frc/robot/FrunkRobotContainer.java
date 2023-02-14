package frc.robot;

import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.RobotArcadeDriveSub;
import frc.robot.commands.arcadeDriveCom;

public class FrunkRobotContainer {
  // The robot's subsystems and commands are defined here...
  final Constants cont = new Constants();  
  private final DriveTrain Frunk_DriveTrain = new DriveTrain(cont.leftFrontMax, cont.rightFrontMax, cont.leftBackMax, cont.rightBackMax);
  private final RobotArcadeDriveSub m_robotArcadeDrive = new RobotArcadeDriveSub();
  private final arcadeDriveCom arcadeDrivingCom = new arcadeDriveCom(m_robotArcadeDrive, cont.m_Xstick);
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public FrunkRobotContainer() {
    //here we need to set the arcade drive command as the default drive
    m_robotArcadeDrive.setDefaultCommand(arcadeDrivingCom);
  }
}