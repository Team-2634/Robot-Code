package frc.robot;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

// this should be everything you neeed here in robot.java
// may want to comment out rizzlerContainer
public class Robot extends TimedRobot {

  FrunkRobotContainer frunkContainer;
  RizzlerRobotContainer rizzlerContainer;
  //LFRobotContainer lordfaarquad;

  @Override
  public void robotInit() {
    frunkContainer = new FrunkRobotContainer();
    //rizzlerContainer = new RizzlerRobotContainer();
  }

  @Override
  public void teleopPeriodic() {
    CommandScheduler.getInstance().run();
  }
}