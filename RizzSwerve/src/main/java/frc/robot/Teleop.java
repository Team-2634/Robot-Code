package frc.robot;

public class Teleop {
    TeleopHelper teleopHelper;
    public Teleop(Driver driver, Shooter shooter, Climber climber) {
        teleopHelper = new TeleopHelper(driver, shooter, climber)
    }
    
    final XboxController xbox = new XboxController(0);
    
    double contXSpeed = TeleopHelper.removeDeadzone(1) * Constants.XdriveSensitivity;
    double contYSpeed = TeleopHelper.removeDeadzone(0) * Constants.YdriveSensitivity;
    double contTurnSpeed = TeleopHelper.removeDeadzone(4) * Constants.turningSensitivity;

    public void drive() {
        TeleopHelper.drive(TeleopHelper.removeDeadzone(1), TeleopHelper.removeDeadzone(0), TeleopHelper.removeDeadzone(4))
    }

    

    




}
