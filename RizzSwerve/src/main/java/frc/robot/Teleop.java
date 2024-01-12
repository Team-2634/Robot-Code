package frc.robot;

public class Teleop {
    TeleopHelper teleopHelper;
    public Teleop(Driver driver, Shooter shooter, Climber climber) {
        teleopHelper = new TeleopHelper(driver, shooter, climber)
    }
    
}
