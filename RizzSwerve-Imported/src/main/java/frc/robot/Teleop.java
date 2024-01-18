package frc.robot;

public class Teleop {
    TeleopHelper teleopHelper;
    public Teleop(Driver driver, Shooter shooter, Climber climber) {
        teleopHelper = new TeleopHelper(driver, shooter, climber)
    }
    
    
    
    public void drive() {
        TeleopHelper.drive(TeleopHelper.removeDeadzone(1), TeleopHelper.removeDeadzone(0), TeleopHelper.removeDeadzone(4))
    }

    

    




}
