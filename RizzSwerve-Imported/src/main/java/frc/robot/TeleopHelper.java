package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.systems.Climber;
import frc.robot.systems.Driver;
import frc.robot.systems.Shooter;

public class TeleopHelper {
    
    Driver driver;
    Shooter shooter;
    Climber climber;

    public TeleopHelper(Driver driver, Shooter shooter, Climber climber) {
        this.driver = driver;
        this.shooter = shooter;
        this.climber = climber;
    }
    
    

    final XboxController xbox = new XboxController(0);

    public void drive(double XSpeed, double YSpeed, double TurnSpeed) {
        double XSpeedField = XSpeed * Math.cos(botYaw_angleRad) - YSpeed * Math.sin(botYaw_angleRad);
        double YSpeedField = XSpeed * Math.sin(botYaw_angleRad) + YSpeed * Math.cos(botYaw_angleRad);
        driver.swerveDrive(XSpeedField * Constants.XdriveSensitivity, YSpeedField * Constants.YdriveSensitivity, TurnSpeed * Constants.turningSensitivity);
    }

    public double removeDeadzone(int axisInput) {
        if (Math.abs(xbox.getRawAxis(axisInput)) < Constants.controllerDeadzone) {
            return 0;
        }
        return xbox.getRawAxis(axisInput);
    }


}
