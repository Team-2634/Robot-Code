package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.systems.Climber;
import frc.robot.systems.Driver;
import frc.robot.systems.Shooter;

public class TeleopHelper {
    
    Driver driver;
    Shooter shooter;
    Climber climber;
    AHRS navx;

    public TeleopHelper(Driver driver, Shooter shooter, Climber climber, AHRS navx) {
        this.driver = driver;
        this.shooter = shooter;
        this.climber = climber;
        this.navx = navx;
    }
    
    

    final XboxController xbox = new XboxController(0);

    public void drive(double XSpeed, double YSpeed, double TurnSpeed) {
        double[] speedsFieldOriented = driver.fieldOrient(XSpeed, YSpeed);
        XSpeed = speedsFieldOriented[0] * Constants.XdriveSensitivity;
        YSpeed = speedsFieldOriented[1] * Constants.YdriveSensitivity;
        TurnSpeed = TurnSpeed * Constants.turningSensitivity;
        
        driver.swerveDrive(XSpeed, YSpeed, TurnSpeed);
    }

    /**
     * Get axis and remove deadzone from controller input
     * @param axisInput axis ID
     * @return
     */
    public double getAxisValue(int axisInput) {
        if (Math.abs(xbox.getRawAxis(axisInput)) < Constants.controllerDeadzone) {
            return 0;
        }
        return xbox.getRawAxis(axisInput);
    }


}
