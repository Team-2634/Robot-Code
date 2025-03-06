package frc.robot;

import com.studica.frc.AHRS;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.systems.Climber;
import frc.robot.systems.Driver;
import frc.robot.systems.Shooter;
public class TeleopHelper {
    
    Driver driver;
    Climber climber;
    AHRS navx;
    public TeleopHelper(Driver driver, Shooter shooter,Climber climber, AHRS navx) {
        this.driver = driver;
        this.climber = climber;
        this.navx = navx;
    }

    final XboxController xbox1 = new XboxController(0);
    public void drive(double XSpeed, double YSpeed, double TurnSpeed) {
        double[] speedsFieldOriented = Driver.fieldOrient(XSpeed, YSpeed, navx);
        XSpeed = speedsFieldOriented[0] * Constants.XdriveSensitivity;
        YSpeed = speedsFieldOriented[1] * Constants.YdriveSensitivity;
        TurnSpeed = TurnSpeed * Constants.turningSensitivity;
        
        driver.swerveDrive(XSpeed, YSpeed, TurnSpeed);
    }

    public double removeDeadzone(int axisInput) {
        if (Math.abs(xbox1.getRawAxis(axisInput)) < Constants.controllerDeadzone) {
            return 0;
        }
        return xbox1.getRawAxis(axisInput);


    }


}
