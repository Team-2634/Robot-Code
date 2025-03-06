package frc.robot;

import com.studica.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.systems.Climber;
import frc.robot.systems.Driver;
import frc.robot.systems.Shooter;
public class TeleopHelper {
    
    Driver driver;
    Climber climber;
    AHRS navx;
    LimeLight limelight;

    PIDController autoXPID = new PIDController(Constants.kpAuto, Constants.kiAuto, Constants.kdAuto);
    PIDController autoYPID = new PIDController(Constants.kpAuto, Constants.kiAuto, Constants.kdAuto);
    PIDController autoTurnPID = new PIDController(Constants.kpAutoRotate, Constants.kiAutoRotate, Constants.kdAutoRotate);

    public TeleopHelper(Driver driver, Shooter shooter,Climber climber, AHRS navx, LimeLight limelight) {
        this.driver = driver;
        this.climber = climber;
        this.navx = navx;
        this.limelight = limelight;
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

    public void autoDriveByDistance(double distanceX, double distanceY) {
        double[] distanceFieldOriented = Driver.fieldOrient(distanceX, distanceY, navx);
        double fieldDistanceX = distanceFieldOriented[0];
        double fieldDistanceY = distanceFieldOriented[1]; 

        double[] displacementFieldOriented = Driver.fieldOrient(navx.getDisplacementX(), navx.getDisplacementY(), navx);
        double currentDisplacementX = displacementFieldOriented[0];
        double currentDisplacementY = displacementFieldOriented[1]; 
        
        double xSpeed = autoXPID.calculate(currentDisplacementX, fieldDistanceX);
        double ySpeed = autoYPID.calculate(currentDisplacementY, fieldDistanceY);
        driver.swerveDrive(xSpeed, ySpeed, 0);
    }

}
