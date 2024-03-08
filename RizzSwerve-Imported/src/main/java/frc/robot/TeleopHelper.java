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

    public void drive(double XSpeed, double YSpeed, double TurnSpeed, boolean modSpeed) {
        if (modSpeed) {
            XSpeed /= 3;
            YSpeed /= 3;
            TurnSpeed /= 3;
        }
        double[] speedsFieldOriented = driver.fieldOrient(XSpeed, YSpeed);
        XSpeed = speedsFieldOriented[0] * Constants.XdriveSensitivity;
        YSpeed = speedsFieldOriented[1] * Constants.YdriveSensitivity;
        TurnSpeed = TurnSpeed * Constants.turningSensitivity;
        
        driver.swerveDrive(XSpeed, YSpeed, TurnSpeed);
    }

    public void shoot(double input) {
        shooter.shootNote(input);
        // if (input) {
        //     shooter.shootNote(Constants.shootSpeed);
        // } else {
        //     shooter.shootNote(0);
        // }
    } 

    public void intake(double input, boolean yButton) {
        
        // if (bButton) {
        //     shooter.collectNote(Constants.intakeSpeed);
         if (yButton) {
            shooter.collectNote(0.2);
        } else {
            shooter.collectNote(-input/3);
        }
        // else {
        //     shooter.collectNote(0);
        // }
    }

    public void arm(boolean up, boolean down) {
        if (up) {
            shooter.moveArm(Constants.armSpeed);
        } else if (down) {
            shooter.moveArm(-Constants.armSpeed);
        } else {
            shooter.moveArm(0);
        }
    }

    public void climb(boolean aButton, boolean bButton) {
        if (aButton) {
            climber.climb(Constants.climbSpeed);
        } else if (bButton) {
            climber.climb(-Constants.climbSpeed);
        } else {
            climber.climb(0);
        }
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
