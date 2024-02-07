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
        double currentYawRadians = Math.toRadians(navx.getYaw());
        double XSpeedField = XSpeed * Math.cos(currentYawRadians) - YSpeed * Math.sin(currentYawRadians);
        double YSpeedField = XSpeed * Math.sin(currentYawRadians) + YSpeed * Math.cos(currentYawRadians);
        driver.swerveDrive(XSpeedField * Constants.XdriveSensitivity, YSpeedField * Constants.YdriveSensitivity, TurnSpeed * Constants.turningSensitivity);
    }

    public double removeDeadzone(int axisInput) {
        if (Math.abs(xbox.getRawAxis(axisInput)) < Constants.controllerDeadzone) {
            return 0;
        }
        return xbox.getRawAxis(axisInput);
    }

    public void Shoot(){
        shooter.shootNote(5.0, Constants.ShooterMotorSpeed);
    }

    public void Unstick(){
        shooter.shootNote(1.0, Constants.ShooterMotorSpeed * 2);
    }

    public void pickup(){
        shooter.collectNote(xbox.getRightTriggerAxis());
    }

    public void RotateArmCL(){
        shooter.RotateArm(1.0);
    }

    public void RotateArmCCL(){
        shooter.RotateArm(-1.0);
    }

    public void RotateStop(){
        shooter.RotateArm(0.0);
    }
}

