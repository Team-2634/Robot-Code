package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.systems.Climber;
import frc.robot.systems.Driver;
import frc.robot.systems.Limelight;
import frc.robot.systems.Shooter;

public class Teleop {
    XboxController xboxDrive = new XboxController(0);
    XboxController xboxArm = new XboxController(0);
    XboxController dev = new XboxController(5);

    TeleopHelper teleopHelper;

    public Teleop(Driver driver, Shooter shooter, Climber climber, AHRS navx, Limelight limelight) {
        teleopHelper = new TeleopHelper(driver, shooter, climber, navx, limelight);
    }
    
    // public double removeDeadzone(int axisInput) {
    //     if (Math.abs(xboxDrive.getRawAxis(axisInput)) < Constants.controllerDeadzone) {
    //         return 0;
    //     }
    //     return xboxDrive.getRawAxis(axisInput);
    // }
    
    public double removeDeadzone(double input) {
        if (Math.abs(input) < Constants.controllerDeadzone) {
            return 0;
        } 
        return input;
    }
    
    public void drive() {
        teleopHelper.drive(-removeDeadzone(xboxDrive.getLeftY()), -removeDeadzone(xboxDrive.getLeftX()), -removeDeadzone(xboxDrive.getRightX()), xboxDrive.getLeftStickButton(), xboxDrive.getXButton() && false, xboxDrive.getXButton());
    }

    public void shoot() {
        teleopHelper.shoot(removeDeadzone(xboxArm.getRightTriggerAxis()));
    }

    public void intake() {
        teleopHelper.intake(removeDeadzone(xboxArm.getLeftTriggerAxis()), xboxArm.getYButton());
    }

    public void arm() {
        teleopHelper.arm(xboxArm.getRightBumper(), xboxArm.getLeftBumper());
    }

    public void climb() {
        teleopHelper.climb(xboxDrive.getAButton(), xboxDrive.getBButton());
    }

    public void panic() {
        teleopHelper.panic(xboxDrive.getRawButton(7));
        SmartDashboard.putBoolean("oops", true);
    }
    //experemental
    public void shootRoutine() {
        teleopHelper.shootRoutine(xboxDrive.getRightTriggerAxis() > 0.5);
    }

    public void armPID() {
        teleopHelper.setArmState(xboxArm.getLeftBumperPressed(), xboxArm.getRightBumperPressed());
    }

    public void limelight(){
        teleopHelper.limelightArmAngle(xboxArm.getXButton());
    }

    // public void targetDetected()){
    //     teleopHelper.detectTarget();
    // }

}
