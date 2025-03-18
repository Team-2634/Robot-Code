package frc.robot;

//import com.kauailabs.navx.frc.AHRS;
import com.studica.frc.AHRS;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.systems.Climber;
import frc.robot.systems.Driver;
import frc.robot.systems.Elevator;
import frc.robot.systems.Arm;
import frc.robot.systems.LimeLight;

public class Teleop {

    XboxController xboxDrive = new XboxController(0);
    XboxController xboxElevator = new XboxController(1);

    TeleopHelper teleopHelper;
    public Teleop(Driver driver, Climber climber, AHRS navx, Elevator elevator, Arm arm, LimeLight limelight) {
        teleopHelper = new TeleopHelper(driver, climber, navx, elevator, arm, limelight);
    }

    public double removeDeadzone(double input) {
        if (Math.abs(input) < Constants.controllerDeadzone) {
            return 0;
        } 
        return input;
    }
    
    public void drive() {
        teleopHelper.drive(
        -removeDeadzone(xboxDrive.getLeftY()), 
        -removeDeadzone(xboxDrive.getLeftX()), 
        -removeDeadzone(xboxDrive.getRightX()), 
        xboxDrive.getXButton(),
        xboxDrive.getAButton());
    }

    public void elevator() {
        teleopHelper.moveElevator(xboxElevator);
    }

    // public void arm() {
    //     teleopHelper.moveArm(xboxDrive);
    // }

    public void clamp() {
        teleopHelper.moveClamp(xboxElevator);
    }

    public void armTest() {
        teleopHelper.armTestTest(xboxElevator);
    }

    public void limelightFunctions(){
        teleopHelper.rotateAlignToAprilTag(xboxDrive);
        teleopHelper.moveToAprilTag(xboxDrive);
    }

    public void elevatorArm(){
        teleopHelper.moveElevatorArm(xboxElevator);
    }

    public void climber(){
        teleopHelper.moveClimber(xboxElevator);
    }

    // public void arm() {

    //     teleopHelper.arm(
    //         xboxElevator.getAButtonPressed(),
    //         xboxElevator.getBButtonPressed(),
    //         xboxElevator.getXButtonPressed(),
    //         xboxElevator.getYButtonPressed()
    //     );

    // }
}