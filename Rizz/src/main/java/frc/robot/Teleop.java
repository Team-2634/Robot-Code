package frc.robot;

//import com.kauailabs.navx.frc.AHRS;
import com.studica.frc.AHRS;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.systems.Climber;
import frc.robot.systems.Driver;
import frc.robot.systems.Elevator;
import frc.robot.systems.Arm;


public class Teleop {

    XboxController xboxDrive = new XboxController(0);
    XboxController xboxElevator = new XboxController(1);

    TeleopHelper teleopHelper;
    public Teleop(Driver driver, Climber climber, AHRS navx, Elevator elevator, Arm arm) {
        teleopHelper = new TeleopHelper(driver, climber, navx, elevator, arm);
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
        xboxDrive.getXButton());
    }

    public void elevatorControl() {

        teleopHelper.elevatorControl(
            xboxElevator.getRightTriggerAxis(),
            xboxElevator.getLeftTriggerAxis()
        );

        teleopHelper.moveArm(xboxDrive);
    }
}
