package frc.robot;

//import com.kauailabs.navx.frc.AHRS;
import com.studica.frc.AHRS;

// import edu.wpi.first.wpilibj.XboxController;
import frc.robot.systems.Climber;
import frc.robot.systems.Driver;
import frc.robot.systems.Elevator;


public class Teleop {
    TeleopHelper teleopHelper;
    public Teleop(Driver driver, Climber climber, AHRS navx, Elevator elevator) {
        teleopHelper = new TeleopHelper(driver, climber, navx, elevator);
    }
    
    public void drive() {
        teleopHelper.drive(teleopHelper.removeDeadzone(1), teleopHelper.removeDeadzone(0), teleopHelper.removeDeadzone(4));
        
    }

    public void elevatorControl() {

        //teleopHelper.liftElevatorFromBumper(teleopHelper.xboxElevatorInput);
        teleopHelper.elevatorControl(teleopHelper.xbox2);
        
    }

    




}
