package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.XboxController;

import frc.robot.systems.Climber;
import frc.robot.systems.Driver;
import frc.robot.systems.Shooter;

public class Teleop {
    final XboxController xbox = new XboxController(0);
    
    TeleopHelper teleopHelperObject;
    
    public Teleop(Driver driver, Shooter shooter, Climber climber, AHRS navx) {
        teleopHelperObject = new TeleopHelper(driver, shooter, climber, navx);
    }
    
    
    
    public void drive() {
        teleopHelperObject.drive(teleopHelperObject.removeDeadzone(1), teleopHelperObject.removeDeadzone(0), teleopHelperObject.removeDeadzone(4));
    }

    public void shooter() {
        if (xbox.getRightBumperPressed() == true){
            teleopHelperObject.Shoot();
        }
        if (xbox.getRightBumper() == true){
            teleopHelperObject.Shoot();
        }
        
        if (xbox.getLeftBumperPressed() == true){
            teleopHelperObject.Unstick();
        }
        
        if (xbox.getAButton() == true){
            teleopHelperObject.RotateArmCL();
        }else{
            teleopHelperObject.RotateStop();
        }

        if (xbox.getXButton()){
            teleopHelperObject.RotateArmCCL();
        }else{
            teleopHelperObject.RotateStop();
        }
        
        teleopHelperObject.pickup();


    }

}
