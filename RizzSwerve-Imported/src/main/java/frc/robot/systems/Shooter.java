package frc.robot.systems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.Timer;

public class Shooter {
    private final TalonFX ShooterMotor_L = new TalonFX(0);
    private final TalonFX ShooterMotor_R = new TalonFX(0);
    
    private final TalonFX ArmMotor_L = new TalonFX(0);
    private final TalonFX ArmMotor_R = new TalonFX(0);

    private final TalonFX PickUpMotor = new TalonFX(0);
    
    Timer LaunchTimer;
    public void collectNote(double input){

        PickUpMotor.set(input);
    }
   
    public void RotateArm(double value){
        ArmMotor_L.set(value);
        ArmMotor_R.set(value);
    }

    public void shootNote(double seconds, double speed) {

        LaunchTimer.reset();
        LaunchTimer.start();
        if (LaunchTimer.get() > seconds){
            ShooterMotor_L.set(speed);
            ShooterMotor_R.set(speed);
        }
        
        ShooterMotor_L.set(0.0);
        ShooterMotor_R.set(0.0);

    }

}
