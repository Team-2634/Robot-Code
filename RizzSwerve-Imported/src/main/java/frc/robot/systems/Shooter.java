package frc.robot.systems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.Timer;

public class Shooter {
    private final TalonFX ShooterMotor_FL = new TalonFX(0);
    private final TalonFX ShooterMotor_FR = new TalonFX(0);
    private final TalonFX ShooterMotor_BL = new TalonFX(0);
    private final TalonFX ShooterMotor_BR = new TalonFX(0);

    private final TalonFX PickUpMotor = new TalonFX(0);
    
    Timer LaunchTimer;
    public void collectNote(double input){

        PickUpMotor.set(input);
    }
   
    
    public void shootNote(double seconds, double speed) {
        ShooterMotor_FL.set(speed);
        ShooterMotor_FR.set(speed);
        
        LaunchTimer.reset();
        LaunchTimer.start();
        if (LaunchTimer.get() > seconds){
            ShooterMotor_BL.set(speed);
            ShooterMotor_BR.set(speed);
        }
        
        ShooterMotor_FL.set(0.0);
        ShooterMotor_FR.set(0.0);
        ShooterMotor_BL.set(0.0);
        ShooterMotor_BR.set(0.0);

    }

}
