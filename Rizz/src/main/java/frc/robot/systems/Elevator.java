package frc.robot.systems;

import com.studica.frc.AHRS;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Solenoid;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;


public class Elevator {

    public final TalonFX armMotor = new TalonFX(Constants.armMotorID); //no ID value yet
    public final TalonFX elevatorMotor = new TalonFX(Constants.elevatorMotorID); //no ID value yet
    private Encoder armEncoder;
    Solenoid solenoid1 = new Solenoid(PneumaticsModuleType.CTREPCM, 0);

    private static final double ENCODER_TICKS_PER_REV = 4096; // Example value
    private static final double ARM_GEAR_RATIO = 2.0; // If arm has a gearbox, adjust this
    private static final double DEGREES_PER_REV = 360.0; 

    public Elevator() {
        armEncoder = new Encoder(0, 1); // Replace with actual encoder ports
    }

    public void elevatorLift(double speedInput) {
        elevatorMotor.set(speedInput);
    }

    public double getArmAngle() {
        return (armEncoder.getDistance() / ENCODER_TICKS_PER_REV) * DEGREES_PER_REV / ARM_GEAR_RATIO;
    }

    public void armAngle(double targetAngle) {

        double currentAngle = getArmAngle(); 
        double speed = 0.2; 
        
        if (currentAngle < targetAngle - 2) { 
            armMotor.set(speed);  // Move arm up
        } 
        else if (currentAngle > targetAngle + 2) { 
            armMotor.set(-speed);  // Move arm down
        } 
        else {
            armMotor.set(0);  // Stop the motor when at target
        }

    }

    public void pneumaticsOpen(Solenoid solenoid) {

        solenoid.set(true);
    }

    public void pneumaticsClose(Solenoid solenoid) {

        solenoid.set(false);
    }
}
