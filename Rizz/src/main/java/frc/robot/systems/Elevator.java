package frc.robot.systems;


import com.studica.frc.AHRS;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Elevator{

    public final TalonFX armMotor = new TalonFX(Constants.armMotorID); //no ID value yet
    public final TalonFX elevatorMotor = new TalonFX(Constants.elevatorMotorID); //no ID value yet
    private Encoder armEncoder;
    private Encoder elevatorEncoder;
    
    private final Compressor compressor1 = new Compressor(PneumaticsModuleType.CTREPCM);
    DoubleSolenoid solenoid1 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 1);
    public Elevator() {
        armEncoder = new Encoder(0, 1); // Replace with actual encoder ports
       
    }

    public void elevatorInitiallize(){
        elevatorMotor.setNeutralMode(NeutralModeValue.Brake);
        armMotor.setNeutralMode(NeutralModeValue.Brake);
        compressor1.enableDigital();
    }
    public void elevatorEncoderReset(){
        elevatorEncoder.reset();
        armEncoder.reset();
    }

    public double getElevatorHeight() {
        double rotations = elevatorEncoder.getDistance() / Constants.ENCODER_TICKS_PER_REV;
        return rotations * Constants.PULLEY_CIRCUMFERENCE;
    }

    public void elevatorLift(double speedInput) {
        elevatorMotor.set(speedInput);
    }

    public void moveToL1() { moveToHeight(Constants.L1_HEIGHT); }
    public void moveToL2() { moveToHeight(Constants.L2_HEIGHT); }
    public void moveToL3() { moveToHeight(Constants.L3_HEIGHT); }
    public void moveToL4() { moveToHeight(Constants.L4_HEIGHT); }

    public void moveToHeight(double targetHeight) {
        double currentHeight = getElevatorHeight();
        double speed = 0.1; // Adjust speed as needed

        if (currentHeight < targetHeight - 0.02) {
            elevatorMotor.set(speed); // Move up
        } else if (currentHeight > targetHeight + 0.02) {
            elevatorMotor.set(-speed); // Move down
        } else {
            elevatorMotor.set(0); // Stop at target
        }
    }

    public double getArmAngle() {
        double rotations = armMotor.getPosition().getValueAsDouble(); 
        return (rotations / Constants.ARM_GEAR_RATIO) * Constants.DEGREES_PER_REV;  
    }

    public void armAngle(double targetAngle) {
        double currentAngle = getArmAngle(); 
        double speed = 0.2;  //arm speed
        
        targetAngle = Math.max(0, Math.min(targetAngle, 90));
    
        if (currentAngle < targetAngle - 2) { 
            armMotor.set(speed);  
        } 
        else if (currentAngle > targetAngle + 2) { 
            armMotor.set(-speed);  
        } 
        else {
            armMotor.set(0); 
        }
    }

    public void openClaw() {
        solenoid1.set(Value.kForward);

    }

    public void closeClaw() {

        solenoid1.set(Value.kReverse);
    }
    
        
}