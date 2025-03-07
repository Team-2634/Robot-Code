package frc.robot.systems;

import com.studica.frc.AHRS;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Arm {

    public final TalonFX armMotor = new TalonFX(Constants.armMotorID); //no ID value yet
    DoubleSolenoid solenoid1 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 1);
    private final Compressor compressor1 = new Compressor(PneumaticsModuleType.CTREPCM);
    private final PIDController armPID = new PIDController(getArmAngle(), getArmAngle(), getArmAngle());

    public void armInitiallize() {
        armMotor.setNeutralMode(NeutralModeValue.Brake);
        compressor1.enableDigital();
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
