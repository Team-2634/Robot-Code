package frc.robot.systems;

import com.studica.frc.AHRS;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Arm {

    public final TalonFX armMotor = new TalonFX(Constants.armMotorID); //no ID value yet
    DoubleSolenoid solenoid1 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 1);
    private final Compressor compressor1 = new Compressor(PneumaticsModuleType.CTREPCM);
    private final PIDController armPID = new PIDController(Constants.kpArm,Constants.kiArm , Constants.kdArm);

    public void armInitiallize() {
        armMotor.setNeutralMode(NeutralModeValue.Brake);
        compressor1.enableDigital();
    }

    /**
     * Move Arm given speed (NO CODE STOP!)
     * @param armSpeed double, From -1 to 1
     */
    public void moveArm(double speed) {
        armMotor.set(speed);
    } 

    /**
     * Move arm to angle with PID controls (NO CODE STOP!)
     * @param position target angle in radians from start position
     */
    public void moveArmPID(double position) {
        double armAngleTolerance = 0.2;
        
        double power = armPID.calculate(getArmAngleRad(), position); // + armFF.calculate(getArmRadians() - Constants.armOffset, position - Constants.armOffset);
        if (getArmAngleRad() < Constants.armLowPosition && Math.abs(position - getArmAngleRad()) < armAngleTolerance) {
            power = 0;
        }
        armMotor.set(power);
    }

    public double getArmAngleRad() {
        return (armMotor.getPosition().getValueAsDouble() / Constants.ARM_GEAR_RATIO) * Constants.DEGREES_PER_REV * 2 * Math.PI;  
        
    }

    public void armAngle(double targetAngle) {
        double currentAngle = getArmAngleRad(); 
        double speed = 0.2;  //arm speed
        
        targetAngle = Constants.clamp(currentAngle, 0, Math.PI/2);
    
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

    public boolean atPosition() {
        return armPID.atSetpoint();
    }

    public boolean isHardStoppedLow() {
        if (Constants.minArmRotationRads < getArmAngleRad()) {
            return false;
        } else return true;
    }

    public boolean isHardStoppedHigh() {
        if (getArmAngleRad() < Constants.maxArmRotationRads) {
            return false;
        } else return true;
    }


    public void openClaw() {
        solenoid1.set(Value.kForward);
    }

    public void closeClaw() {
        solenoid1.set(Value.kReverse);
    }
}
