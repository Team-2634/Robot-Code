package frc.robot.systems;

import com.studica.frc.AHRS;

import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Constants;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Elevator {

    public final TalonFX armMotor = new TalonFX(Constants.armMotorID); //no ID value yet
    public final TalonFX elevatorMotor = new TalonFX(Constants.elevatorMotorID); //no ID value yet
    private Encoder armEncoder;

    public Elevator() {
        armEncoder = new Encoder(0, 1); // Replace with actual encoder ports
    }

    public double getArmAngle() {
        return armEncoder.getDistance(); // Returns the arm's angle
    }

    public void elevatorLift(double speedInput) {

        elevatorMotor.set(speedInput);

    }

    public void armLift(double speedInput){

        armMotor.set(speedInput);

    }

    public void ArmAngle(double angleInput){

        armMotor.set(angleInput);

    }

}