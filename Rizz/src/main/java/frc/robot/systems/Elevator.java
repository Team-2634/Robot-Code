package frc.robot.systems;

import com.studica.frc.AHRS;

import frc.robot.Constants;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Elevator {

    public final TalonFX armMotor = new TalonFX(Constants.armMotorID); //no ID value yet


    public void elevatorLiftUp(double speedInput) {

        armMotor.set(speedInput);


    }
    
}