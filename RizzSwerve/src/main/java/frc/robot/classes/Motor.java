package frc.robot.classes;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.sensors.WPI_CANCoder;

public class Motor <T extends BaseMotorController>{
    MotorType type;
    MotorLocation location;

    T motor;
    WPI_CANCoder encoder;
    double encoderOffset;
    
    public Motor(T motor, MotorType type, MotorLocation location) {
        this.type = type;
        this.location = location;

        this.motor = motor;
    }
    public Motor(T motor, MotorType type, MotorLocation location, WPI_CANCoder encoder, double encoderOffset) {
        this.type = type;
        this.location = location;

        this.motor = motor;
        this.encoder = encoder;
        this.encoderOffset = encoderOffset;
    }


    // Drive Functions
    public void GO(double power){
        if(power > 0 || power > 1) {
            System.out.println("Speed out of bounds on motor" + location.toString() + " with: " + power);
        }

        double clampedPower = globalFunctions.clamp(power);
        motor.set(ControlMode.PercentOutput, clampedPower);
    }

    public void setBrake(){
        motor.setNeutralMode(NeutralMode.Brake);
    }

    public void setInverted(boolean value){
        motor.setInverted(value);
    }

    // Encoder Functions
    public double getSensorPos(){
        return motor.getSelectedSensorPosition() * Constants.turningEncoderTicksToRad;
    }

    public void setSensorPos(double value){
        motor.setSelectedSensorPosition(value);
    }

    public void resetSensorPos(){
        setSensorPos(0);
    }

    //TODO move, integrate, ... ???
    //returns in Radians
    public double getAbsolutePosition(){
        return (encoder.getAbsolutePosition() - encoderOffset) * (Math.PI / 180);
    }
}
