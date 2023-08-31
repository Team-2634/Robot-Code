package frc.robot.classes;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.sensors.WPI_CANCoder;

//Base Motor object, this should provide basic functionality regardless of which type of motor is actually in use
public class Motor<T>{
    GenericMotor<T> motor;

    MotorModel type;
    MotorLocation location;

    WPI_CANCoder encoder;
    double encoderOffset;
    
    public Motor(T motor, MotorModel type, MotorLocation location) {
        this.motor = new GenericMotor<T>(motor);
        this.location = location;
        this.type = type;        
        //this.type = this.motor.getMotorType();
    }
    public Motor(T motor, MotorModel type, MotorLocation location, WPI_CANCoder encoder, double encoderOffset) {
        this.motor = new GenericMotor<T>(motor);
        this.location = location;
        this.type = type;

        this.encoder = encoder;
        this.encoderOffset = encoderOffset;
    }

    // Drive Functions
    public void GO(double power){
        if(power > 0 || power > 1) {
            System.out.println("Speed out of bounds on motor" + location.toString() + " with: " + power);
        }
        if(power == 0) {
            //TODO Set Brakes
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

    //TODO move, integrate with rest of object, ... ???
    //returns in Radians
    public double getAbsolutePosition(){
        return (encoder.getAbsolutePosition() - encoderOffset) * (Math.PI / 180);
    }
}
