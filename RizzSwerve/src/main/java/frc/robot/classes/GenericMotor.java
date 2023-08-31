package frc.robot.classes;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

public class GenericMotor<T> {
    T motor;
    
    public GenericMotor(T motor) {
        this.motor = motor;
    }


    public void set(ControlMode controlMode, double clampedPower) {
        if(this.motor instanceof WPI_TalonFX){
            ((WPI_TalonFX) motor).set(controlMode, clampedPower);
        }
        else{ throw new Error("Command Unhandled for MotorType: " + getMotorType());}
    }
    public void set(double clampedPower) {
        if(this.motor instanceof WPI_TalonFX){
            ((WPI_TalonFX) motor).set(clampedPower);
        }
        else if(this.motor instanceof CANSparkMax){
            ((CANSparkMax) motor).set(clampedPower);
        } 
        else{ throw new Error("Command Unhandled for MotorType: " + getMotorType());}
    }

    public void setNeutralMode(NeutralMode neutralMode) {
        if(this.motor instanceof WPI_TalonFX){
            ((WPI_TalonFX) motor).setNeutralMode(neutralMode);
        } 
        else{ throw new Error("Command Unhandled for MotorType: " + getMotorType());}
    }
    public void setNeutralMode(IdleMode idleMode) {
        if(this.motor instanceof CANSparkMax){
            ((CANSparkMax) motor).setIdleMode(idleMode);
        } 
        else{ throw new Error("Command Unhandled for MotorType: " + getMotorType());}
    }

    public void setInverted(boolean value) {
        if(this.motor instanceof WPI_TalonFX){
            ((WPI_TalonFX) motor).setInverted(value);
        }
        else if(this.motor instanceof CANSparkMax){
            ((CANSparkMax) motor).setInverted(value);
        }
        else{ throw new Error("Command Unhandled for MotorType: " + getMotorType());}
    }

    public int getSelectedSensorPosition() {
        if(this.motor instanceof WPI_TalonFX){
            ((WPI_TalonFX) motor).getSelectedSensorPosition();
        }
        else{ throw new Error("Command Unhandled for MotorType: " + getMotorType());}
        return 0;
    }

    public void setSelectedSensorPosition(double value) {
        if(this.motor instanceof WPI_TalonFX){
            ((WPI_TalonFX) motor).setSelectedSensorPosition(value);
        }
        else{ throw new Error("Command Unhandled for MotorType: " + getMotorType());}
    }


    public Class getMotorType(){
        return (Class<T>)motor.getClass();
    }
}
