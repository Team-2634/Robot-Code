package frc.robot.systems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Constants;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Elevator{

    public final TalonFX elevatorMotor = new TalonFX(Constants.elevatorMotorID);
    private Encoder armEncoder;
    private Encoder elevatorEncoder;
    PIDController elevatorPID = new PIDController(Constants.kpElevator, Constants.kiElevator ,Constants.kdElevator);
    

    public void elevatorInitiallize(){
        elevatorMotor.setNeutralMode(NeutralModeValue.Brake);
        elevatorMotor.setPosition(0);
    }

    public void elevatorEncoderReset(){
        elevatorEncoder.reset();
        armEncoder.reset();
    }

    public void elevatorLift(double speed) {

        if (getElevatorHeight() > Constants.elevatorHighHardstop) {
            speed = Constants.clamp(speed, 0.0, 1.0);
        } else if (getElevatorHeight() < Constants.elevatorLowHardstop) {
            speed = Constants.clamp(speed, -1.0, 0.0);
        }

        elevatorMotor.set(speed);
    }

    public boolean atTargetElevatorPositionL4(){ //For Auto

        double targetHeight = Constants.L4_HEIGHT;
        double currentHeight = getElevatorHeight(); // Get the current elevator height

        double tolerance = 0.05;
        return Math.abs(currentHeight - targetHeight) < tolerance;

    }

    public boolean atTargetElevatorPositionInTake(){ //For Auto

        double targetHeight = Constants.L0_HEIGHT;
        double currentHeight = getElevatorHeight(); // Get the current elevator height

        double tolerance = 0.05;
        return Math.abs(currentHeight - targetHeight) < tolerance;

    }

    public boolean atTargetElevatorPositionL1(){ //For Auto

        double targetHeight = Constants.L1_HEIGHT;
        double currentHeight = getElevatorHeight(); // Get the current elevator height

        double tolerance = 0.05;
        return Math.abs(currentHeight - targetHeight) < tolerance;

    }

    public void elevatorPIDLift(double height) {

        double tolerance = 0.2;

        double speedInput = -elevatorPID.calculate(getElevatorHeight(), height);
        if (Math.abs(height - getElevatorHeight()) < tolerance) {
            speedInput = 0;
        }
        
        elevatorLift(speedInput);
    }

    public double getElevatorHeight() {
        return (elevatorMotor.getPosition().getValueAsDouble() / Constants.talonEncoder_TicksPerRev) * Constants.drumDiameter * -Math.PI * 2256.809339;
    }

    public void moveToL1() { moveToHeight(Constants.L1_HEIGHT); }
    public void moveToL2() { moveToHeight(Constants.L2_HEIGHT); }
    public void moveToL3() { moveToHeight(Constants.L3_HEIGHT); }
    public void moveToL4() { moveToHeight(Constants.L4_HEIGHT); }

    public void moveToHeight(double targetHeight) {
        double currentHeight = getElevatorHeight(); 
        //Backup function incase PID's don't work
        double speed = 0.7; // Adjust speed as needed
        
        if (currentHeight < targetHeight - 0.02) {
            elevatorMotor.set(speed); // Move up
        } else if (currentHeight > targetHeight + 0.02) {
            elevatorMotor.set(-speed); // Move down
        } else {
            elevatorMotor.set(0); // Stop at target
        }
    }
}