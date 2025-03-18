package frc.robot.systems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;

public class Climber {
       public final TalonFX climbMotor = new TalonFX(Constants.climbMotorID); 

       public void initializeClimb() {
              climbMotor.setPosition(0);
              climbMotor.setNeutralMode(NeutralModeValue.Brake);
       }

       public void moveClimb(double speed) {
              climbMotor.set(speed);
       }

}
