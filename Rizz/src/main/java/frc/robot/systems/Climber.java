package frc.robot.systems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;

public class Climber {
       public final TalonFX climbMotor1 = new TalonFX(Constants.climbMotorID1); 
       public final TalonFX climbMotor2 = new TalonFX(Constants.climbMotorID2); 


       public void initializeClimb() {
              climbMotor1.setPosition(0);
              climbMotor2.setPosition(0);
              climbMotor1.setNeutralMode(NeutralModeValue.Brake);
              climbMotor2.setNeutralMode(NeutralModeValue.Brake);
       }

       public void moveClimb(double speed) {
              climbMotor1.set(speed);
              climbMotor2.set(speed);

       }

}
