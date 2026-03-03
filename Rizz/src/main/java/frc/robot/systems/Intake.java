package frc.robot.systems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import frc.robot.Constants;

public  class Intake {

   public static PWMSparkMax intakeMotor = new PWMSparkMax(1);
   public static TalonFX upDownIntakeMotor = new TalonFX(16);

    public static void spin() {
        intakeMotor.set(Constants.intakeMotorSpeed);
    }

    public static void spinReverse() {
        intakeMotor.set(-Constants.intakeMotorSpeed);
    }

    //example: intake.spin()

    public static void intakeUp(){
        upDownIntakeMotor.set(0.5);
    }

    public static void intakeDown(){
        upDownIntakeMotor.set(-0.5);
    }

}