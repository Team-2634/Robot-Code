package frc.robot.systems;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import frc.robot.Constants;

public  class Intake {

   public static PWMSparkMax intakeMotor = new PWMSparkMax(1);

    public static void spin() {
        intakeMotor.set(Constants.intakeMotorSpeed);
    }

    //example: intake.spin()

}