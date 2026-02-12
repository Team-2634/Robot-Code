package frc.robot;

import frc.robot.systems.*;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

public  class Intake {

   public PWMSparkMax intakeMotor = new PWMSparkMax(1);

    public void spin() {
        intakeMotor.set(1);
    }

    //example: intake.spin()

}