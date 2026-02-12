package frc.robot.systems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import frc.robot.Constants;

public class shooter {

    public static PWMSparkMax shooterMotor1 = new PWMSparkMax(2);
    public static PWMSparkMax shooterMotor2 = new PWMSparkMax(3);

    public static void shooterOn() {
        shooterMotor1.set(Constants.shooterMotor1Speed);
        shooterMotor2.set(Constants.shooterMotor2Speed);
    }

}