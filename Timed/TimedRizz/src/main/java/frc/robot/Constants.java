package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

public class Constants {
    // Frunk ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        
        public final CANSparkMax leftFront = new CANSparkMax(11, MotorType.kBrushless);
        public final CANSparkMax rightFront = new CANSparkMax(6, MotorType.kBrushless);
        public final CANSparkMax leftBack = new CANSparkMax(5, MotorType.kBrushless);
        public final CANSparkMax rightBack = new CANSparkMax(8, MotorType.kBrushless);
        
    // rizzler ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        /*
        public final WPI_TalonFX leftFront = new WPI_TalonFX(1);
        public final WPI_TalonFX rightFront = new WPI_TalonFX(2);
        public final WPI_TalonFX leftBack = new WPI_TalonFX(3);
        public final WPI_TalonFX rightBack= new WPI_TalonFX(4); 
 */
    // Lf and Rs sides motors~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    public final MotorControllerGroup m_leftSide = new MotorControllerGroup(leftBack, leftFront);
    public final MotorControllerGroup m_rightSide = new MotorControllerGroup(rightBack, rightFront);

    // intake Morotrs:~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    // private final CANSparkMax leftIntake = new CANSparkMax(2, MotorType.kBrushless);
    // private final CANSparkMax rightIntake = new CANSparkMax(7, MotorType.kBrushless);

    // private final MotorControllerGroup intakeMotors = new MotorControllerGroup(leftIntake, rightIntake);
}