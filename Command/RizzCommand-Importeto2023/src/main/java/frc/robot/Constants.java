package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.XboxController;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public final class Constants {

    final XboxController m_Xstick = new XboxController(0);
// Frunk ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    
    public final CANSparkMax leftFrontMax = new CANSparkMax(17, MotorType.kBrushless);
    public final CANSparkMax rightFrontMax = new CANSparkMax(10, MotorType.kBrushless);
    public final CANSparkMax leftBackMax = new CANSparkMax(4, MotorType.kBrushless);
    public final CANSparkMax rightBackMax = new CANSparkMax(18, MotorType.kBrushless);
    
// rizzler ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    
    public final WPI_TalonFX leftFrontFX = new WPI_TalonFX(1);
    public final WPI_TalonFX rightFrontFX = new WPI_TalonFX(3);
    public final WPI_TalonFX leftBackFX = new WPI_TalonFX(2);
    public final WPI_TalonFX rightBackFX = new WPI_TalonFX(4);
    
// LF ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
   /* 
    public final CANSparkMax leftFrontMax = new CANSparkMax(11, MotorType.kBrushless);
    public final CANSparkMax rightFrontMax = new CANSparkMax(6, MotorType.kBrushless);
    public final CANSparkMax leftBackMax = new CANSparkMax(5, MotorType.kBrushless);
    public final CANSparkMax rightBackMax = new CANSparkMax(8, MotorType.kBrushless);
    */
}