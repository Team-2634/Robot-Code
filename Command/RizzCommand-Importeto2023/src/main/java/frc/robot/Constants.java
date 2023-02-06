// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.XboxController;
//import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    final XboxController m_Xstick = new XboxController(0);
// Frunk ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    
    public final CANSparkMax leftFrontMax = new CANSparkMax(11, MotorType.kBrushless);
    public final CANSparkMax rightFrontMax = new CANSparkMax(6, MotorType.kBrushless);
    public final CANSparkMax leftBackMax = new CANSparkMax(5, MotorType.kBrushless);
    public final CANSparkMax rightBackMax = new CANSparkMax(8, MotorType.kBrushless);
    
// rizzler ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    /* 
    public final WPI_TalonFX leftFrontFX = new WPI_TalonFX(1);
    public final WPI_TalonFX rightFrontFX = new WPI_TalonFX(2);
    public final WPI_TalonFX leftBackFX = new WPI_TalonFX(3);
    public final WPI_TalonFX rightBackFX = new WPI_TalonFX(4);
    */
// LF ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
     // for Lord F we wait till robotics room: okie dokie
}