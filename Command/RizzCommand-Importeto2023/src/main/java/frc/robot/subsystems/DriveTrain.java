package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

public class DriveTrain extends PIDSubsystem {
  private static DifferentialDrive m_robotDrive;
  private MotorControllerGroup m_leftSide;
  private MotorControllerGroup m_rightSide;

  public DriveTrain(MotorController leftFront, MotorController rightFront, MotorController leftBack, MotorController rightBack) {
    super(
        // The PIDController used by the subsystem
        new PIDController(0, 0, 0));

        m_leftSide = new MotorControllerGroup(leftFront, leftBack);
        m_rightSide = new MotorControllerGroup(rightFront, rightBack);
        
        m_robotDrive = new DifferentialDrive(m_leftSide, m_rightSide);
  }

  public static DifferentialDrive getRobotDrive() {
    return m_robotDrive;
}

  @Override
  public void periodic() {
  }

  //This is for the PID's we can ignore below for now 
  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return 0;
  }
  
}