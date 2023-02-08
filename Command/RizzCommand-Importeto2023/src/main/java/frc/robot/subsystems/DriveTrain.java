package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

public class DriveTrain extends PIDSubsystem {
  private DifferentialDrive m_robotDrive;
  private MotorControllerGroup m_leftSide;
  private MotorControllerGroup m_rightSide;

  public DriveTrain(MotorController leftFront, MotorController rightFront, MotorController leftBack, MotorController rightBack) {
    super(
        // The PIDController used by the subsystem
        // dont worry about PID's for now
        new PIDController(0, 0, 0));
  
    m_leftSide = new MotorControllerGroup(leftFront, leftBack);
    m_rightSide = new MotorControllerGroup(rightFront, rightBack);
    
    m_robotDrive = new DifferentialDrive(m_leftSide, m_rightSide);
  }

  //here we are creating the drive system so we can call it in other location
  //note: trying to call this subsystem in the DriveTrain subsystems is tricky and wont work always with conventinal code
  //call it with DriveTrain.arcadeDriveSubClass
  public class arcadeDriveSubClass {
    public void arcadeDriveSub(double xSpeed, double zRotation) {
      m_robotDrive.arcadeDrive(xSpeed, zRotation);
    }
    //another Idea is making arcadeDriveSubClass its own subsystem
    //then you will have to pass the Drivetrain diffreiental "m_robotDrive" drive into the arcadeDrive subsystem to define .arcadeDrive
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