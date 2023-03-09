package frc.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
 
  public final WPI_TalonFX leftFront= new WPI_TalonFX(1);
  public final WPI_TalonFX rightFront= new WPI_TalonFX(3);
  public final WPI_TalonFX leftBack= new WPI_TalonFX(2);
  public final WPI_TalonFX rightBack= new WPI_TalonFX(4); 
  public final WPI_TalonFX topRight= new WPI_TalonFX(6); 
  public final WPI_TalonFX topLeft= new WPI_TalonFX(5); 

  final MotorControllerGroup m_leftSide = new MotorControllerGroup(leftBack, leftFront);
  final MotorControllerGroup m_rightSide = new MotorControllerGroup(rightBack, rightFront);

  final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftSide, m_rightSide);
  private final DifferentialDrive topsDrive = new DifferentialDrive(topLeft, topRight);

  final XboxController xBoxCont = new XboxController(0);

  final double kp = 0.5;
  final double ki = 0.05;
  final double kd = 0.05;
  boolean xButtonPressed = false;

  double armSpeed = 0.3;
  double reArmSpeed = -0.3;
  double diameter = 6.5;
  //double radius = diameter/2; //of wheel in inchs
  //double circumferenceOfWheel = 2*Math.PI*radius;
  double countsPerRevTalonFX = 2048;
 // double distancePerPulse = circumferenceOfWheel / pulesPerRevTalonFX;
  
  PIDController drive = new PIDController(kp, ki, kd);   

  //Converts encoder ticks to feet
  public double tickToFeet(double numTicks, double gearRatio, double conversion, double diameter){
    //ticks x 1 rotation / 4096    x  1/1 gear ratio    x 6pi inches /   1 rotation x  1 feet/12 inch = ?feet
  double feet = numTicks * ((1 / 4028.0) * (gearRatio) * ((diameter * Math.PI)) * (1/12.0));
  //return ticks converted to feet
  return feet;

}

  public void driveFwd(double targetDistance, double tolerance) {
    resetEncoder();
    double output;
    double currentDistance = leftFront.getSelectedSensorPosition();

    if (Math.abs(targetDistance - currentDistance) > tolerance) {
      output = drive.calculate(currentDistance, targetDistance);
      
      SmartDashboard.putNumber("PID currentD", currentDistance);
      SmartDashboard.putNumber("PID SetPoint", targetDistance);
      SmartDashboard.putNumber("PID Output  ", output);
      
      output = Math.max(-1, Math.min(1, output));

      m_robotDrive.arcadeDrive(0, -output);
    } else{
      System.out.println("stop motor");
      m_robotDrive.stopMotor();
    }
  }

  public void setMotorsNeutral() {
    
    leftFront.setNeutralMode(NeutralMode.Brake);
    leftBack.setNeutralMode(NeutralMode.Brake);
    rightFront.setNeutralMode(NeutralMode.Brake);
    rightBack.setNeutralMode(NeutralMode.Brake);
  }
  
  public void resetEncoder(){

    topRight.setSelectedSensorPosition(0);
    topLeft.setSelectedSensorPosition(0);
    leftFront.setSelectedSensorPosition(0);

  }

@Override
  public void robotInit() {

  topRight.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);
  topLeft.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);
  leftFront.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);

  resetEncoder();
    
  // Set the distance per pulse for the integrated encoder
  
  //topRight.configSelectedFeedbackCoefficient(distancePerPulse / 2048.0);
  //topLeft.configSelectedFeedbackCoefficient(distancePerPulse / 2048.0);
  leftFront.configSelectedFeedbackCoefficient(tickToFeet(leftFront.getSelectedSensorPosition(), 10.71, countsPerRevTalonFX, diameter));
  
  // Set the direction of the integrated encoder
  topRight.setSensorPhase(false);
  topLeft.setSensorPhase(false);
  leftFront.setSensorPhase(false);
    
  resetEncoder();
  }

  @Override
  public void robotPeriodic() {
  }
  
  @Override
  public void autonomousInit() {
    setMotorsNeutral();
    driveFwd(12, 1);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    /*
     * align with starting node
     * rais arm
     * set down object
     * drive to next object
     * pick up
     * drive and align with new node
     * set object
     * balance
     */
    
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    setMotorsNeutral();
  }
/*
  public void limitArmRotation() {

    if (topRIght.getSelectedSensorPosition() >= 5){
      topsDrive.tankDrive(armSpeed, reArmSpeed);
    } 
    if (topRIght.getSelectedSensorPosition() <= -0.5){
      topsDrive.tankDrive(reArmSpeed, armSpeed);
    } 
  }
  */
  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    m_robotDrive.arcadeDrive(xBoxCont.getRawAxis(4) * 0.8, xBoxCont.getRawAxis(1) * 0.8);
  }
}