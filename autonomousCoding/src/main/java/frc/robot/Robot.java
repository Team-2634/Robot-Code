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
  double radius = 6.5/2; //of wheel in inchs
  double circumferenceOfWheel = 2*Math.PI*radius;
  double pulesPerRevTalonFX = 2048;
  double distancePerPulse = circumferenceOfWheel / pulesPerRevTalonFX;
  double targetDistance;

  PIDController driveFwdPid = new PIDController(kp, ki, kd);   

  public void driveForwarddd() {
    m_robotDrive.arcadeDrive(0, -0.30);
    SmartDashboard.putNumber("leftFront motorPercent", leftFront.get());
  }

  public void driveForward() {
    //resetEncoder();
    double output;

     
    targetDistance = 200; //inchs
    double currentDistance = leftFront.getSelectedSensorPosition();
    double tolerance = 1;

    //driveFwdPid.reset();
    //driveFwdPid.setSetpoint(targetDistance);



    if (Math.abs(targetDistance - currentDistance) > tolerance) {
      output = driveFwdPid.calculate(currentDistance, targetDistance);
      
      SmartDashboard.putNumber("PID currentD", currentDistance);
      SmartDashboard.putNumber("PID SetPoint", targetDistance);
      //SmartDashboard.putNumber("leftFront motorPercent", leftFront.get());
      SmartDashboard.putNumber("PID Output  ", output);
      //SmartDashboard.putNumber("PID Output Divided", output);
      
      output = Math.max(-1, Math.min(1, output));

      m_robotDrive.arcadeDrive(0, -output);
      //leftFront.set(output/10);
    } else{
      System.out.println("stop motor");
      m_robotDrive.stopMotor();
    }

    driveFwdPid.close();
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
  
  topRight.configSelectedFeedbackCoefficient(distancePerPulse / 2048.0);
  topLeft.configSelectedFeedbackCoefficient(distancePerPulse / 2048.0);
  leftFront.configSelectedFeedbackCoefficient(distancePerPulse);// this one is fixed
  
  // Set the direction of the integrated encoder
  topRight.setSensorPhase(false);
  topLeft.setSensorPhase(false);
  leftFront.setSensorPhase(false);
    
  resetEncoder();
  }

  @Override
  public void robotPeriodic() {
    double leftFrontEncoder = leftFront.getSelectedSensorPosition();
    double topRightEncoder = topRight.getSelectedSensorPosition();
    double topLeftEncoder = topLeft.getSelectedSensorPosition();
    //SmartDashboard.putNumber("topLeft sensorPos",  topLeftEncoder);
    //SmartDashboard.putNumber("Output PID", output);
  }
  
  @Override
  public void autonomousInit() {
    setMotorsNeutral();
    driveForward();
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

    //limitArmRotation();

    m_robotDrive.arcadeDrive(xBoxCont.getRawAxis(4) * 0.8, xBoxCont.getRawAxis(1) * 0.8);
    
    if (xBoxCont.getXButtonPressed() == true) {
      xButtonPressed = !xButtonPressed;
  }
  double deadzone = 0.5;
  if (xBoxCont.getRawAxis(4) > deadzone ||
          xBoxCont.getRawAxis(4) < -deadzone &&
                  xBoxCont.getRawAxis(1) > deadzone || 
                      xBoxCont.getRawAxis(1) < -deadzone) {
      xButtonPressed = false;
                      }
  if (xButtonPressed == true) {
    driveForward();
  }
  SmartDashboard.putBoolean("driveForward(); mode: ", xButtonPressed);
    
/*
    if (xBoxCont.getLeftBumper() == true) {
      topsDrive.tankDrive(armSpeed, reArmSpeed);
      topLeft.setNeutralMode(NeutralMode.Brake);
      topRIght.setNeutralMode(NeutralMode.Brake);
  }else if (xBoxCont.getRightBumper() == true) {
      topsDrive.tankDrive(reArmSpeed,armSpeed);
      topLeft.setNeutralMode(NeutralMode.Brake);
      topRIght.setNeutralMode(NeutralMode.Brake);
  }else {
      topsDrive.tankDrive(0,0);
      topLeft.setNeutralMode(NeutralMode.Brake);
      topRIght.setNeutralMode(NeutralMode.Brake);
  }  */
  }
}