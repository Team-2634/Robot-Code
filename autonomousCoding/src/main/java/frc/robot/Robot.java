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
  double arcadeDrive_GearRatio = 0; //figure out note: will change for swerver drive

  final MotorControllerGroup m_leftSide = new MotorControllerGroup(leftBack, leftFront);
  final MotorControllerGroup m_rightSide = new MotorControllerGroup(rightBack, rightFront);

  final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftSide, m_rightSide);

  public final WPI_TalonFX topRight= new WPI_TalonFX(6); 
  public final WPI_TalonFX topLeft= new WPI_TalonFX(5); 
  double armLift_GearRatio = Math.pow(6,2);

  private final DifferentialDrive topsDrive = new DifferentialDrive(topLeft, topRight);

  // push arm ratio: 12:1

  final XboxController xBoxCont = new XboxController(0);

  final double kp = 0.5;
  final double ki = 0.05;
  final double kd = 0.05;
  boolean xButtonPressed = false;

  double armSpeed = 0.4;
  double reArmSpeed = -0.4;
  double diameterRizzArcadeWheels = 6.5;
  double radiusRizzArcadeWheels = diameterRizzArcadeWheels/2; //of wheel in inchs
  double countsPerRevTalonFX = 2048;
  
  PIDController drive = new PIDController(kp, ki, kd);   

public double encoderToFeet(double radius, double countsPerRev, double encoderValue, double gearRatio){
  double circumferenceOfWheel = 2*Math.PI*radius;
  double countsPerInch = countsPerRev / circumferenceOfWheel / gearRatio;
  double inches = encoderValue / countsPerInch;
  double feet = inches / 12;

  // in other words: double Feet = (encoderValue/(countsPerRev/(2*Math.PI*radius)))/12;

  return feet;
}

public double encoderToDegrees(double countsPerRev, double encoderValue, double gearRatio) {
  double countsPerRotation = countsPerRev * gearRatio;
  double degreesPerCount = 360.0 / countsPerRotation;
  double degrees = encoderValue * degreesPerCount;
  return degrees;
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
public void driveFwd_or_Bwd_AUTONOMOUS(double targetDistanceFeet, double tolerance) {
    resetEncoder();
    double output;
    double currentDistance = leftFront.getSelectedSensorPosition();

    if (Math.abs(targetDistanceFeet - currentDistance) > tolerance) {
      output = drive.calculate(currentDistance, targetDistanceFeet);
      output = Math.max(-1, Math.min(1, output));
      m_robotDrive.arcadeDrive(0, -output);
    } else{
      System.out.println("stop motor");
      m_robotDrive.stopMotor();
    }
  }

public void driveFwd_or_Bwd_NORMAL(double targetDistanceFeet, double tolerance) {
   resetEncoder();
   double output;
   double currentDistance = leftFront.getSelectedSensorPosition();
   if (Math.abs(targetDistanceFeet - currentDistance) > tolerance) {
     output = drive.calculate(currentDistance, targetDistanceFeet);
     output = Math.max(-1, Math.min(1, output));
     m_robotDrive.arcadeDrive(output, 0);
   } else{
     System.out.println("stop motor");
     m_robotDrive.stopMotor();
   }
 }

public void driveTurn_leftOrRight_AUTONOMOUS(double targetDistanceDegrees, double tolerance) {
  resetEncoder();
  double output;
  double currentDistance = rightFront.getSelectedSensorPosition();
  if (Math.abs(targetDistanceDegrees - currentDistance) > tolerance) {
    output = drive.calculate(currentDistance, targetDistanceDegrees);
    output = Math.max(-1, Math.min(1, output));
    m_robotDrive.arcadeDrive(-output, 0);
  } else{
    System.out.println("stop motor");
    m_robotDrive.stopMotor();
  }
 }

public void driveTurn_leftOrRight_NORMAL(double targetDistanceDegrees, double tolerance) {
 resetEncoder();
 double output;
 double currentDistance = rightFront.getSelectedSensorPosition();
 if (Math.abs(targetDistanceDegrees - currentDistance) > tolerance) {
   output = drive.calculate(currentDistance, targetDistanceDegrees);
   output = Math.max(-1, Math.min(1, output));
   m_robotDrive.arcadeDrive(0, output);
 } else{
   System.out.println("stop motor");
   m_robotDrive.stopMotor();
 }
}

public void limitArmRotation() {
  if (topRight.getSelectedSensorPosition() >= 90){ //change degrees
    topsDrive.tankDrive(armSpeed, reArmSpeed);
  } 
  if (topRight.getSelectedSensorPosition() <= -10){ //change degrees
    topsDrive.tankDrive(reArmSpeed, armSpeed);
  } 
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

public void setMotorsNeutral() {
    
    leftFront.setNeutralMode(NeutralMode.Brake);
    leftBack.setNeutralMode(NeutralMode.Brake);
    rightFront.setNeutralMode(NeutralMode.Brake);
    rightBack.setNeutralMode(NeutralMode.Brake);
  }
  
public void resetEncoder(){
    leftFront.setSelectedSensorPosition(0);
    rightFront.setSelectedSensorPosition(0);
    topRight.setSelectedSensorPosition(0);
  }

@Override
public void robotInit() {
  setMotorsNeutral();
  //tell motor to use integrated sensor
  leftFront.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);
  rightFront.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);
  topRight.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);

  //config units you want encoder to be in
  leftFront.configSelectedFeedbackCoefficient(encoderToFeet(radiusRizzArcadeWheels, countsPerRevTalonFX, leftFront.getSelectedSensorPosition(), arcadeDrive_GearRatio));
  rightFront.configSelectedFeedbackCoefficient(encoderToDegrees(countsPerRevTalonFX, rightFront.getSelectedSensorPosition(), arcadeDrive_GearRatio));
  topRight.configSelectedFeedbackCoefficient(encoderToDegrees(countsPerRevTalonFX, topRight.getSelectedSensorPosition(), armLift_GearRatio));

  // Set the direction of the integrated encoder
  leftFront.setSensorPhase(false);
  rightFront.setSensorPhase(false);
  topRight.setSensorPhase(false);
  resetEncoder();
  }

  @Override
public void robotPeriodic() {
  //limitArmRotation();

  // print oout encoder status
  SmartDashboard.putNumber("topRight.Encoder: ", topRight.getSelectedSensorPosition());
  SmartDashboard.putNumber("leftFront.Encoder: ", leftFront.getSelectedSensorPosition());
  SmartDashboard.putNumber("rightFront.Encoder: ", rightFront.getSelectedSensorPosition());

  }
  
  @Override
  public void autonomousInit() {
    driveFwd_or_Bwd_AUTONOMOUS(12, 1);
    driveFwd_or_Bwd_AUTONOMOUS(-12, 1);
    driveTurn_leftOrRight_AUTONOMOUS(180, 1);
    driveTurn_leftOrRight_AUTONOMOUS(-180, 1);
  }

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

  @Override
  public void teleopInit() {
  }
  
  @Override
  public void teleopPeriodic() {
    m_robotDrive.arcadeDrive(xBoxCont.getRawAxis(4) * 0.8, xBoxCont.getRawAxis(1) * 0.8);
    topsDrive.tankDrive(xBoxCont.getLeftTriggerAxis(), -xBoxCont.getLeftTriggerAxis());
    topsDrive.tankDrive(-xBoxCont.getLeftTriggerAxis(), xBoxCont.getLeftTriggerAxis());
  }
}