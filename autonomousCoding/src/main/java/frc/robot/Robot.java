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
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

public class Robot extends TimedRobot {

  private AHRS navx = new AHRS(SPI.Port.kMXP);
 
  final WPI_TalonFX leftFront= new WPI_TalonFX(1);
  final WPI_TalonFX rightFront= new WPI_TalonFX(3);
  final WPI_TalonFX leftBack= new WPI_TalonFX(2);
  final WPI_TalonFX rightBack= new WPI_TalonFX(4); 
  final MotorControllerGroup m_leftSide = new MotorControllerGroup(leftBack, leftFront);
  final MotorControllerGroup m_rightSide = new MotorControllerGroup(rightBack, rightFront);
  final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftSide, m_rightSide);
  double arcadeDrive_GearRatio = 0; //figure out note: will change for swerver drive
  //^^arcade setup

  public final WPI_TalonFX topRight= new WPI_TalonFX(6); 
  public final WPI_TalonFX topLeft= new WPI_TalonFX(5);
  double armSpeed = 0.4;
  double reArmSpeed = -0.4;
  private final DifferentialDrive topsDrive = new DifferentialDrive(topLeft, topRight); 
  double armLift_GearRatio = Math.pow(6,2);
  //^^TankArm setup

  final WPI_TalonFX armTalonExtenstion = new WPI_TalonFX(7);
  double armTalonExtenstion_GearRatio = 12;
  //^^extend arm setup

  final XboxController xBoxCont = new XboxController(0);

  double diameterRizzArcadeWheels = 6.5;
  double radiusRizzArcadeWheels = diameterRizzArcadeWheels/2; //of wheel in inchs
  double countsPerRevTalonFX = 2048;
  
  final double kp = 0.5;
  final double ki = 0.05;
  final double kd = 0.05;
  final double kpPitch = 0.5; 
  final double kpYaw = 0.5;
  // may need to be set by user in function call

  PIDController drive = new PIDController(kp, ki, kd); // may need to spearate for turnings
  PIDController pidPitch = new PIDController(kpPitch, ki, kd);
  PIDController pidYaw = new PIDController(kpYaw, ki, kd);

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
public void drive_AUTONOMOUS(double targetDistanceFeet, double tolerance, boolean currentMotorTurn) {
    resetEncoder();
    double output;
    double currentDistance;

    if (currentMotorTurn == false){
      currentDistance = leftFront.getSelectedSensorPosition();
    } else if (currentMotorTurn == true) {
      currentDistance = rightFront.getSelectedSensorPosition();
    } else {
      currentDistance = leftFront.getSelectedSensorPosition();
    }
    
    if (Math.abs(targetDistanceFeet - currentDistance) > tolerance) {
      output = drive.calculate(currentDistance, targetDistanceFeet);
      output = Math.max(-1, Math.min(1, output));
      m_robotDrive.arcadeDrive(0, -output);
    } else{
      System.out.println("stop motor");
      m_robotDrive.stopMotor();
    }
  }

public void drive_NORMAL(double targetDistanceFeet, double tolerance, boolean currentMotorTurn) {
   resetEncoder();
   double output;
   double currentDistance;

   if (currentMotorTurn == false){
    currentDistance = leftFront.getSelectedSensorPosition();
  } else if (currentMotorTurn == true) {
    currentDistance = rightFront.getSelectedSensorPosition();
  } else {
    currentDistance = leftFront.getSelectedSensorPosition();
  }

   if (Math.abs(targetDistanceFeet - currentDistance) > tolerance) {
     output = drive.calculate(currentDistance, targetDistanceFeet);
     output = Math.max(-1, Math.min(1, output));
     m_robotDrive.arcadeDrive(output, 0);
   } else{
     System.out.println("stop motor");
     m_robotDrive.stopMotor();
   }
 }

/*
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
 */

public void limitArmRotation() {
  if (topRight.getSelectedSensorPosition() >= 90){ //change degrees
    topsDrive.tankDrive(armSpeed, reArmSpeed);
  } 
  if (topRight.getSelectedSensorPosition() <= -10){ //change degrees
    topsDrive.tankDrive(reArmSpeed, armSpeed);
  } 
}

public void autoBalance_NORMAL() {
  double outputPitch=0;
  double outputYaw=0;
  double currentPitch;
  double currentYaw;
  double targetAnglePitch = 0;
  double targetAngleYaw = 0;
  double tolerance = 5;
  
  navx.reset(); // recalibration may take 10 seconds!!!!!!!!!!
  currentPitch = navx.getPitch();
  currentYaw = navx.getYaw();

  if (Math.abs(targetAnglePitch - currentPitch) > tolerance) {
    outputPitch = pidPitch.calculate(currentPitch, targetAnglePitch);
    outputPitch = Math.max(-1, Math.min(1, outputPitch));
  }
  if (Math.abs(targetAngleYaw - currentYaw) > tolerance) {
    outputYaw = pidYaw.calculate(currentYaw, targetAngleYaw);
    outputYaw = Math.max(-1, Math.min(1, outputYaw));
  }
  m_robotDrive.arcadeDrive(outputPitch, outputYaw);
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

public void setMotorsNeutral() {
    
  leftFront.setNeutralMode(NeutralMode.Brake);
  leftBack.setNeutralMode(NeutralMode.Brake);
  rightFront.setNeutralMode(NeutralMode.Brake);
  rightBack.setNeutralMode(NeutralMode.Brake);
  topLeft.setNeutralMode(NeutralMode.Brake);
  topRight.setNeutralMode(NeutralMode.Brake);
  armTalonExtenstion.setNeutralMode(NeutralMode.Brake);
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

  SmartDashboard.putNumber("topRight.Encoder: ", topRight.getSelectedSensorPosition());
  SmartDashboard.putNumber("leftFront.Encoder: ", leftFront.getSelectedSensorPosition());
  SmartDashboard.putNumber("rightFront.Encoder: ", rightFront.getSelectedSensorPosition());
  //^^encoder
  SmartDashboard.putNumber("navx getPitch: ", navx.getPitch());
  SmartDashboard.putNumber("navx getYaw: ", navx.getYaw());
  SmartDashboard.putNumber("navx getRoll: ", navx.getRoll());
  SmartDashboard.putNumber("navx getAngle: ", navx.getAngle());
  //^^navx Angles

  }
  
  @Override
  public void autonomousInit() {
    drive_AUTONOMOUS(12, 1, false); //fwd 12 feet
    drive_AUTONOMOUS(-12, 1, false); //Bakwd 12 feet
    drive_AUTONOMOUS(180, 1, true); //turn 180
    drive_AUTONOMOUS(-180, 1, true); //turn back 180
  }

  @Override
  public void autonomousPeriodic() {
    limitArmRotation();
    autoBalance_NORMAL();
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
    /*
    if (xBoxCont.getLeftTriggerAxis() >= 0.5) {
      topsDrive.tankDrive(armSpeed,-armSpeed);
  }else if (xBoxCont.getRightTriggerAxis() >= 0.5) {
      topsDrive.tankDrive(-armSpeed,armSpeed);
  }else {
      topsDrive.tankDrive(0,0);
  } 
  */

    if (xBoxCont.getRightBumper() == true){
      armTalonExtenstion.set(.30);
  } else if (xBoxCont.getLeftBumper() == true){
      armTalonExtenstion.set(-0.30);
  } else {
      armTalonExtenstion.set(0);
  }
  }
}