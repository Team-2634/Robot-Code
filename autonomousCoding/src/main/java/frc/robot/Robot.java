package frc.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.SPI;

public class Robot extends TimedRobot {

  AHRS navx = new AHRS(SPI.Port.kMXP);
  Timer timer = new Timer();
  final XboxController xBoxCont = new XboxController(0);
 
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

  private final DoubleSolenoid dSolenoidShifter = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 2);
  private final DoubleSolenoid dSolenoidClaw = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 3);
  private final DoubleSolenoid coolingSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 3);
  private final Compressor compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
  private final double Scale = 250, offset = -25;
  private final AnalogPotentiometer potentiometer = new AnalogPotentiometer(0, Scale, offset);
  //^^pnuematics solenoid values chek!!!

  final WPI_TalonFX armTalonExtenstion = new WPI_TalonFX(7);
  double armTalonExtenstion_GearRatio = 12;
  //^^extend arm setup

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
//functions start here~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

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

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
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

public void autoBalance_NORMAL() {
  double outputPitch=0;
  double outputYaw=0;
  double currentPitch;
  double currentYaw;
  double targetAnglePitch = 0;
  double targetAngleYaw = 0;
  double tolerance = 5;
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

public void autoBalance_AUTONOMOUS() {
  double outputPitch=0;
  double outputYaw=0;
  double currentPitch;
  double currentYaw;
  double targetAnglePitch = 0;
  double targetAngleYaw = 0;
  double tolerance = 5;
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

public void armLift_Lower(double targetDistanceDegrees, double tolerance){
  resetEncoder();
  double output;
  double currentDistance = rightFront.getSelectedSensorPosition();
  if (Math.abs(targetDistanceDegrees - currentDistance) > tolerance) {
    output = drive.calculate(currentDistance, targetDistanceDegrees);
    output = Math.max(-1, Math.min(1, output));
    m_robotDrive.tankDrive(output, output);
  } else{
    System.out.println("stop motor");
    m_robotDrive.stopMotor();
  }
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
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

public void limitArmRotation() {
  if (topRight.getSelectedSensorPosition() >= 90){ //change degrees
    topsDrive.tankDrive(armSpeed, reArmSpeed);
  } 
  if (topRight.getSelectedSensorPosition() <= -10){ //change degrees
    topsDrive.tankDrive(reArmSpeed, armSpeed);
  } 
}

private void pulsePiston(double Time) {
  int pulseFreq = 15;
  int pulseDuration = 1;
  if (Time % pulseFreq < pulseDuration) {
    coolingSolenoid.set(Value.kForward);
  } else {
    coolingSolenoid.set(Value.kReverse);
  }
}
//some more functions~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
@Override
public void robotInit() {
  timer.reset();
  timer.start();
  double currentPsi = potentiometer.get();
  int psiCap = 117;
  if (currentPsi <= psiCap) {
      compressor.enableDigital(); 
  } else if (currentPsi > 119) {
      compressor.disable();
  }

  navx.reset(); // recalibration may take 10 seconds!!!!!!!!!!
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
    //armLift_Lower(90, 5);
    //Timer.delay(5);
    drive_AUTONOMOUS(12, 1, false); //fwd 12 feet
    //Timer.delay(5);
    drive_AUTONOMOUS(-12, 1, false); //Bkwd 12 feet
    //Timer.delay(5);
    drive_AUTONOMOUS(180, 1, true); //turn 180
    //Timer.delay(5);
    drive_AUTONOMOUS(-180, 1, true); //turn back 180
  }

  @Override
  public void autonomousPeriodic() {
    double AutonomousTime = timer.get();
    pulsePiston(AutonomousTime);
    limitArmRotation();
    autoBalance_AUTONOMOUS();
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
  
  boolean aButtonPressed = false;
  boolean bButtonPressed = false;
  boolean xButtonPressed = false;
  double deadzone = 0.5;

  @Override
  public void teleopPeriodic() {
    double teleopTime = timer.get();
    pulsePiston(teleopTime);
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

   if (xBoxCont.getAButtonPressed() == true) {
     aButtonPressed = !aButtonPressed;
 }
 if (aButtonPressed == true) {
     dSolenoidShifter.set(Value.kForward);
 } else if (aButtonPressed == false) {
     dSolenoidShifter.set(Value.kReverse);
 }
 
 if (xBoxCont.getBButtonPressed() == true) {
   bButtonPressed = !bButtonPressed;
 } 
 if (bButtonPressed == true){
   dSolenoidClaw.set(Value.kForward);
 } else if (bButtonPressed == false){
   dSolenoidClaw.set(Value.kReverse);
 }
 
   if (xBoxCont.getRightBumper() == true){
     armTalonExtenstion.set(.30);
 } else if (xBoxCont.getLeftBumper() == true){
     armTalonExtenstion.set(-0.30);
 } else {
     armTalonExtenstion.set(0);
 }
 
 if (xBoxCont.getRawAxis(4) > deadzone || xBoxCont.getRawAxis(4) < -deadzone &&
     xBoxCont.getRawAxis(1) > deadzone || xBoxCont.getRawAxis(1) < -deadzone) 
     {
     xButtonPressed = false;
 }
 if (xButtonPressed == true) {
     autoBalance_NORMAL();
 }
 }
}

/*
 * bumbers = arm extenstion
 * triggers = TankDrive aka armLift
 * A button = gearShift
 * B button = claw open close
 * X button = toggle balance code teleop
 * JoyStick = fwd, back, left and right
 */