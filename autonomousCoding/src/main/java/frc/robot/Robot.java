package frc.robot;
// the constants in this code need to be changed and edited after installation of swerver drive
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
  double maxDegree = 360; // if your over 360 then your driving to much

  final WPI_TalonFX leftFront= new WPI_TalonFX(1);
  final WPI_TalonFX rightFront= new WPI_TalonFX(3);
  final WPI_TalonFX leftBack= new WPI_TalonFX(2);
  final WPI_TalonFX rightBack= new WPI_TalonFX(4); 
  final MotorControllerGroup m_leftSide = new MotorControllerGroup(leftBack, leftFront);
  final MotorControllerGroup m_rightSide = new MotorControllerGroup(rightBack, rightFront);
  final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftSide, m_rightSide);
  double arcadeDrive_GearRatio = 36; //figure out, note: this will change for swerver drive
  double maxSpeed_FeetPerSecond = 12.7588;
  //^^arcade setup

  public final WPI_TalonFX topRight= new WPI_TalonFX(6); 
  public final WPI_TalonFX topLeft= new WPI_TalonFX(5);
  double armSpeed = 0.55;
  double reArmSpeed = -0.55;
  double minArmAngle = 1;
  double maxArmAngle = -17;
  private final DifferentialDrive topsDrive = new DifferentialDrive(topLeft, topRight); 
  double armLift_GearRatio = 144; //36 olds news
  //^^TankArm setup

  final WPI_TalonFX armTalonExtenstion = new WPI_TalonFX(7);
  double armTalonExtenstion_GearRatio = 12;
  //^^extend arm setup

  private final DoubleSolenoid dSolenoidClaw = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 3, 0);
  //private final DoubleSolenoid coolingSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 3);
  private final Compressor compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
  private final double Scale = 250, offset = -25;
  private final AnalogPotentiometer potentiometer = new AnalogPotentiometer(0, Scale, offset);
  //^^pnuematics solenoid values chek!!!

  double diameterRizzArcadeWheels = 6.5;
  double radiusRizzArcadeWheels = diameterRizzArcadeWheels/2; //of wheel in inchs
  double radiusArmGear = 0; //GET RADIUS
  double countsPerRevTalonFX = 2048;
  
  final double kp = 0.5;
  final double ki = 0.05;
  final double kd = 0.05;
  final double kpPitch = 0.1; 
  final double kpYaw = 0.1;
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
  return feet;
}

public double encoderToDegrees(double countsPerRev, double encoderValue, double gearRatio) {
  double countsPerRotation = countsPerRev * gearRatio;
  double degreesPerCount = 360.0 / countsPerRotation;
  double degrees = encoderValue * degreesPerCount;
  return degrees;
}
 
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
public void drive_AUTONOMOUS(double targetDistance, double tolerance, boolean currentMotorTurn) {
  resetEncoder_DriveFeet();
  resetEncoder_TurnDegrees();
  double output;
  double currentDistance;

  currentDistance = currentMotorTurn ? rightFront.getSelectedSensorPosition() : leftFront.getSelectedSensorPosition();
  
  if (Math.abs(targetDistance - currentDistance) > tolerance) {
    
    if (currentMotorTurn == false){
    output = drive.calculate(currentDistance, targetDistance) / (maxSpeed_FeetPerSecond * 12.0); //calculate output in feet then convert to precentage
    m_robotDrive.arcadeDrive(0, -output);

  } else if (currentMotorTurn == true){ 
    output = drive.calculate(currentDistance, targetDistance) / (maxDegree * 100.0); //calculate output in degrees then convert to precentage
    m_robotDrive.arcadeDrive(-output, 0);
  }  
  } else{
    System.out.println("stop motor");
    m_robotDrive.stopMotor();
  }
}
// change pid values ^^^
public void drive_NORMAL(double targetDistance, double tolerance, boolean currentMotorTurn) {
  resetEncoder_DriveFeet();
  resetEncoder_TurnDegrees();
  double output;
  double currentDistance;

  currentDistance = currentMotorTurn ? rightFront.getSelectedSensorPosition() : leftFront.getSelectedSensorPosition();
  
  if (Math.abs(targetDistance - currentDistance) > tolerance) {
    
    if (currentMotorTurn == false){
    output = drive.calculate(currentDistance, targetDistance) / (maxSpeed_FeetPerSecond * 12.0); //calculate output in feet then convert to precentage
    m_robotDrive.arcadeDrive(output, 0);

  } else if (currentMotorTurn == true){ 
    output = drive.calculate(currentDistance, targetDistance) / (maxDegree * 100.0); //calculate output in degrees then convert to precentage
    m_robotDrive.arcadeDrive(0, output);
  }  
  } else{
    System.out.println("stop motor");
    m_robotDrive.stopMotor();
  }
}
// change pid values ^^^
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
    outputPitch = pidPitch.calculate(currentPitch, targetAnglePitch) / (maxDegree * 100.0); //calculate output in degrees then convert to precentage
  }
  if (Math.abs(targetAngleYaw - currentYaw) > tolerance) {
    outputYaw = pidYaw.calculate(currentYaw, targetAngleYaw) / (maxDegree * 100.0); //calculate output in degrees then convert to precentage
  }
  m_robotDrive.arcadeDrive(outputPitch, outputYaw);
}
// change pid values ^^^
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
    outputPitch = pidPitch.calculate(currentPitch, targetAnglePitch) / (maxDegree * 100.0); //calculate output in degrees then convert to precentage
  }
  if (Math.abs(targetAngleYaw - currentYaw) > tolerance) {
    outputYaw = pidYaw.calculate(currentYaw, targetAngleYaw) / (maxDegree * 100.0); //calculate output in degrees then convert to precentage
  }
  m_robotDrive.arcadeDrive(-outputPitch, -outputYaw);
}
// change pid values ^^^
public void armLift_Lower(double targetDistanceDegrees, double tolerance){
  resetEncoder_armLift();
  double output;
  double currentDistance = topRight.getSelectedSensorPosition();
  if (Math.abs(targetDistanceDegrees - currentDistance) > tolerance) {
    output = drive.calculate(currentDistance, targetDistanceDegrees) / (maxDegree * 100.0); //calculate output in degrees then convert to precentage
    m_robotDrive.tankDrive(output, output);
  } else{
    System.out.println("stop motor");
    m_robotDrive.stopMotor();
  }
}
// needs its own pid ^^^^^
public void armExtender(double targetDistanceFeet, double tolerance){
  resetEncoder_Extend();
  double output;
  double currentDistance = armTalonExtenstion.getSelectedSensorPosition();
  if (Math.abs(targetDistanceFeet - currentDistance) > tolerance) {
    output = drive.calculate(currentDistance, targetDistanceFeet) / (maxSpeed_FeetPerSecond * 12.0); //calculate output in feet then convert to precentage
    armTalonExtenstion.set(output);
  } else{
    System.out.println("stop motor");
    m_robotDrive.stopMotor();
  }
}
// needs its own pid ^^^^
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
  
public void resetEncoder_DriveFeet(){
  //resets eveything including encoder value/Coefficient
  leftFront.setSelectedSensorPosition(0);
  //config units you want encoder to be in
  leftFront.configSelectedFeedbackCoefficient(encoderToFeet(radiusRizzArcadeWheels, countsPerRevTalonFX, leftFront.getSelectedSensorPosition(), arcadeDrive_GearRatio));
}

public void resetEncoder_TurnDegrees(){
  rightFront.setSelectedSensorPosition(0);  
  rightFront.configSelectedFeedbackCoefficient(encoderToDegrees(countsPerRevTalonFX, rightFront.getSelectedSensorPosition(), arcadeDrive_GearRatio));
}

public void resetEncoder_armLift(){
  topRight.setSelectedSensorPosition(0);  
  topRight.configSelectedFeedbackCoefficient(encoderToDegrees(countsPerRevTalonFX, topRight.getSelectedSensorPosition(), armLift_GearRatio));
}

public void resetEncoder_Extend(){
  armTalonExtenstion.setSelectedSensorPosition(0);
  armTalonExtenstion.configSelectedFeedbackCoefficient(encoderToFeet(radiusArmGear, countsPerRevTalonFX, armTalonExtenstion.getSelectedSensorPosition(), armTalonExtenstion_GearRatio));
}

public void limitArmRotation(double getArmDegValue) {
  if (getArmDegValue <= maxArmAngle){ //change degrees
    topsDrive.tankDrive(armSpeed, reArmSpeed);
  }
  if (topRight.getSelectedSensorPosition() >= minArmAngle){ //change degrees
    topsDrive.tankDrive(reArmSpeed, armSpeed);
  } 
}

public void limitArmRotateSpeed(double getArmDegValue) {
  
}
/*
private void pulsePiston(double Time) {
  int pulseFreq = 15;
  int pulseDuration = 1;
    if (Time % pulseFreq < pulseDuration) {
    coolingSolenoid.set(Value.kForward);
  } else {
    coolingSolenoid.set(Value.kReverse);
  }
} */
//some more functions~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  @Override
  public void robotInit() {
  timer.reset();
  timer.start();
  topRight.setSelectedSensorPosition(0);
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

  // Set the direction of the integrated encoder
  leftFront.setSensorPhase(false);
  rightFront.setSensorPhase(false);
  topRight.setSensorPhase(false);

  //resetvvvvvvvvvvvvvvvvvv
  resetEncoder_DriveFeet();
  resetEncoder_TurnDegrees();
  resetEncoder_armLift();
  /*
   * leftFront.configSelectedFeedbackCoefficient(0);
   * rightFront.configSelectedFeedbackCoefficient(0);
   * topRight.configSelectedFeedbackCoefficient(0);
   */
  }

  @Override
  public void robotPeriodic() { 
  //limitArmRotation(topRight.getSelectedSensorPosition());

  
  SmartDashboard.putNumber("topRight.Encoder Degrees: ", topRight.getSelectedSensorPosition());
  SmartDashboard.putNumber("leftFront.Encoder Feet: ", leftFront.getSelectedSensorPosition());
  SmartDashboard.putNumber("rightFront.Encoder Degrees: ", rightFront.getSelectedSensorPosition());
  /*
  double talonDeg_TopRight = encoderToDegrees(countsPerRevTalonFX, topRight.getSelectedSensorPosition(), armLift_GearRatio);
  SmartDashboard.putNumber("topRight.Encoder Degrees: ", talonDeg_TopRight);
  limitArmRotation(talonDeg_TopRight);

  double talonDeg_rightFront = encoderToDegrees(countsPerRevTalonFX, rightFront.getSelectedSensorPosition(), arcadeDrive_GearRatio);
  SmartDashboard.putNumber("rightFront.Encoder Degrees: ", talonDeg_rightFront);
  
  double leftFront_Feet = encoderToFeet(radiusRizzArcadeWheels, countsPerRevTalonFX, leftFront.getSelectedSensorPosition(), arcadeDrive_GearRatio);
  SmartDashboard.putNumber("leftFront.Encoder Feet: ", leftFront_Feet);
   */
  
  //^^encoder
  SmartDashboard.putNumber("navx getPitch: ", navx.getPitch());
  SmartDashboard.putNumber("navx getYaw: ", navx.getYaw());
  SmartDashboard.putNumber("navx getRoll: ", navx.getRoll());
  SmartDashboard.putNumber("navx getAngle: ", navx.getAngle());
  //^^navx Angles
  SmartDashboard.putBoolean("claw || close: ", bButtonPressed);
  SmartDashboard.putBoolean("BalanceCode: ", xButtonPressed);
  //^^ButtonToggles
  }
  
  @Override
  public void autonomousInit() {
    Timer.delay(5);
    drive_AUTONOMOUS(6, 1, false); //fwd 6 feet
    Timer.delay(5);
    drive_AUTONOMOUS(-6, 1, false); //Bkwd 6 feet
    Timer.delay(5);
    drive_AUTONOMOUS(180, 1, true); //turn 180
    Timer.delay(5);
    drive_AUTONOMOUS(-180, 1, true); //turn back 180 
    Timer.delay(5);
    armLift_Lower(-45, 5); // raise arm 45 degrees
    Timer.delay(5);
    armExtender(3, 0.5);// extend 3 feet
    Timer.delay(5);
    armExtender(-3, 0.5);// retract 3 feet
    Timer.delay(5);
    armLift_Lower(45, 5); // lower 45 deg

  }

  @Override
  public void autonomousPeriodic() {
    double AutonomousTime = timer.get();
    //pulsePiston(AutonomousTime);
    autoBalance_AUTONOMOUS();
    /*
     * thread armLift and Arm Lower for autonomous: 
     * note: thread what we can for down the line stuff.
     * also idea: thread section of autonomous as long as 1 thread aint using the same thing twice
     * 
note: Create a new thread for armLift_Lower
Thread thread_ArmLift = new Thread(() -> {
  armLift_Lower(targetDistanceDegrees, tolerance);
});

note: Create a new thread for armExtender
Thread thread_ArmExtend = new Thread(() -> {
  armExtender(targetDistanceFeet, tolerance);
});

note: to Start both threads ie
thread_ArmLift.start();
thread_ArmExtend.start();

//Note: tolerance for now = 0 but needs testing
  * robot Init
  * armLift_Lower // line to drop object
  * armExtension // line extend over pole/space
  * clawOpen // drop object 
  * armExtension // retract arm
  * drive_AUTONOMOUS(18, 0, false); // drive and !!TURN!! to be lined up to get next object 18 feet away (not calculated ramp)
  * clawClose // pickup object
  * armLift_Lower // to driving position/ set position
  * drive_AUTONOMOUS(-18, 0, false); // go back to home
  * clawOpen // drop object
  * drive_AUTONOMOUS(7.9375, 0, false); // go to platform and balance
  * autoBalance_AUTONOMOUS(); // BALANCE FOR LEFT OVER TIME
  */
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
    double teleopTime = timer.get();
    //pulsePiston(teleopTime);
  
  //drive vvv
  m_robotDrive.arcadeDrive(xBoxCont.getRawAxis(4) * 0.55, xBoxCont.getRawAxis(1) * 0.7);

  //arm angle vvv
 if (xBoxCont.getRightTriggerAxis() >= 0.5) {
     topsDrive.tankDrive(0.5,-0.5); //adjusted armSPeed and reArmSPeed
 }else if (xBoxCont.getLeftTriggerAxis() >= 0.5) {
     topsDrive.tankDrive(-0.45,0.45);
 }else {
     topsDrive.tankDrive(0,0);
 } 

// arm extendo vvv
 if (xBoxCont.getLeftBumper() == true){
  armTalonExtenstion.set(.30);
} else if (xBoxCont.getRightBumper() == true){
  armTalonExtenstion.set(-0.30);
} else {
  armTalonExtenstion.set(0);
}

 //BButton aka CLAW vvv
 if (xBoxCont.getBButtonPressed() == true) {
   bButtonPressed = !bButtonPressed;
 } 
 if (bButtonPressed == true){
   dSolenoidClaw.set(Value.kForward);
 } else if (bButtonPressed == false){
   dSolenoidClaw.set(Value.kReverse);
 }

//XButton aka balance vvv
 if (xBoxCont.getXButtonPressed() == true) {
  xButtonPressed = !xButtonPressed;
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

 boolean bButtonPressed = false;
 boolean xButtonPressed = false;
 double deadzone = 0.5;

}
/*
 * bumbers = arm extenstion
 * triggers = TankDrive aka armLift
 * B button = claw open close
 * X button = toggle balance code teleop
 * JoyStick = fwd, back, left and right
 */
