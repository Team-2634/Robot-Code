package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
 
  public final WPI_TalonFX leftFront = new WPI_TalonFX(1);
  public final WPI_TalonFX rightFront = new WPI_TalonFX(3);
  public final WPI_TalonFX leftBack = new WPI_TalonFX(2);
  public final WPI_TalonFX rightBack= new WPI_TalonFX(4); 
  public final WPI_TalonFX topRIght= new WPI_TalonFX(6); 
  public final WPI_TalonFX topLeft= new WPI_TalonFX(5); 
  
  public final Encoder leftFrontEncoder = new Encoder(0,1, false); // not needed

  public final Encoder topRightEncoder = new Encoder(5, 6);
  public final Encoder topLeftEncoder = new Encoder(2, 3);

  /*
  public final CANSparkMax leftFront = new CANSparkMax(11, MotorType.kBrushless);
  public final CANSparkMax rightFront = new CANSparkMax(6, MotorType.kBrushless);
  public final CANSparkMax leftBack = new CANSparkMax(5, MotorType.kBrushless);
  public final CANSparkMax rightBack = new CANSparkMax(8, MotorType.kBrushless); 
*/
  final MotorControllerGroup m_leftSide = new MotorControllerGroup(leftBack, leftFront);
  final MotorControllerGroup m_rightSide = new MotorControllerGroup(rightBack, rightFront);

  final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftSide, m_rightSide);
 private final DifferentialDrive topsDrive = new DifferentialDrive(topLeft, topRIght);

  final XboxController xBoxCont = new XboxController(0);

  final double kp = 0.5;
  final double ki = 0.05;
  final double kd = 0.05;
  boolean xButtonPressed = false;
  double output;

  double armSpeed = 0.7;
  double reArmSpeed = -0.7;
  double radius = 6.5/2; //of wheel in inchs
  double circumferenceOfWheel = 2*Math.PI*radius;
  double pulesPerRevTalonFX = 2048;
  double distancePerPulse = circumferenceOfWheel / pulesPerRevTalonFX;

  public void driveForward() {
    PIDController driveFwdPid = new PIDController(kp, ki, kd);    
    double targetDistance = 5;
    double tolerance = 0.5;

    driveFwdPid.reset();
    driveFwdPid.setSetpoint(targetDistance);

    while (Math.abs(targetDistance - leftFrontEncoder.getDistance()) > tolerance) {
      output = driveFwdPid.calculate(leftFrontEncoder.getDistance(), targetDistance);
      m_robotDrive.arcadeDrive(0, -output);
    }
    
    m_robotDrive.stopMotor();
    driveFwdPid.close();
  }

  public void setMotorsNeutral() {
    
    leftFront.setNeutralMode(NeutralMode.Brake);
    leftBack.setNeutralMode(NeutralMode.Brake);
    rightFront.setNeutralMode(NeutralMode.Brake);
    rightBack.setNeutralMode(NeutralMode.Brake);
     /*
    leftFront.setIdleMode(IdleMode.kBrake);
    leftBack.setIdleMode(IdleMode.kBrake);
    rightFront.setIdleMode(IdleMode.kBrake);
    rightBack.setIdleMode(IdleMode.kBrake);
    */
  }
  
@Override
  public void robotInit() {
    topRIght.setSelectedSensorPosition(0, 0, 10);
    topLeft.setSelectedSensorPosition(0, 0, 10);

    topRightEncoder.setDistancePerPulse(distancePerPulse); 
    topLeftEncoder.setDistancePerPulse(distancePerPulse);

    // Set the direction of the encoders
    topRightEncoder.setReverseDirection(false);
    topLeftEncoder.setReverseDirection(false);

    // Reset the encoders to zero
    topRightEncoder.reset();
    topLeftEncoder.reset();
  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("leftFrontEncoderDIS", leftFrontEncoder.getDistance());
    SmartDashboard.putNumber("leftFrontEncoderPuls", leftFrontEncoder.getDistancePerPulse());
    SmartDashboard.putNumber("Output PID", output);
    SmartDashboard.putNumber("topRight sensorPos",  topRIght.getSelectedSensorPosition());

  }

  @Override
  public void autonomousInit() {
    setMotorsNeutral();
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

  public void limitArmRotation() {

    while (topRIght.getSelectedSensorPosition() >= 90){
      topsDrive.tankDrive(armSpeed, reArmSpeed);
    } 
    while (topRIght.getSelectedSensorPosition() <= 25){
      topsDrive.tankDrive(reArmSpeed, armSpeed);
    } 
  }
 
  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    limitArmRotation();

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

  if (xButtonPressed == true) {
    driveForward();

  SmartDashboard.putBoolean("driveForward(); mode: ", xButtonPressed);
    

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
  } 
  }
}
  }
}