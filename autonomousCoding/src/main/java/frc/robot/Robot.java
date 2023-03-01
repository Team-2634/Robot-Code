package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.Faults;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
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

  final Faults _faults = new Faults(); 

  final double kp = 0.5;
  final double ki = 0.05;
  final double kd = 0.05;
  boolean xButtonPressed = false;

/*
  private double getDistanceTraveled() {
    final double wheelDiameter = 6.5;
    final double encoderCPR = 2048;
    final double gearRatio = 10.71;
    
    double counts = (leftFront.getSelectedSensorPosition() + rightFront.getSelectedSensorPosition()) / 2.0;
    double revolutions = counts / encoderCPR;
    double distance = Math.PI * wheelDiameter * revolutions / gearRatio / 12.0; // in feet
    
    return distance;
  }

  private void driveForward(double distance) {
    PIDController drive = new PIDController(kp, ki, kd);
    double targetDistance = distance;
    double tolerance = 0.5;
    
    drive.reset();
    drive.setSetpoint(targetDistance);
    
    while (Math.abs(targetDistance - getDistanceTraveled()) > tolerance) {
      double output = drive.calculate(getDistanceTraveled());
      m_robotDrive.arcadeDrive(output, 0);
    }
    
    m_robotDrive.stopMotor();
    drive.close();
  }  */

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
  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Sensor Vel:", topRIght.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Sensor Pos:", topRIght.getSelectedSensorPosition());
    SmartDashboard.putNumber("Out %:", topRIght.getMotorOutputPercent());
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
    topRIght.configFactoryDefault();
    topRIght.setInverted(false);
    topRIght.setSensorPhase(false); 
  }

  private void limitArmRotation() {
    /*
     * get value of encoder with rather
     * a) topRIght.getSelectedSensorPosition()                                    aka relative position
     * b) topRIght.getSensorCollection().setIntegratedSensorPositionToAbsolute()  aka absolute position
     * https://www.chiefdelphi.com/t/falcon-500-talons-fx-absolute-encoder-reset/426613/11
     * or configur aboslute in pheniox tuner 
     * switch it to deg (if can)
     * 
     * while motor past x degrees then stop motor/set motor to a good position
     * can do with if as well
     */
    while (topRIght.getSelectedSensorPosition() >= 2 && topRIght.getSelectedSensorPosition() <= 3){
      topsDrive.stopMotor();
    }
  }

  /** This function is called periodically during operator control. */
  double armSpeed = 0.7;
  @Override
  public void teleopPeriodic() {
    m_robotDrive.arcadeDrive(xBoxCont.getRawAxis(4) * 0.8, xBoxCont.getRawAxis(1) * 0.8);

    if (xBoxCont.getLeftBumper() == true) {
      topsDrive.tankDrive(armSpeed,-armSpeed);
      topLeft.setNeutralMode(NeutralMode.Brake);
      topRIght.setNeutralMode(NeutralMode.Brake);
  }else if (xBoxCont.getRightBumper() == true) {
      topsDrive.tankDrive(-armSpeed,armSpeed);
      topLeft.setNeutralMode(NeutralMode.Brake);
      topRIght.setNeutralMode(NeutralMode.Brake);
  }else {
      topsDrive.tankDrive(0,0);
      topLeft.setNeutralMode(NeutralMode.Brake);
      topRIght.setNeutralMode(NeutralMode.Brake);
  }
  }
}