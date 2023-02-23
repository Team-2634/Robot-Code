package frc.robot;

//import com.ctre.phoenix.motorcontrol.NeutralMode;
//import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  /*
  final WPI_TalonFX leftFront = new WPI_TalonFX(1);
  final WPI_TalonFX rightFront = new WPI_TalonFX(3);
  final WPI_TalonFX leftBack = new WPI_TalonFX(2);
  final WPI_TalonFX rightBack= new WPI_TalonFX(4); 
  final WPI_TalonFX topRIght= new WPI_TalonFX(6); 
  final WPI_TalonFX topLeft= new WPI_TalonFX(5);
  */
  
  public final CANSparkMax leftFront = new CANSparkMax(17, MotorType.kBrushless);
  public final CANSparkMax rightFront = new CANSparkMax(10, MotorType.kBrushless);
  public final CANSparkMax leftBack = new CANSparkMax(4, MotorType.kBrushless);
  public final CANSparkMax rightBack = new CANSparkMax(18, MotorType.kBrushless); 

  final MotorControllerGroup m_leftSide = new MotorControllerGroup(leftBack, leftFront);
  final MotorControllerGroup m_rightSide = new MotorControllerGroup(rightBack, rightFront);

  final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftSide, m_rightSide);
  //final DifferentialDrive m_clawRaise = new DifferentialDrive(topLeft, topRIght);

  final XboxController xBoxCont = new XboxController(0);

  double setpoint = 0;
  final double kP = 0.5;
  private Encoder encoder = new Encoder(0, 1, false, EncodingType.k4X);
  private final double kDriveTick2Feet = 1.0/128*6* Math.PI/12;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public void encoderFunction() {
      
    encoder.reset();
    if (xBoxCont.getYButton()) {
        setpoint = 10;
    }else if (xBoxCont.getRightBumper()){
        setpoint = 0;
    }

    double sensorPosition = encoder.get()*kDriveTick2Feet;
    double error = setpoint - sensorPosition;
    double outputSpeed = kP * error;

    leftBack.set(outputSpeed);
    leftFront.set(outputSpeed);
    rightBack.set(-outputSpeed);
    rightFront.set(-outputSpeed);
  }

  public void setMotorsNeutral() {
    /*
    leftFront.setNeutralMode(NeutralMode.Brake);
    leftBack.setNeutralMode(NeutralMode.Brake);
    rightFront.setNeutralMode(NeutralMode.Brake);
    rightBack.setNeutralMode(NeutralMode.Brake);
     */
    leftFront.setIdleMode(IdleMode.kBrake);
    leftBack.setIdleMode(IdleMode.kBrake);
    rightFront.setIdleMode(IdleMode.kBrake);
    rightBack.setIdleMode(IdleMode.kBrake);
  }
  
@Override
  public void robotInit() {
  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("encoder value", encoder.get()*kDriveTick2Feet);
  }

  @Override
  public void autonomousInit() {
    setMotorsNeutral();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    /*
     * align with starting 
     * rais arm
     * set down object
     * drive to next object
     * pick up
     * drive and align (with new location)
     * set object
     * balance
     */
    
     encoderFunction();
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    m_robotDrive.arcadeDrive(xBoxCont.getRawAxis(4) * 0.8, xBoxCont.getRawAxis(1) * 0.8);
/*
    if (xBoxCont.getAButton() == true) {
      m_clawRaise.tankDrive(1,-1);
    } else if (xBoxCont.getBButton() == true){
      m_clawRaise.tankDrive(-1,1);
    } else {
      m_clawRaise.tankDrive(-0,0);
    } */
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
