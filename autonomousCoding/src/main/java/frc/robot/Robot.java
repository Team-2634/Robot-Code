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
  
  public final CANSparkMax leftFront = new CANSparkMax(11, MotorType.kBrushless);
  public final CANSparkMax rightFront = new CANSparkMax(6, MotorType.kBrushless);
  public final CANSparkMax leftBack = new CANSparkMax(5, MotorType.kBrushless);
  public final CANSparkMax rightBack = new CANSparkMax(8, MotorType.kBrushless); 

  final MotorControllerGroup m_leftSide = new MotorControllerGroup(leftBack, leftFront);
  final MotorControllerGroup m_rightSide = new MotorControllerGroup(rightBack, rightFront);

  final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftSide, m_rightSide);
  //final DifferentialDrive m_clawRaise = new DifferentialDrive(topLeft, topRIght);

  final XboxController xBoxCont = new XboxController(0);

  boolean yButtonPressed = false;

  double setpoint = 0;
  final double kP = 0.05;
  private Encoder encoder = new Encoder(0, 1, false, EncodingType.k4X);//!!!!!
  private final double kDriveTick2Feet = 1.0/128*6* Math.PI/12;
 
  public void encoderFunction() {
    
    encoder.reset();
    if (xBoxCont.getYButton()) {
        setpoint = 10;
    }else if (xBoxCont.getXButtonPressed()){
        setpoint = 0;
    }

    double sensorPosition = encoder.get()*kDriveTick2Feet;
    double error = setpoint - sensorPosition;
    double outputSpeed = kP * error;

    leftBack.set(outputSpeed);
    leftFront.set(outputSpeed);
    rightBack.set(-outputSpeed);
    rightFront.set(-outputSpeed);

    SmartDashboard.getNumber("outputSpeed", outputSpeed);
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
    SmartDashboard.putNumber("kP", kP);
   // SmartDashboard.putNumber("Joystick input 10 feet", xBoxCont.);
    SmartDashboard.putNumber("encoder.get", encoder.get());
    SmartDashboard.putNumber("encoder", encoder.get()*kDriveTick2Feet);
    SmartDashboard.putNumber("error",  setpoint - encoder.get()*kDriveTick2Feet);
    SmartDashboard.putNumber("outPut", kP * setpoint - encoder.get()*kDriveTick2Feet);
  }

  @Override
  public void autonomousInit() {
    setMotorsNeutral();
    encoder.reset();
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
    encoder.reset();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    m_robotDrive.arcadeDrive(xBoxCont.getRawAxis(4) * 0.8, xBoxCont.getRawAxis(1) * 0.8);
    
    if (xBoxCont.getYButtonPressed() == true){
            yButtonPressed = !yButtonPressed;
        }
    
    double deadzone = 0.5;
      if (xBoxCont.getRawAxis(4) > deadzone ||
      xBoxCont.getRawAxis(4) < -deadzone &&
      xBoxCont.getRawAxis(1) > deadzone || 
      xBoxCont.getRawAxis(1) < -deadzone) {
            yButtonPressed = false;
        }

        if (yButtonPressed == true) {
          encoderFunction();
        }
    //encoderFunction();
    
/*
    if (xBoxCont.getAButton() == true) {
      m_clawRaise.tankDrive(1,-1);
    } else if (xBoxCont.getBButton() == true){
      m_clawRaise.tankDrive(-1,1);
    } else {
      m_clawRaise.tankDrive(-0,0);
    } 
*/
  }
}
