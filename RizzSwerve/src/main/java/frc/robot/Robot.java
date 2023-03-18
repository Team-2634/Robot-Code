package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.SPI;

public class Robot extends TimedRobot {

  //Constants vvvvv
  Timer timer = new Timer();
  final XboxController xBoxCont = new XboxController(0);
  double maxDegree = 360; // if your over 360 then your driving to much
  
  //these are used for swerve vvv
  final double kp = 0.3;
  final double ki = 0.0;
  final double kd = 0.0;

  PIDController pidFrontLeftTurn = new PIDController(kp, ki, kd);
  PIDController pidFrontRightTurn = new PIDController(kp, ki, kd);
  PIDController pidBackLeftTurn = new PIDController(kp, ki, kd);
  PIDController pidBackRightTurn = new PIDController(kp, ki, kd);

  public final WPI_TalonFX frontLeftDrive = new WPI_TalonFX(2);
  public final WPI_TalonFX frontRightDrive = new WPI_TalonFX(4);
  public final WPI_TalonFX backLeftDrive = new WPI_TalonFX(6);
  public final WPI_TalonFX backRightDrive= new WPI_TalonFX(8); 

  public final WPI_TalonFX frontLeftSteer = new WPI_TalonFX(1);
  public final WPI_TalonFX frontRightSteer = new WPI_TalonFX(3);
  public final WPI_TalonFX backLeftSteer = new WPI_TalonFX(5);
  public final WPI_TalonFX backRightSteer = new WPI_TalonFX(7); 
  
  public final double kWheelDiameterMeters = Units.inchesToMeters(3.75);
  public final double kDriveMotorGearRatio = 1 / 8.45;
  public final double kTurningMotorGearRatio = 1.0 / (150.0/7.0); //motor rotations to wheel rotations conversion factor
  public final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
  public final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI; 
  public final double kTurningEncoderTicksToRad = kTurningEncoderRot2Rad/2048;

  double driveSensitivity = 0.4; //do not change above 1
  double turningSensitivity = 2;
  double maxSpeedMpS = 5; // metres/sec

  Translation2d m_frontLeftLocation = new Translation2d(0.340, 0.285);
  Translation2d m_frontRightLocation = new Translation2d(0.340, -0.285);
  Translation2d m_backLeftLocation = new Translation2d(-0.340, 0.285);
  Translation2d m_backRightLocation = new Translation2d(-0.340, -0.285);

  SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
  m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  //these are for the arm vvv
  public final WPI_TalonFX leftArmSide= new WPI_TalonFX(6); 
  public final WPI_TalonFX rightArnSide= new WPI_TalonFX(5);
  private final DifferentialDrive armRotate = new DifferentialDrive(leftArmSide, rightArnSide); 
  final WPI_TalonFX armTalonExtenstion = new WPI_TalonFX(7);

  //Claw and pnuematics vvv  
  private final DoubleSolenoid dSolenoidClaw = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 3, 0);
  private final Compressor compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
  private final double Scale = 250, offset = -25;
  private final AnalogPotentiometer potentiometer = new AnalogPotentiometer(0, Scale, offset);

  //navx2 vvv
  final double kp_Pitch = 0.1; 
  final double kp_Yaw = 0.1;
  final double ki_Navx = 0.05;
  final double kd_Navx = 0.05;
  AHRS navx = new AHRS(SPI.Port.kMXP);  
  PIDController pidPitch = new PIDController(kp_Pitch, ki_Navx, kd_Navx);
  PIDController pidYaw = new PIDController(kp_Yaw, ki_Navx, kd_Navx);
  //constants ^^^^^

  // our functions vvvvvv

  public void resetEncoders () {
    frontLeftSteer.setSelectedSensorPosition(0);
    frontRightSteer.setSelectedSensorPosition(0);
    backLeftSteer.setSelectedSensorPosition(0);
    backRightSteer.setSelectedSensorPosition(0);
  }

  public void setMotorBreaks () {
    frontLeftSteer.setNeutralMode(NeutralMode.Brake);
    frontRightSteer.setNeutralMode(NeutralMode.Brake);
    backLeftSteer.setNeutralMode(NeutralMode.Brake);
    backRightSteer.setNeutralMode(NeutralMode.Brake);

    frontLeftSteer.setNeutralMode(NeutralMode.Brake);
    frontRightSteer.setNeutralMode(NeutralMode.Brake);
    backLeftSteer.setNeutralMode(NeutralMode.Brake);
    backRightSteer.setNeutralMode(NeutralMode.Brake);
  }

  public void invertMotors () {
    frontLeftSteer.setInverted(true);
    frontRightSteer.setInverted(true);
    backLeftSteer.setInverted(true);
    backRightSteer.setInverted(true);

    frontLeftDrive.setInverted(true);
    frontRightDrive.setInverted(true);
    backLeftDrive.setInverted(true);
    backRightDrive.setInverted(true);
  }

  public void continouousInput () {
    pidFrontLeftTurn.enableContinuousInput(-Math.PI, Math.PI);
    pidFrontRightTurn.enableContinuousInput(-Math.PI, Math.PI);
    pidBackLeftTurn.enableContinuousInput(-Math.PI, Math.PI);
    pidBackRightTurn.enableContinuousInput(-Math.PI, Math.PI);
  }

  public double removeDeadzone(int axisInput) {
    if (Math.abs(xBoxCont.getRawAxis(axisInput)) < 0.1) {
      return 0;
    }
    return xBoxCont.getRawAxis(axisInput);
  }

  public void swerveDrive(double xSpeed, double ySpeed, double rotSpeed) {
    ChassisSpeeds desiredSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rotSpeed);
  }

  //execution Functions vvvvv
  @Override
  public void robotInit() {
    resetEncoders();
    setMotorBreaks();
    invertMotors();
    continouousInput();
  }

  @Override
  public void robotPeriodic() {

  }
  
  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
    
  }

  @Override
  public void teleopInit() {

  }

  @Override
  public void teleopPeriodic() {

  }
}
