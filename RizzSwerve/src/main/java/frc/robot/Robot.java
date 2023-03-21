package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.SPI;

// getangle for 360 instead of getYaw which is over


public class Robot extends TimedRobot {
  //Constants vvvvv
  Timer timer = new Timer();
  final XboxController driving_xBoxCont = new XboxController(0);
  final XboxController arm_xBoxCont = new XboxController(1);
  double maxDegree = 360; // if your over 360 then your driving to much
  
  //these are used for swerve vvv
  final double kp = 0.3;
  final double ki = 0.0;
  final double kd = 0.0;

  PIDController pidFrontLeftTurn = new PIDController(kp, ki, kd);
  PIDController pidFrontRightTurn = new PIDController(kp, ki, kd);
  PIDController pidBackLeftTurn = new PIDController(kp, ki, kd);
  PIDController pidBackRightTurn = new PIDController(kp, ki, kd);

  PIDController drive = new PIDController(kp, ki, kd); // this will be used for driving in feet direction

  public final WPI_TalonFX frontLeftDrive = new WPI_TalonFX(2);
  public final WPI_TalonFX frontRightDrive= new WPI_TalonFX(4);
  public final WPI_TalonFX backLeftDrive  = new WPI_TalonFX(6);
  public final WPI_TalonFX backRightDrive = new WPI_TalonFX(8); 

  public final WPI_TalonFX frontLeftSteer = new WPI_TalonFX(1);
  public final WPI_TalonFX frontRightSteer = new WPI_TalonFX(3);
  public final WPI_TalonFX backLeftSteer = new WPI_TalonFX(5);
  public final WPI_TalonFX backRightSteer = new WPI_TalonFX(7); 

  public final WPI_CANCoder frontLeftAbsEncoder = new WPI_CANCoder(1);
  public final WPI_CANCoder frontRightAbsEncoder = new WPI_CANCoder(2);
  public final WPI_CANCoder backLeftAbsEncoder = new WPI_CANCoder(3);
  public final WPI_CANCoder backRightAbsEncoder = new WPI_CANCoder(4);

  //put in the unique angle offsets
  public double frontLeftAbsOffset = 0;
  public double frontRightAbsOffset = 0;
  public double backLeftAbsOffset = 0;
  public double backRightAbsOffset = 0;

  public double frontLeftAbsAngle = 0;
  public double frontRightAbsAngle = 0;
  public double backLeftAbsAngle = 0;
  public double backRightAbsAngle = 0;
  
  public final double kWheelDiameterMeters = Units.inchesToMeters(3.75);
  public final double kDriveMotorGearRatio = 1 / 8.14;
  public final double kTurningMotorGearRatio = 1.0 / (150.0/7.0); //motor rotations to wheel rotations conversion factor
  public final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
  public final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI; 
  public final double kTurningEncoderTicksToMetresPerSec = kDriveEncoderRot2Meter/2048;
  public final double kTurningEncoderTicksToRad = kTurningEncoderRot2Rad/2048;

  double encoderLeftFrontDrive_Meteres;
  double encoderleftFrontSteer_Rad;

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
  public final WPI_TalonFX rightArmSide= new WPI_TalonFX(5);
  private final DifferentialDrive armRotate = new DifferentialDrive(leftArmSide, rightArmSide); 
  final WPI_TalonFX armTalonExtenstion = new WPI_TalonFX(7);
  double maxArmDeg = 60;
  double minArmDeg = -5;
  double armSpeed_Fast =  0.50;
  double armSpeed_Slow = 0.45;
  double armTalonExtenstionSpeed = 0.3;

  //Claw and pnuematics aka claw vvv  
  private final DoubleSolenoid dSolenoidClaw = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 3, 0);
  private final Compressor compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
  private final double Scale = 250, offset = -25;
  private final AnalogPotentiometer potentiometer = new AnalogPotentiometer(0, Scale, offset);
  boolean bButtonPressed = false;

  //navx2 vvv
  final double kp_Pitch = 0.1; 
  final double kp_Yaw = 0.1;
  final double ki_Navx = 0.05;
  final double kd_Navx = 0.05;
  AHRS navx = new AHRS(SPI.Port.kMXP);  
  PIDController pidPitch = new PIDController(kp_Pitch, ki_Navx, kd_Navx);
  PIDController pidYaw = new PIDController(kp_Yaw, ki_Navx, kd_Navx);
  double navxYaw_Deg;
  double navxPitch_Deg;
  double navxRoll_Deg;
  // constants ^^^^^

  // our functions vvvvvv
/*
  public double encoderToMetres(double radius, double countsPerRev, double encoderValue, double gearRatio){
    double circumferenceOfWheel = 2*Math.PI*radius; 
    double countsPerInch = countsPerRev / circumferenceOfWheel / gearRatio;
    double inches = encoderValue / countsPerInch;
    double metres = Units.inchesToMeters(inches);
    return metres;
  }
 */

  public void resetEncoders () {
    frontLeftSteer.setSelectedSensorPosition(0);
    frontRightSteer.setSelectedSensorPosition(0);
    backLeftSteer.setSelectedSensorPosition(0);
    backRightSteer.setSelectedSensorPosition(0);
  }

  public void setMotorBreaks () {
    frontLeftDrive.setNeutralMode(NeutralMode.Brake);
    frontRightDrive.setNeutralMode(NeutralMode.Brake);
    backLeftDrive.setNeutralMode(NeutralMode.Brake);
    backRightDrive.setNeutralMode(NeutralMode.Brake);

    frontLeftSteer.setNeutralMode(NeutralMode.Brake);
    frontRightSteer.setNeutralMode(NeutralMode.Brake);
    backLeftSteer.setNeutralMode(NeutralMode.Brake);
    backRightSteer.setNeutralMode(NeutralMode.Brake);

    leftArmSide.setNeutralMode(NeutralMode.Brake);
    rightArmSide.setNeutralMode(NeutralMode.Brake);
    armTalonExtenstion.setNeutralMode(NeutralMode.Brake);
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
    if (Math.abs(driving_xBoxCont.getRawAxis(axisInput)) < 0.1) {
      return 0;
    }
    return driving_xBoxCont.getRawAxis(axisInput);
  }

  public void swerveDrive(double xSpeed, double ySpeed, double rotSpeed) {
    ChassisSpeeds desiredSpeeds = new ChassisSpeeds(xSpeed*maxSpeedMpS, ySpeed*maxSpeedMpS, rotSpeed);
    
    //make desiredSpeeds into speeds and angles for each module
    SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(desiredSpeeds);

    //normalize module values to remove impossible speed values
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, maxSpeedMpS);
    
    SwerveModuleState frontLeftModule = moduleStates[0];
    SwerveModuleState frontRightModule = moduleStates[1];
    SwerveModuleState backLeftModule = moduleStates[2];
    SwerveModuleState backRightModule = moduleStates[3];

    //optimize wheel angles (ex. wheel is at 359deg and needs to go to 1deg. wheel will now go 2deg instead of 358deg)

    double frontLeftSensorPos = frontLeftSteer.getSelectedSensorPosition()*kTurningEncoderTicksToRad;
    double frontRightSensorPos = frontRightSteer.getSelectedSensorPosition()*kTurningEncoderTicksToRad;
    double backLeftSensorPos = backLeftSteer.getSelectedSensorPosition()*kTurningEncoderTicksToRad;
    double backRightSensorPos = backRightSteer.getSelectedSensorPosition()*kTurningEncoderTicksToRad;

    var frontLeftCurrentAngle = new Rotation2d(frontLeftSensorPos);
    var frontRightCurrentAngle = new Rotation2d(frontRightSensorPos);
    var backLeftCurrentAngle = new Rotation2d(backLeftSensorPos);
    var backRightCurrentAngle = new Rotation2d(backRightSensorPos);
    
    var frontLeftOptimized = SwerveModuleState.optimize(frontLeftModule, frontLeftCurrentAngle);
    var frontRightOptimized = SwerveModuleState.optimize(frontRightModule, frontRightCurrentAngle);
    var backLeftOptimized = SwerveModuleState.optimize(backLeftModule, backLeftCurrentAngle);
    var backRightOptimized = SwerveModuleState.optimize(backRightModule, backRightCurrentAngle);
    
    //set steer motor power to the pid output of current position in radians and desired position in radians
    double frontLeftTurnPower = pidFrontLeftTurn.calculate(frontLeftSteer.getSelectedSensorPosition()*kTurningEncoderTicksToRad, frontLeftOptimized.angle.getRadians());
    double frontRightTurnPower = pidFrontRightTurn.calculate(frontRightSteer.getSelectedSensorPosition()*kTurningEncoderTicksToRad, frontRightOptimized.angle.getRadians());
    double backLeftTurnPower = pidBackLeftTurn.calculate(backLeftSteer.getSelectedSensorPosition()*kTurningEncoderTicksToRad, backLeftOptimized.angle.getRadians());
    double backRightTurnPower = pidBackRightTurn.calculate(backRightSteer.getSelectedSensorPosition()*kTurningEncoderTicksToRad, backRightOptimized.angle.getRadians());

    //positive is clockwise (right side up)
    frontLeftSteer.set(frontLeftTurnPower);
    frontRightSteer.set(frontRightTurnPower);
    backLeftSteer.set(backLeftTurnPower);
    backRightSteer.set(backRightTurnPower);

    //set drive power to desired speed div max speed to get value between 0 and 1
    frontLeftDrive.set(frontLeftOptimized.speedMetersPerSecond/maxSpeedMpS);
    frontRightDrive.set(frontRightOptimized.speedMetersPerSecond/maxSpeedMpS);
    backLeftDrive.set(backLeftOptimized.speedMetersPerSecond/maxSpeedMpS);
    backRightDrive.set(backRightOptimized.speedMetersPerSecond/maxSpeedMpS);
  }

  public void limitationArm(double getArmDegValue, double setDegree) {
    if (getArmDegValue <= maxArmDeg){
      armRotate.tankDrive(armSpeed_Fast, -armSpeed_Fast);
    }
    if (getArmDegValue >= minArmDeg){ 
      armRotate.tankDrive(-armSpeed_Slow, armSpeed_Slow);
    } 
    
  }

  public void robotArm(double armDown, double armUp, Boolean claw_xBox, Boolean extendArm, Boolean retractArm) {
    //arm angle vvv
    if (armDown >= 0.5) {
      armRotate.tankDrive(armSpeed_Fast,-armSpeed_Fast); //adjusted armSPeed and reArmSPeed
    }else if (armUp >= 0.5) {
      armRotate.tankDrive(-armSpeed_Slow ,armSpeed_Slow);
    }else {
      armRotate.tankDrive(0,0);
    } 

    //b Button aka CLAW vvv
     if (claw_xBox == true) {
      bButtonPressed = !bButtonPressed;
    } 
    if (bButtonPressed == true){
      dSolenoidClaw.set(Value.kForward);
    } else if (bButtonPressed == false){
      dSolenoidClaw.set(Value.kReverse);
    }

    // arm extendo vvv
    if (extendArm == true){
      armTalonExtenstion.set(armTalonExtenstionSpeed);
    } else if (retractArm == true){
      armTalonExtenstion.set(-armTalonExtenstionSpeed);
    } else {
      armTalonExtenstion.set(0);
    }
  }

  public void drive_PID(double targetXdistance_Metres, double targetYdistance_Metres, double targetYaw_deg, double tolerance) {
    double currentDistanceY;
    currentDistanceY = encoderleftFrontSteer_Rad;
    double outputYSpeed=0;

    double currentDistanceX;
    currentDistanceX = encoderLeftFrontDrive_Meteres;
    double outputXSpeed=0;

    double currentYaw;
    currentYaw = navxYaw_Deg;
    double outputYaw;
    double outputYaw_Rad=0;

    //currentDistance = currentMotorTurn ? rightFront.getSelectedSensorPosition() : leftFront.getSelectedSensorPosition();
    
    if (Math.abs(targetXdistance_Metres - currentDistanceX) > tolerance) {
      outputXSpeed = drive.calculate(currentDistanceX, targetXdistance_Metres)/maxSpeedMpS;
      //swerveDrive(outputXSpeed, 0, 0);
    }
    if (Math.abs(targetYdistance_Metres - currentDistanceY) > tolerance) {
      outputYSpeed = drive.calculate(currentDistanceY, targetYdistance_Metres)/maxSpeedMpS;
      //swerveDrive(0, outputYSpeed, 0);
    }
    if (Math.abs(targetYaw_deg - currentYaw) > tolerance) {
      outputYaw = pidYaw.calculate(currentYaw, targetYaw_deg);
      outputYaw_Rad = Math.toRadians(outputYaw);
      //swerveDrive(0, 0, outputYaw_Rad);
    }
    swerveDrive(outputXSpeed, outputYSpeed, outputYaw_Rad);
  }

  public void autoBalance() {
    double outputPitch=0;
    double outputYaw=0;
    double outputYawRad=0;
    double currentPitch;
    double currentYaw;
    double targetAnglePitch = 0;
    double targetAngleYaw = 0;
    double tolerance = 5;
    currentPitch = navx.getPitch();
    currentYaw = navx.getYaw();
  
    if (Math.abs(targetAnglePitch - currentPitch) > tolerance) {
      outputPitch = pidPitch.calculate(currentPitch, targetAnglePitch); // turn into precent
    }
    if (Math.abs(targetAngleYaw - currentYaw) > tolerance) {
      outputYaw = pidYaw.calculate(currentYaw, targetAngleYaw);
      outputYawRad = Math.toRadians(outputYaw);
    }
    swerveDrive(outputPitch, 0, outputYawRad);
  }

  //returns in radians
  public void absolutePosition() {
    frontLeftAbsAngle = (frontLeftAbsEncoder.getAbsolutePosition() - frontLeftAbsOffset) * (Math.PI / 180);
    frontRightAbsAngle = (frontRightAbsEncoder.getAbsolutePosition() - frontRightAbsOffset) * (Math.PI / 180);
    backLeftAbsAngle = (backLeftAbsEncoder.getAbsolutePosition() - backLeftAbsOffset) * (Math.PI / 180);
    backRightAbsAngle = (backRightAbsEncoder.getAbsolutePosition() - backRightAbsOffset) * (Math.PI / 180);
  }

  public void straightenModules() {
    //while loop might break things by over going the 20ms cycle time. Redo this part if it doesn't work
    while (Math.abs(frontLeftAbsAngle)>0.05 || 
    Math.abs(frontRightAbsAngle)>0.05 || 
    Math.abs(backLeftAbsAngle)>0.05 || 
    Math.abs(backRightAbsAngle)>0.05) {

    absolutePosition();

    double frontLeftTurnPower = pidFrontLeftTurn.calculate(frontLeftAbsAngle, 0);
    double frontRightTurnPower = pidFrontRightTurn.calculate(frontRightAbsAngle, 0);
    double backLeftTurnPower = pidBackLeftTurn.calculate(backLeftAbsAngle, 0);
    double backRightTurnPower = pidBackRightTurn.calculate(backRightAbsAngle, 0);

    frontLeftSteer.set(frontLeftTurnPower);
    frontRightSteer.set(frontRightTurnPower);
    backLeftSteer.set(backLeftTurnPower);
    backRightSteer.set(backRightTurnPower);
    }
  }

  //execution Functions vvvvv
  @Override
  public void robotInit() {
    setMotorBreaks();
    invertMotors();
    continouousInput();
    straightenModules();
    navx.calibrate();

    navx.reset();

    double currentPsi = potentiometer.get();
    int psiCap = 117;
    if (currentPsi <= psiCap) {
        compressor.enableDigital(); 
    } else if (currentPsi > 119) {
        compressor.disable();
    }
    
    resetEncoders();
  } 

  @Override
  public void robotPeriodic() {
    // limit arm vvv
  double encoderDegrees_rightArmSide = rightArmSide.getSelectedSensorPosition()/maxDegree;
    SmartDashboard.putNumber("encoderDegrees_rightArmSide", encoderDegrees_rightArmSide);
    //limitArmRotation(encoderDegrees_rightArmSide);  

    //encoder drive variables vvv
    encoderLeftFrontDrive_Meteres = frontLeftDrive.getSelectedSensorPosition()*kTurningEncoderTicksToMetresPerSec;
    SmartDashboard.putNumber("Drive_Distance_Metres: ", encoderLeftFrontDrive_Meteres);
    encoderleftFrontSteer_Rad = frontLeftSteer.getSelectedSensorPosition()*kTurningEncoderTicksToRad;
    SmartDashboard.putNumber("Orientation: ", encoderleftFrontSteer_Rad);

    //camera vvv
    UsbCamera camera = CameraServer.startAutomaticCapture();

    // navX2 vvv
    navxYaw_Deg = navx.getYaw();
    SmartDashboard.putNumber("Yaw_Deg", navxYaw_Deg);
    navxPitch_Deg = navx.getPitch();
    SmartDashboard.putNumber("Pitch_deg", navxPitch_Deg);
    navxRoll_Deg = navx.getRoll();
    SmartDashboard.putNumber("Roll_Deg", navxRoll_Deg);

    //Pnuematics vvv
    SmartDashboard.putNumber("Current PSI:", potentiometer.get());
    SmartDashboard.putNumber("frontLeftAbs Offset", frontLeftAbsEncoder.getAbsolutePosition());
    SmartDashboard.putNumber("frontRightAbs Offset", frontRightAbsEncoder.getAbsolutePosition());
    SmartDashboard.putNumber("backLeftAbs Offset", backLeftAbsEncoder.getAbsolutePosition());
    SmartDashboard.putNumber("backRightAbs Offset", backRightAbsEncoder.getAbsolutePosition());
  }
  
  @Override
  public void autonomousInit() {
    drive_PID(3,0,0,0.5);
  }

  @Override
  public void autonomousPeriodic() {
    autoBalance();
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
    Thread swerveThread = new Thread(() -> {
      while (true) {
          double contXSpeed = removeDeadzone(1) * driveSensitivity;
          double contYSpeed = removeDeadzone(0) * driveSensitivity;
          double contTurnSpeed = removeDeadzone(4) * turningSensitivity;
          swerveDrive(contXSpeed, contYSpeed, contTurnSpeed);
          
          //turn field oriented on/off
          if(true){
          double angleRad = Math.toRadians(navx.getAngle());
          SmartDashboard.putNumber("getGyroAngle", navx.getAngle());
          contXSpeed = contXSpeed * Math.cos(angleRad) + contYSpeed * Math.sin(angleRad);
          //is the negative necessary?
          contYSpeed = -contXSpeed * Math.sin(angleRad) + contYSpeed * Math.cos(angleRad);
        }
        swerveDrive(contXSpeed, contYSpeed, contTurnSpeed);
      }
  });
  
  Thread armThread = new Thread(() -> {
      while (true) {
          double armDown = arm_xBoxCont.getLeftTriggerAxis();
          double armUp = arm_xBoxCont.getRightTriggerAxis();
          boolean claw_xBox = arm_xBoxCont.getBButtonPressed();
          boolean extendArm = arm_xBoxCont.getLeftBumper();
          boolean retractArm = arm_xBoxCont.getRightBumper();
          robotArm(armDown, armUp, claw_xBox, extendArm, retractArm);
      }
  });
  
  swerveThread.start();
  armThread.start();
/*
    //swerve vvv (uses driving_xBoxCont)
    double contXSpeed = removeDeadzone(1) * maxSpeedMpS * driveSensitivity;
    double contYSpeed = removeDeadzone(0) * maxSpeedMpS * driveSensitivity;
    double contTurnSpeed = removeDeadzone(4) * turningSensitivity;
    swerveDrive(contXSpeed, contYSpeed, contTurnSpeed);

    //robot arm
    double armDown = arm_xBoxCont.getLeftTriggerAxis();
    double armUp = arm_xBoxCont.getRightTriggerAxis();
    boolean claw_xBox = arm_xBoxCont.getBButtonPressed();
    boolean extendArm = arm_xBoxCont.getLeftBumper();
    boolean retractArm = arm_xBoxCont.getRightBumper();
    robotArm(armDown, armUp, claw_xBox, extendArm, retractArm);
     */
  }
}