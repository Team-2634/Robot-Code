package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.SPI;

public class Robot extends TimedRobot {
    // Constants vvvvv
    Timer timer = new Timer();
    double autonomousStartTime;
    double targetDistance_Xauto = 0;
    double targetDistance_Yauto = 0;
    double targetRad_auto = 0;
    final XboxController driving_xBoxCont = new XboxController(0);
    final XboxController arm_xBoxCont = new XboxController(1);
    double maxDegree = 360; // if your over 360 then your driving to much
    private boolean isFirstTime = true;
    double talonEncoder_TicksPerRev = 2048;
    double neoEncoder_TicksPerRev = 42;

    double contXSpeedField;
    double contYSpeedField;

    // these are used for swerve vvv
    final double kp = 0.3;
    final double ki = 0.01;
    final double kd = 0.01;

    PIDController pidFrontLeftTurn = new PIDController(kp, ki, kd);
    PIDController pidFrontRightTurn = new PIDController(kp, ki, kd);
    PIDController pidBackLeftTurn = new PIDController(kp, ki, kd);
    PIDController pidBackRightTurn = new PIDController(kp, ki, kd);

    PIDController drive = new PIDController(kp, ki, kd);

    public final WPI_TalonFX frontLeftDrive = new WPI_TalonFX(7);
    public final WPI_TalonFX frontRightDrive = new WPI_TalonFX(1);
    public final WPI_TalonFX backLeftDrive = new WPI_TalonFX(5);
    public final WPI_TalonFX backRightDrive = new WPI_TalonFX(3);

/*
    public final CANSparkMax frontLeftDrive = new CANSparkMax(17, MotorType.kBrushless);
    public final CANSparkMax frontRightDrive = new CANSparkMax(10, MotorType.kBrushless);
    public final CANSparkMax backLeftDrive = new CANSparkMax(4, MotorType.kBrushless);
    public final CANSparkMax backRightDrive = new CANSparkMax(18, MotorType.kBrushless);
 */

    public final WPI_TalonFX frontLeftSteer = new WPI_TalonFX(6);
    public final WPI_TalonFX frontRightSteer = new WPI_TalonFX(0);
    public final WPI_TalonFX backLeftSteer = new WPI_TalonFX(4);
    public final WPI_TalonFX backRightSteer = new WPI_TalonFX(2);
    
/*
    public final CANSparkMax frontLeftSteer = new CANSparkMax(17, MotorType.kBrushless);
    public final CANSparkMax frontRightSteer = new CANSparkMax(10, MotorType.kBrushless);
    public final CANSparkMax backLeftSteer = new CANSparkMax(4, MotorType.kBrushless);
    public final CANSparkMax backRightSteer = new CANSparkMax(18, MotorType.kBrushless);
 */

    public final WPI_CANCoder frontLeftAbsEncoder = new WPI_CANCoder(3);
    public final WPI_CANCoder frontRightAbsEncoder = new WPI_CANCoder(0);
    public final WPI_CANCoder backLeftAbsEncoder = new WPI_CANCoder(2);
    public final WPI_CANCoder backRightAbsEncoder = new WPI_CANCoder(1);

    final public double frontLeftAbsOffset = 197.19;
    final public double frontRightAbsOffset = 31.8;
    final public double backLeftAbsOffset = 285.205;
    final public double backRightAbsOffset = 50.4;

    public double frontLeftAbsAngle = 0;
    public double frontRightAbsAngle = 0;
    public double backLeftAbsAngle = 0;
    public double backRightAbsAngle = 0;

    public final double kWheelDiameterMeters = Units.inchesToMeters(3.75);
    public final double kDriveMotorGearRatio = 1 / 8.14;
    public final double kTurningMotorGearRatio = 1.0 / (150.0 / 7.0); // motor rotations to wheel rotations conversion
                                                                      // factor
    public final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
    public final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
    public final double kTurningEncoderTicksToMetresPerSec = kDriveEncoderRot2Meter / talonEncoder_TicksPerRev;

    public final double kTurningEncoderTicksToRad = kTurningEncoderRot2Rad / talonEncoder_TicksPerRev;

    double encoderLeftFrontDriveDisplacement_Meteres;
    double encoderleftFrontSteer_Rad;
    double XdriveSensitivity = 1;
    double YdriveSensitivity = 1; // do not change above 1
    double turningSensitivity = 20; // radians
    double maxSpeedMpS = 20; // metres/sec

    Translation2d m_frontLeftLocation = new Translation2d(0.340, 0.285);
    Translation2d m_frontRightLocation = new Translation2d(0.340, -0.285);
    Translation2d m_backLeftLocation = new Translation2d(-0.340, 0.285);
    Translation2d m_backRightLocation = new Translation2d(-0.340, -0.285);

    SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
            m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

    
    //here we setup arcade/diff
    MotorControllerGroup leftSideTalon = new MotorControllerGroup(frontLeftDrive, backLeftDrive);
    MotorControllerGroup rightSideTalon = new MotorControllerGroup(frontRightDrive, backRightDrive);
    DifferentialDrive autonomousScuffed = new DifferentialDrive(leftSideTalon,rightSideTalon);

    // these are for the arm  liftvvv
    public final WPI_TalonFX leftArmSide = new WPI_TalonFX(9);
    public final WPI_TalonFX rightArmSide = new WPI_TalonFX(8);
    private final DifferentialDrive armRotate = new DifferentialDrive(leftArmSide, rightArmSide);
    double liftArmSide_GearRatio = 64*(60/15);
	double armRotate_ToRad= ((1.0/liftArmSide_GearRatio) * 2 * Math.PI)/2048;
    double armRad_current;
    double kp_armAngle = 0.5, ki_armAngle = 0.05, kd_armAngle = 0.05;
    final PIDController PID_armAngle = new PIDController(kp_armAngle, ki_armAngle, kd_armAngle);
    double maxArmAngleRad = -2;
    double minArmAngleRad= 1;
    double speed_armRotation = 0.75;
    // flag to indicate if arm angle is being limited
    private boolean armAngleLimited = false;

	// arm extend vvv
    final WPI_TalonFX armTalonExtenstion = new WPI_TalonFX(10);
	double armExtenstion_gearRatio= 1 /36.0;
    double armTalonExtenstionSpeed = 0.80; 
    double armExtenstion_ToMetres = (armExtenstion_gearRatio*Math.PI*Units.inchesToMeters(2.75))/2048.0; // metres 
    double extenstionEncoder_CurrentMetres;
    double kp_armE = 0.5, ki_armE, kd_armD; 
    final PIDController pidArmExtensController = new PIDController(kp, ki, kd);

	//claw_Wheels vvv
    private final CANSparkMax claw_Wheels = new CANSparkMax(13, MotorType.kBrushless);
    double ClawIntake_WheelSpeed = -1; 
    double ClawExpel_WheelSpeed = 0.50;

    // pnuematics vvv
    private final DoubleSolenoid dSolenoidClaw = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 5, 4);
	boolean dSolenoidClaw_ButtonPressed = false;

    // navx2 vvv
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
    double angleRad;
    // constants ^^^^^
    // our functions vvvvvv

    public void resetEncoders() {
        frontLeftSteer.setSelectedSensorPosition(0);
        frontRightSteer.setSelectedSensorPosition(0);
        backLeftSteer.setSelectedSensorPosition(0);
        backRightSteer.setSelectedSensorPosition(0);

        rightArmSide.setSelectedSensorPosition(0);
        armTalonExtenstion.setSelectedSensorPosition(0);
    }

    public void setMotorBreaks() {
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
        claw_Wheels.setIdleMode(IdleMode.kBrake);
    }

    public void invertMotors() {
        frontLeftSteer.setInverted(true);
        frontRightSteer.setInverted(true);
        backLeftSteer.setInverted(true);
        backRightSteer.setInverted(true);

        frontLeftDrive.setInverted(true);
        frontRightDrive.setInverted(true);
        backLeftDrive.setInverted(true);
        backRightDrive.setInverted(true);
    }

    public void continouousInput() {
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

    public double removeDeadzone(int axisInput, int axisInput2) {
        if (Math.abs(driving_xBoxCont.getRawAxis(axisInput)) < 0.1 && Math.abs(driving_xBoxCont.getRawAxis(axisInput2)) < 0.1) {
            return 0;
        }
        return driving_xBoxCont.getRawAxis(axisInput);
    }

    public void swerveDrive(double xSpeed, double ySpeed, double rotSpeed) {
        ChassisSpeeds desiredSpeeds = new ChassisSpeeds(xSpeed * maxSpeedMpS, ySpeed * maxSpeedMpS, rotSpeed);

        // make desiredSpeeds into speeds and angles for each module
        SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(desiredSpeeds);

        // normalize module values to remove impossible speed values
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, maxSpeedMpS);

        SwerveModuleState frontLeftModule = moduleStates[0];
        SwerveModuleState frontRightModule = moduleStates[1];
        SwerveModuleState backLeftModule = moduleStates[2];
        SwerveModuleState backRightModule = moduleStates[3];

        // optimize wheel angles (ex. wheel is at 359deg and needs to go to 1deg. wheel
        // will now go 2deg instead of 358deg)

        double frontLeftSensorPos = frontLeftSteer.getSelectedSensorPosition() * kTurningEncoderTicksToRad;
        double frontRightSensorPos = frontRightSteer.getSelectedSensorPosition() * kTurningEncoderTicksToRad;
        double backLeftSensorPos = backLeftSteer.getSelectedSensorPosition() * kTurningEncoderTicksToRad;
        double backRightSensorPos = backRightSteer.getSelectedSensorPosition() * kTurningEncoderTicksToRad;

        var frontLeftCurrentAngle = new Rotation2d(frontLeftSensorPos);
        var frontRightCurrentAngle = new Rotation2d(frontRightSensorPos);
        var backLeftCurrentAngle = new Rotation2d(backLeftSensorPos);
        var backRightCurrentAngle = new Rotation2d(backRightSensorPos);

        var frontLeftOptimized = SwerveModuleState.optimize(frontLeftModule, frontLeftCurrentAngle);
        var frontRightOptimized = SwerveModuleState.optimize(frontRightModule, frontRightCurrentAngle);
        var backLeftOptimized = SwerveModuleState.optimize(backLeftModule, backLeftCurrentAngle);
        var backRightOptimized = SwerveModuleState.optimize(backRightModule, backRightCurrentAngle);

        // set steer motor power to the pid output of current position in radians and
        // desired position in radians
        double frontLeftTurnPower = pidFrontLeftTurn.calculate(
                frontLeftSteer.getSelectedSensorPosition() * kTurningEncoderTicksToRad,
                frontLeftOptimized.angle.getRadians());
        double frontRightTurnPower = pidFrontRightTurn.calculate(
                frontRightSteer.getSelectedSensorPosition() * kTurningEncoderTicksToRad,
                frontRightOptimized.angle.getRadians());
        double backLeftTurnPower = pidBackLeftTurn.calculate(
                backLeftSteer.getSelectedSensorPosition() * kTurningEncoderTicksToRad,
                backLeftOptimized.angle.getRadians());
        double backRightTurnPower = pidBackRightTurn.calculate(
                backRightSteer.getSelectedSensorPosition() * kTurningEncoderTicksToRad,
                backRightOptimized.angle.getRadians());

        // positive is clockwise (right side up)
        frontLeftSteer.set(frontLeftTurnPower);
        frontRightSteer.set(frontRightTurnPower);
        backLeftSteer.set(backLeftTurnPower);
        backRightSteer.set(backRightTurnPower);

        // set drive power to desired speed div max speed to get value between 0 and 1
        frontLeftDrive.set(frontLeftOptimized.speedMetersPerSecond / maxSpeedMpS);
        frontRightDrive.set(frontRightOptimized.speedMetersPerSecond / maxSpeedMpS);
        backLeftDrive.set(backLeftOptimized.speedMetersPerSecond / maxSpeedMpS);
        backRightDrive.set(backRightOptimized.speedMetersPerSecond / maxSpeedMpS);
    }

    public void limitationArm(double getCurrent_ArmAngleRad) {
        if (getCurrent_ArmAngleRad <= maxArmAngleRad) {
            armRotate.tankDrive(-speed_armRotation, speed_armRotation); //go down
            //armLift_LowerAuto(-1.5, 0);
            armAngleLimited = true; // set flag to indicate arm angle is being limited
        }
        if (getCurrent_ArmAngleRad >= minArmAngleRad) {
            armRotate.tankDrive(speed_armRotation, -speed_armRotation); //go up
            //armLift_LowerAuto(, 0);
            armAngleLimited = true; // set flag to indicate arm angle is being limited
        }
        // disable arm angle limit flag if arm angle is within limits vvv
        if (getCurrent_ArmAngleRad > maxArmAngleRad && getCurrent_ArmAngleRad < minArmAngleRad) {
            armAngleLimited = false; // reset flag
        }
    }

    public void robotArm(double armDown, double armUp, Boolean claw_xBox, Boolean extendArm, Boolean retractArm,
            boolean claw_expel, boolean claw_intake, boolean clawIntake_and_Extend) {

        // check if arm angle is being limited
        if (armAngleLimited) {
            return; // exit function if arm angle is being limited
        }

        // arm angle roation vvv
        if (armDown >= 0.5) {
            armRotate.tankDrive(speed_armRotation, -speed_armRotation);
        } else if (armUp >= 0.5) {
            armRotate.tankDrive(-speed_armRotation, speed_armRotation);
        } else {
            armRotate.tankDrive(0, 0);
        }
 
        // arm extendo vvv
        if (extendArm == true) {
            armTalonExtenstion.set(armTalonExtenstionSpeed);
        } else if (retractArm == true) {
            armTalonExtenstion.set(-armTalonExtenstionSpeed);
        } else {
            armTalonExtenstion.set(0);
        }

        // b Button aka CLAW vvv
        if (claw_xBox == true) {
            dSolenoidClaw_ButtonPressed = !dSolenoidClaw_ButtonPressed;
        }
        if (dSolenoidClaw_ButtonPressed == true) {
            dSolenoidClaw.set(Value.kForward);
        } else if (dSolenoidClaw_ButtonPressed == false) {
            dSolenoidClaw.set(Value.kReverse);
        }

        // wheels claw vvv
        if (claw_intake == true) {
            claw_Wheels.set(ClawIntake_WheelSpeed);
        } else if (claw_expel == true) {
            claw_Wheels.set(ClawExpel_WheelSpeed);
        } else {
            claw_Wheels.set(0);
        } 

        if (clawIntake_and_Extend == true){
            claw_Wheels.set(ClawIntake_WheelSpeed);
            armTalonExtenstion.set(armTalonExtenstionSpeed);
        }
    }

    public void drive_PID(double targetXdistance_Metres, double targetYdistance_Metres) {
        if (isFirstTime == true) {
            frontLeftDrive.setSelectedSensorPosition(0);
            isFirstTime = false;
        } 
                        
        double currentDistanceX;
        currentDistanceX = encoderLeftFrontDriveDisplacement_Meteres;
        double outputXSpeed;

        double currentDistanceY;
        currentDistanceY = encoderLeftFrontDriveDisplacement_Meteres;
        double outputYSpeed;

        // double currentSteer_Rad;
        // currentSteer_Rad = angleRad;
        // double outputYaw_RadPerSec;

        // if (Math.abs(targetXdistance_Metres - currentDistanceX)) {
            outputXSpeed = drive.calculate(currentDistanceX, targetXdistance_Metres) ;
            // swerveDrive(outputXSpeed, 0, 0);
        //}
        //if (Math.abs(targetYdistance_Metres - currentDistanceY) > tolerance) {
            outputYSpeed = drive.calculate(currentDistanceY, targetYdistance_Metres);
            // swerveDrive(0, outputYSpeed, 0);
        //}
        //if (Math.abs(target_RadDis - currentSteer_Rad) > tolerance) {
            //outputYaw_RadPerSec  = drive.calculate(currentSteer_Rad, target_RadDis);
            // swerveDrive(0, 0, outputYaw_Rad);
        //}
        //contXSpeedField = outputXSpeed * Math.cos(angleRad) - outputYSpeed * Math.sin(angleRad);
        //contYSpeedField = outputXSpeed * Math.sin(angleRad) + outputYSpeed * Math.cos(angleRad);
        //swerveDrive(outputXSpeed, 0, 0);
        autonomousScuffed.arcadeDrive(-outputXSpeed, -outputYSpeed);
    }

    public void armLift_LowerAuto(double targetDistanceRads, double tolerance, boolean down) {
        double output;
        double currentDistance = armRad_current;
        if (Math.abs(targetDistanceRads - currentDistance) > tolerance) {
            output = PID_armAngle.calculate(currentDistance, targetDistanceRads);
            armRotate.tankDrive(output, -output);
            /* 
            if (down == false){
                armRotate.tankDrive(-output, output);
            } else if (down == true ){
                armRotate.tankDrive(output, -output);
            }
            */
        }
    }

    public void armExtender_Auto(double targetDistanceMetres, double tolerance) {
        double output;
        double currentDistance = extenstionEncoder_CurrentMetres;
        if (Math.abs(targetDistanceMetres - currentDistance) > tolerance) {
            output = pidArmExtensController.calculate(currentDistance, targetDistanceMetres);
            armTalonExtenstion.set(output);
        }
    }

    public void autoBalance() {
        double outputPitch = 0;
        double outputYaw = 0;
        double outputYawRad = 0;
        double currentPitch;
        double currentYaw;
        double targetAnglePitch = 0;
        double targetAngleYaw = 0;
        double tolerance = 5;
        currentPitch = navxPitch_Deg;
        currentYaw = navxYaw_Deg;
        double maxDriveDistance = 1.22; // maximum allowed drive distance before robot is considered to be off the platform
        double currentDriveDistanceX = encoderLeftFrontDriveDisplacement_Meteres;

        if (Math.abs(targetAnglePitch - currentPitch) > tolerance) {
            outputPitch = pidPitch.calculate(currentPitch, targetAnglePitch);
        }
        if (Math.abs(targetAngleYaw - currentYaw) > tolerance) {
            outputYaw = pidYaw.calculate(currentYaw, targetAngleYaw);
            outputYawRad = Math.toRadians(outputYaw);
        }
        if (Math.abs(currentDriveDistanceX) > maxDriveDistance) {
            swerveDrive(0, 0, 0); // stop robot from moving if it has driven too far
        } else {
            swerveDrive(outputPitch, 0, outputYawRad);
        }
    }

    public void absolutePosition() {
        // return radians
        frontLeftAbsAngle = (frontLeftAbsEncoder.getAbsolutePosition() - frontLeftAbsOffset) * (Math.PI / 180);
        frontRightAbsAngle = (frontRightAbsEncoder.getAbsolutePosition() - frontRightAbsOffset) * (Math.PI / 180);
        backLeftAbsAngle = (backLeftAbsEncoder.getAbsolutePosition() - backLeftAbsOffset) * (Math.PI / 180);
        backRightAbsAngle = (backRightAbsEncoder.getAbsolutePosition() - backRightAbsOffset) * (Math.PI / 180);
    }

    public void straightenModules() {
        if (Math.abs(frontLeftAbsAngle) > 0.0 ||
                Math.abs(frontRightAbsAngle) > 0.0 ||
                Math.abs(backLeftAbsAngle) > 0.0 ||
                Math.abs(backRightAbsAngle) > 0.0) {

            double frontLeftTurnPower = pidFrontLeftTurn.calculate(frontLeftAbsAngle, 0);
            double frontRightTurnPower = pidFrontRightTurn.calculate(frontRightAbsAngle, 0);
            double backLeftTurnPower = pidBackLeftTurn.calculate(backLeftAbsAngle, 0);
            double backRightTurnPower = pidBackRightTurn.calculate(backRightAbsAngle, 0);

            frontLeftSteer.set(frontLeftTurnPower);
            frontRightSteer.set(frontRightTurnPower);
            backLeftSteer.set(backLeftTurnPower);
            backRightSteer.set(backRightTurnPower);

            SmartDashboard.putNumber("frontLeftTurnPower", frontLeftTurnPower);
            SmartDashboard.putNumber("frontRightTurnPower", frontRightTurnPower);
            SmartDashboard.putNumber("backLeftTurnPower", backLeftTurnPower);
            SmartDashboard.putNumber("backRightTurnPower", backRightTurnPower);
        }
    }

    // execution Functions vvvvv
    @Override
    public void robotInit() {
        UsbCamera camera = CameraServer.startAutomaticCapture();
        setMotorBreaks();
        invertMotors();
        continouousInput();
        navx.calibrate();
        navx.reset();
        resetEncoders();
    }

    @Override
    public void robotPeriodic() {
        /*if (timer.get() <= 2) {
            absolutePosition();
            straightenModules();
            resetEncoders();
        }*/

        // claw vvv
        // claw_Wheels.set(constant_claw_WheelSpeed);

        // limit arm vvv
        armRad_current = rightArmSide.getSelectedSensorPosition()*armRotate_ToRad;
        SmartDashboard.putNumber("encoderRad_rightArmSide", armRad_current);
        limitationArm(armRad_current);
        SmartDashboard.putBoolean("armAngleLimited: ", armAngleLimited);

        // armExtenstion
        extenstionEncoder_CurrentMetres = armTalonExtenstion.getSelectedSensorPosition() * armExtenstion_ToMetres;
        SmartDashboard.putNumber("Arm_Distance_metres", extenstionEncoder_CurrentMetres);

        // encoder drive variables vvv
        encoderLeftFrontDriveDisplacement_Meteres = frontLeftDrive.getSelectedSensorPosition()
                * kTurningEncoderTicksToMetresPerSec;
        SmartDashboard.putNumber("Drive_Distance_Metres: ", encoderLeftFrontDriveDisplacement_Meteres);

        encoderleftFrontSteer_Rad = frontLeftSteer.getSelectedSensorPosition()*kTurningEncoderTicksToRad;
        SmartDashboard.putNumber("Drive_Rotation_Radians: ", encoderleftFrontSteer_Rad);

        // navX2 vvv
        navxYaw_Deg = navx.getYaw();
        SmartDashboard.putNumber("Yaw_Deg", navxYaw_Deg);
        angleRad = Math.toRadians(navx.getAngle());
        SmartDashboard.putNumber("getGyroAngle", navx.getAngle());
        navxPitch_Deg = navx.getPitch();
        SmartDashboard.putNumber("Pitch_deg", navxPitch_Deg);
        navxRoll_Deg = navx.getRoll();
        SmartDashboard.putNumber("Roll_Deg", navxRoll_Deg);
        SmartDashboard.putData(navx);

        // encoders vvv
        SmartDashboard.putNumber("frontLeftAbs Offset", frontLeftAbsEncoder.getAbsolutePosition());
        SmartDashboard.putNumber("frontRightAbs Offset", frontRightAbsEncoder.getAbsolutePosition());
        SmartDashboard.putNumber("backLeftAbs Offset", backLeftAbsEncoder.getAbsolutePosition());
        SmartDashboard.putNumber("backRightAbs Offset", backRightAbsEncoder.getAbsolutePosition());

        //pid charts vvv
        //auto vvv
        SmartDashboard.putNumber("drive PID (atm used for x, y and turn of autonomous)", drive.getPositionError());

        // aline vvv
        SmartDashboard.putNumber("pid frontLeft Error", pidFrontLeftTurn.getPositionError());
    }

    @Override
    public void autonomousInit() {
        drive.setIntegratorRange(-0.1, 0.1);
        drive.setTolerance(0.1);

        autonomousStartTime = Timer.getFPGATimestamp();
        // double elapsedTime = Timer.getFPGATimestamp() - autonomousStartTime;
        //  if (elapsedTime <= 5){
        //     targetDistance_Xauto = -2;
        //      drive_PID(targetDistance_Xauto, 0, 0, 0.5); //fwd 2 metres
        // } else if (elapsedTime <= 10){
        //     targetDistance_Yauto = -2;
        //     drive_PID(0, targetDistance_Yauto, 0, 0.5); //right 2 metres
        // } else if (elapsedTime <= 15){
        //     targetRad_auto = 3.14159;
        //     drive_PID(0, 0, targetRad_auto, 0.5); //turn 3 rads per second
        // } else {
            //autoBalance();
        // }
    }
 
    @Override
    public void autonomousPeriodic() {
        // double elapsedTime = Timer.getFPGATimestamp() - autonomousStartTime;
        //  if (elapsedTime <= 5){
            targetDistance_Xauto = 2;
             drive_PID(targetDistance_Xauto, 0); //fwd 2 metres
             // } else if (elapsedTime <= 10){
        //     targetDistance_Yauto = -2;
        //     drive_PID(0, targetDistance_Yauto, 0, 0.5); //right 2 metres
        // } else if (elapsedTime <= 15){
        //     targetRad_auto = 3.14159;
        //     drive_PID(0, 0, targetRad_auto, 0.5); //turn 3 rads per second
        // } else {
            //autoBalance();
        // }
        
    
    SmartDashboard.putBoolean("at setpoint",drive.atSetpoint());
}

    // Method to maintain the current target distance for a specified duration
    // public void maintainDistance(double duration) {
    //     maintainDistance = true;
    //     maintainDuration = duration;
    // }

    @Override
    public void teleopInit() {
        navx.reset();
    }

    @Override
    public void teleopPeriodic() {

        // swerve vvv (uses driving_xBoxCont)
        double contXSpeed = removeDeadzone(1) * XdriveSensitivity;
        double contYSpeed = removeDeadzone(0) * YdriveSensitivity;
        double contTurnSpeed = removeDeadzone(4) * turningSensitivity;
         
         //turn field oriented on/off
        if(true){
             contXSpeedField = contXSpeed * Math.cos(angleRad) - contYSpeed * Math.sin(angleRad);
             contYSpeedField = contXSpeed * Math.sin(angleRad) + contYSpeed * Math.cos(angleRad);
            }
            
        swerveDrive(contXSpeedField, contYSpeedField, contTurnSpeed);
        //swerveDrive(contXSpeed, contYSpeed, contTurnSpeed);

        //robot arm
        double armDown = arm_xBoxCont.getRightTriggerAxis();
        double armUp = arm_xBoxCont.getLeftTriggerAxis();
        boolean claw_xBox = arm_xBoxCont.getBButtonPressed();
        boolean extendArm = arm_xBoxCont.getLeftBumper();
        boolean retractArm = arm_xBoxCont.getRightBumper();
        boolean expel = arm_xBoxCont.getAButton();
        boolean intake = arm_xBoxCont.getXButton();
        boolean clawIntake_and_Extend = arm_xBoxCont.getYButton();
        robotArm(armDown, armUp, claw_xBox, extendArm, retractArm, expel, intake, clawIntake_and_Extend);
    }
    //asdf
}