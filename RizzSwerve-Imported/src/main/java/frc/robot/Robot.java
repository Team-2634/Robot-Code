package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.SPI;

public class Robot extends TimedRobot {
    // Constants vvvvv
    DigitalInput extendLimitSwitch = new DigitalInput(9);
    Timer timerRobot = new Timer();
    Timer timerAuto = new Timer();
    double autonomousStartTime;
    double targetDistance_Xauto = 0;
    double targetDistance_Yauto = 0;
    double targetRad_auto = 0;
    final XboxController driving_xBoxCont = new XboxController(0);
    final XboxController arm_xBoxCont = new XboxController(1);
    double maxDegree = 360; // if your over 360 then your driving to much
    double talonEncoder_TicksPerRev = 2048;
    double neoEncoder_TicksPerRev = 42;

    double contXSpeedField;
    double contYSpeedField;

    // these are used for swerve vvv
    final double kpDrive = 0.3;
    final double kiDrive = 0.01;
    final double kdDrive = 0.01;

    final double kpAuto = 1;
    final double kiAuto = 0;
    final double kdAuto = 0.0000075;

    PIDController pidFrontLeftTurn = new PIDController(kpDrive, kiDrive, kdDrive);
    PIDController pidFrontRightTurn = new PIDController(kpDrive, kiDrive, kdDrive);
    PIDController pidBackLeftTurn = new PIDController(kpDrive, kiDrive, kdDrive);
    PIDController pidBackRightTurn = new PIDController(kpDrive, kiDrive, kdDrive);

    PIDController pidFrontLeftTurnAuto = new PIDController(kpAuto, kiAuto, kdAuto);
    PIDController pidFrontRightTurnAuto = new PIDController(kpAuto, kiAuto, kdAuto);
    PIDController pidBackLeftTurnAuto = new PIDController(kpAuto, kiAuto, kdAuto);
    PIDController pidBackRightTurnAuto = new PIDController(kpAuto, kiAuto, kdAuto);

    
    public final TalonFX frontLeftDrive = new TalonFX(7);
    public final TalonFX frontRightDrive = new TalonFX(1);
    public final TalonFX backLeftDrive = new TalonFX(5);
    public final TalonFX backRightDrive = new TalonFX(3);

    /*
     * public final CANSparkMax frontLeftDrive = new CANSparkMax(17,
     * MotorType.kBrushless);
     * public final CANSparkMax frontRightDrive = new CANSparkMax(10,
     * MotorType.kBrushless);
     * public final CANSparkMax backLeftDrive = new CANSparkMax(4,
     * MotorType.kBrushless);
     * public final CANSparkMax backRightDrive = new CANSparkMax(18,
     * MotorType.kBrushless);
     */

    public final TalonFX frontLeftSteer = new TalonFX(6);
    public final TalonFX frontRightSteer = new TalonFX(0);
    public final TalonFX backLeftSteer = new TalonFX(4);
    public final TalonFX backRightSteer = new TalonFX(2);

    /*
     * public final CANSparkMax frontLeftSteer = new CANSparkMax(17,
     * MotorType.kBrushless);
     * public final CANSparkMax frontRightSteer = new CANSparkMax(10,
     * MotorType.kBrushless);
     * public final CANSparkMax backLeftSteer = new CANSparkMax(4,
     * MotorType.kBrushless);
     * public final CANSparkMax backRightSteer = new CANSparkMax(18,
     * MotorType.kBrushless);
     */
    
    public final CANcoder frontLeftAbsEncoder = new CANcoder(3);
    public final CANcoder frontRightAbsEncoder = new CANcoder(0);
    public final CANcoder backLeftAbsEncoder = new CANcoder(2);
    public final CANcoder backRightAbsEncoder = new CANcoder(1);

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
    double encoderRightFrontDriveDisplacement_Meteres;
    double encoderleftFrontSteer_Rad;
    double XdriveSensitivity = 1;
    double YdriveSensitivity = 1; // do not change above 1
    double turningSensitivity = 25; // radians
    double maxSpeedMpS = 20; // metres/sec

    Translation2d m_frontLeftLocation = new Translation2d(0.340, 0.285);
    Translation2d m_frontRightLocation = new Translation2d(0.340, -0.285);
    Translation2d m_backLeftLocation = new Translation2d(-0.340, 0.285);
    Translation2d m_backRightLocation = new Translation2d(-0.340, -0.285);

    SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
            m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

    // these are for the arm lift vvv
    public final WPI_TalonFX leftArmSide = new WPI_TalonFX(9);
    public final WPI_TalonFX rightArmSide = new WPI_TalonFX(8);
    private final DifferentialDrive armRotate = new DifferentialDrive(leftArmSide, rightArmSide);
    double liftArmSide_GearRatio = 64 * (60 / 15);
    double armRotate_ToRad = ((1.0 / liftArmSide_GearRatio) * 2 * Math.PI) / 2048;
    double armRad_current;
    double kp_armAngle = 0.5, ki_armAngle = 0.05, kd_armAngle = 0.05;
    final PIDController PID_armAngle = new PIDController(kp_armAngle, ki_armAngle, kd_armAngle);
    double maxArmAngleRad = -1.95; // -1.95
    double minArmAngleRad = 0.015;
    double speed_armRotation = 0.75;
    // flag to indicate if arm angle is being limited
    private boolean armAngleLimited = false;

    // arm extend vvv
    final WPI_TalonFX armTalonExtenstion = new WPI_TalonFX(10);
    double maxArmExtend_Metres = 0.79;
    double minArmExtend_Metres = 0.005;
    private boolean armExtendLimited = false;
    double armExtenstion_gearRatio = 1 / 36.0;
    double armTalonExtenstionSpeed_Out = 0.95; // 0.83
    double armTalonExtenstionSpeed_In = 0.95; // 0.83.
    double armTalonExtenstionSpeed_autoExtend = 0.20;
    double armTalonExtenstionSpeed_autoRetreat = 0.10;
    double armExtenstion_ToMetres = (armExtenstion_gearRatio * Math.PI * Units.inchesToMeters(2.75)) / 2048.0; // metres
    double extenstionEncoder_CurrentMetres;
    double kp_armE = 0.5, ki_armE = 0, kd_armD = 0;
    final PIDController pidArmExtensController = new PIDController(kpAuto, kiAuto, kdAuto);

    // claw_Wheels vvv
    private final CANSparkMax claw_Wheels = new CANSparkMax(13, MotorType.kBrushless);
    double ClawIntake_WheelSpeed = -1;
    double ClawIntake_WheelSpeed_SLOW = -0.10;
    double ClawExpel_WheelSpeed = 0.40;

    // pnuematics vvv
    private final DoubleSolenoid dSolenoidClaw = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 5, 4);
    boolean dSolenoidClaw_ButtonPressed = false;

    // navx2 vvv
    final double kp_Pitch = 0.01;
    //final double kp_Yaw = 0.1;
    final double ki_Navx = 0.0;
    final double kd_Navx = 0.0;
    AHRS navx = new AHRS(SPI.Port.kMXP);
    PIDController pidPitch = new PIDController(kp_Pitch, ki_Navx, kd_Navx);
    //PIDController pidYaw = new PIDController(kp_Yaw, ki_Navx, kd_Navx);
    double navxYaw_Deg;
    double navxPitch_Deg;
    double navxRoll_Deg;
    double botYaw_angleRad;

    
    PIDController drive = new PIDController(0.5, 0.0, 0.05);
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

    public void resetPIDs(){
        pidFrontLeftTurn.reset();
        pidFrontRightTurn.reset();
        pidBackLeftTurn.reset();
        pidBackRightTurn.reset();
        
        pidFrontLeftTurnAuto.reset();
        pidFrontRightTurnAuto.reset();
        pidBackLeftTurnAuto.reset();
        pidBackRightTurnAuto.reset();
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
        if (Math.abs(driving_xBoxCont.getRawAxis(axisInput)) < 0.15) {
            return 0;
        }
        return driving_xBoxCont.getRawAxis(axisInput);
    }

    // public double removeDeadzone(int axisInput, int axisInput2) {
    //     if (Math.abs(driving_xBoxCont.getRawAxis(axisInput)) < 0.1
    //             && Math.abs(driving_xBoxCont.getRawAxis(axisInput2)) < 0.1) {
    //         return 0;
    //     }
    //     return driving_xBoxCont.getRawAxis(axisInput);
    // }

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

    public void limitationArmRise(double getCurrent_ArmAngleRad) {
        if (getCurrent_ArmAngleRad <= maxArmAngleRad) {
            armRotate.tankDrive(-speed_armRotation, speed_armRotation); // go down
            // armLift_LowerAuto(-1.5, 0);
            armAngleLimited = true; // set flag to indicate arm angle is being limited
        }
        if (getCurrent_ArmAngleRad >= minArmAngleRad) {
            armRotate.tankDrive(speed_armRotation, -speed_armRotation); // go up
            // armLift_LowerAuto(, 0);
            armAngleLimited = true; // set flag to indicate arm angle is being limited
        }

        // disable arm angle limit flag if arm angle is within limits vvv
        if (getCurrent_ArmAngleRad > maxArmAngleRad && getCurrent_ArmAngleRad < minArmAngleRad) {
            armAngleLimited = false; // reset flag
        }
    }

    public void limitationArmExtend(double getCurrent_ArmExtendMetres) {
        /*
         * if (getCurrent_ArmExtendMetres < minArmExtend_Metres) {
         * armTalonExtenstion.set(armTalonExtenstionSpeed_autoExtend);
         * armExtendLimited = true; // set flag to indicate arm angle is being limited
         * }
         */
        if (getCurrent_ArmExtendMetres > maxArmExtend_Metres) {
            armTalonExtenstion.set(-armTalonExtenstionSpeed_autoRetreat);
            armExtendLimited = true;
        }

        if (getCurrent_ArmExtendMetres < maxArmExtend_Metres) {
            armExtendLimited = false; // reset flag
        }
    }

    public void robotArm(double armDown, double armUp, Boolean claw_xBox, Boolean extendArm, Boolean retractArm,
            boolean claw_expel, boolean claw_intake, boolean clawIntake_and_Extend, boolean slowClawWheels) {

        // check if arm angle is being limited
        if (armAngleLimited) {
            return; // exit function if arm angle is being limited
        }

        if (armExtendLimited) {
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
            armTalonExtenstion.set(armTalonExtenstionSpeed_Out);
        } else if (retractArm == true) {
            if (!extendLimitSwitch.get()) {
                armTalonExtenstion.set(-armTalonExtenstionSpeed_In);
            } else if (extendLimitSwitch.get()) {
                armTalonExtenstion.set(0);
            }
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

        if (clawIntake_and_Extend == true) {
            claw_Wheels.set(ClawIntake_WheelSpeed);
            armTalonExtenstion.set(armTalonExtenstionSpeed_Out);
        }

        if (slowClawWheels == true) {
            claw_Wheels.set(ClawIntake_WheelSpeed_SLOW);
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

            double frontLeftTurnPower = pidFrontLeftTurnAuto.calculate(frontLeftAbsAngle, 0);
            double frontRightTurnPower = pidFrontRightTurnAuto.calculate(frontRightAbsAngle, 0);
            double backLeftTurnPower = pidBackLeftTurnAuto.calculate(backLeftAbsAngle, 0);
            double backRightTurnPower = pidBackRightTurnAuto.calculate(backRightAbsAngle, 0);

            frontLeftSteer.set(frontLeftTurnPower);
            frontRightSteer.set(frontRightTurnPower);
            backLeftSteer.set(backLeftTurnPower);
            backRightSteer.set(backRightTurnPower);
        }
    }

    // execution Functions vvvvv
    @Override
    public void robotInit() {
        timerRobot.reset();
        timerRobot.start();
        UsbCamera camera0 = CameraServer.startAutomaticCapture(0);
        UsbCamera camera1 = CameraServer.startAutomaticCapture(1);
        setMotorBreaks();
        invertMotors();
        continouousInput();
        navx.calibrate();
        navx.reset();
        resetEncoders();
        resetPIDs();
        frontRightDrive.setSelectedSensorPosition(0);
        frontLeftDrive.setSelectedSensorPosition(0);
    }

    double navxYaw_Rad;
    double straightenTolerance = 1;

    @Override
    public void robotPeriodic() {

        // }

        // claw vvv
        // claw_Wheels.set(constant_claw_WheelSpeed);

        // limit arm vvv
        armRad_current = rightArmSide.getSelectedSensorPosition() * armRotate_ToRad;
        SmartDashboard.putNumber("encoderRad_rightArmSide", armRad_current);

        // armExtenstion
        extenstionEncoder_CurrentMetres = armTalonExtenstion.getSelectedSensorPosition() * armExtenstion_ToMetres;
        SmartDashboard.putNumber("Arm_Distance_metres", extenstionEncoder_CurrentMetres);

        // encoder drive variables vvv
        encoderLeftFrontDriveDisplacement_Meteres = frontLeftDrive.getSelectedSensorPosition()
                * kTurningEncoderTicksToMetresPerSec;
        SmartDashboard.putNumber("Drive_Distance_Metres x axis: ", encoderLeftFrontDriveDisplacement_Meteres);

        encoderRightFrontDriveDisplacement_Meteres = frontRightDrive.getSelectedSensorPosition()
                * kTurningEncoderTicksToMetresPerSec;
        SmartDashboard.putNumber("Drive_Distance_Metres yaxis: ", encoderRightFrontDriveDisplacement_Meteres);

        encoderleftFrontSteer_Rad = frontLeftSteer.getSelectedSensorPosition() * kTurningEncoderTicksToRad;
        SmartDashboard.putNumber("Drive_Rotation_Radians: ", encoderleftFrontSteer_Rad);

        // navX2 vvv
        navxYaw_Deg = navx.getYaw();
        SmartDashboard.putNumber("Yaw_Deg", navxYaw_Deg);
        navxYaw_Rad = Units.degreesToRadians(navxYaw_Deg);
        SmartDashboard.putNumber("navxYaw_Rad ", navxYaw_Rad);
        botYaw_angleRad = Math.toRadians(navx.getAngle());
        SmartDashboard.putNumber("angleRad: ", botYaw_angleRad);
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

        // pid charts vvv
        // auto vvv
        SmartDashboard.putNumber("drive PID (atm used for x, y and turn of autonomous)", drive.getPositionError());
        SmartDashboard.putNumber("PID_armAngle", PID_armAngle.getPositionError());

        // aline vvv
        SmartDashboard.putNumber("pid_frontLeft_Error", pidFrontLeftTurn.getPositionError());
        SmartDashboard.putNumber("pid_frontRight_Error", pidFrontRightTurn.getPositionError());
        SmartDashboard.putNumber("pid_backLeft_Error", pidBackLeftTurn.getPositionError());
        SmartDashboard.putNumber("pid_backRight_Error", pidBackRightTurn.getPositionError());
    }

    public boolean driveSwerve_EncoderIf_FwdAndBwd(double targetX) {
        targetX = -targetX;

        double currentDistanceX;
        currentDistanceX = encoderLeftFrontDriveDisplacement_Meteres;
        double outPutX = 0;

        double toleranc = 0.05;
        double xSpeed = 0.45;
        double xSpeed_Rev = -0.30;
        if (Math.abs(targetX - currentDistanceX) > toleranc) {
            if (currentDistanceX < targetX) {
                outPutX = xSpeed;
                swerveDrive(outPutX, 0, 0);
                System.out.println("behind target");
                return false;
            }
            if (currentDistanceX > targetX) {
                outPutX = -xSpeed;
                swerveDrive(outPutX, 0, 0);
                System.out.println("in front of target");
                return false;
            }
            return false;
        } else {
            swerveDrive(0, 0, 0);
            return true;
        }
    }

    public boolean driveSwerve_EncoderIf_turnOnSpot(double targetYaw_inRad) {
        double currentRoationYaw_inRad;
        currentRoationYaw_inRad = botYaw_angleRad;
        double outPutRad = 0;

        double tolerance = 0.2;
        double RotSpeed = 25; // rads per sec
        if (Math.abs(targetYaw_inRad - currentRoationYaw_inRad) > tolerance) {
            if (currentRoationYaw_inRad < targetYaw_inRad) {
                outPutRad = RotSpeed;
                swerveDrive(0, 0, outPutRad);
                return false;
            } else if (currentRoationYaw_inRad > targetYaw_inRad) {
                outPutRad = -RotSpeed;
                swerveDrive(0, 0, outPutRad);
                return false;
            }
            return false;
        } else {
            swerveDrive(0, 0, 0);
            return true;
        }
    }

    public boolean armRotate_encoderIf_upAndDown(double targetY) {

        double currentDistanceY;
        currentDistanceY = armRad_current;
        double outPutY = 0;

        double toleranc = 0.1;
        double ySpeed = 0.70;
        double ySpeed_Rev = -0.15;
        if (Math.abs(targetY - currentDistanceY) > toleranc) {
            if (currentDistanceY > targetY) {
                outPutY = -ySpeed;
                armRotate.tankDrive(-outPutY, outPutY);
                return false;
            }
            if (currentDistanceY < targetY) {
                outPutY = ySpeed;
                armRotate.tankDrive(-outPutY, outPutY);
                return false;
            }return false;
        } else {
            armRotate.tankDrive(0, 0);
            return true;
        }
    }

    public boolean armExtend_encoderIf_outAndIn(double targetExtend) {

        double currentDistance_Metres;
        currentDistance_Metres = extenstionEncoder_CurrentMetres;
        double outPut_prec = 0;

        double toleranc = 0.05;
        double xSpeed = 0.90;
        double xSpeed_Rev = -0.15;
        if (Math.abs(targetExtend - currentDistance_Metres) > toleranc) {
            if (currentDistance_Metres < targetExtend) {
                outPut_prec = xSpeed;
                armTalonExtenstion.set(outPut_prec);
                return false;
            }
            if (currentDistance_Metres > targetExtend) {
                if (!extendLimitSwitch.get()) {
                    outPut_prec = -xSpeed;
                    armTalonExtenstion.set(outPut_prec);
                    return false;
                } else if (extendLimitSwitch.get()) {
                    armTalonExtenstion.set(0);
                    return false;
                }
            }
        }
        else {
            armTalonExtenstion.set(0);
            return true;
        }
        return false;
    }

    @Override
    public void autonomousInit() {
        timerAuto.reset();
        timerAuto.start();
        dSolenoidClaw.set(Value.kReverse);

        Collections.fill(commandFlags, Boolean.FALSE);
    }

    List<Boolean> commandFlags = new ArrayList<Boolean>(Arrays.asList(new Boolean[10]));

    @Override
    public void autonomousPeriodic() {
        // autoTopCone(); //balance
        autoTopAndBottom(); //not balance
    } 

    public void autoBalance() {
        double outputPitch;
        double currentPitch;
        double targetAnglePitch = 0;
        double tolerance = 5;
        currentPitch = navxPitch_Deg;

        if (Math.abs(targetAnglePitch - currentPitch) > tolerance) {
            outputPitch = pidPitch.calculate(currentPitch, targetAnglePitch);
        } else {
            outputPitch = 0;
        }
        
        swerveDrive(outputPitch, 0, 0);
    }

    public void autoMidBalance() {
        if (timerInterval_Auto(0, 2.5)){
            driveSwerve_EncoderIf_turnOnSpot(Math.PI-0.01); // face nodes
            System.out.println("turning");
            frontLeftDrive.setSelectedSensorPosition(0);
        }else if (timerInterval_Auto(2.51, 3)){
            driveSwerve_EncoderIf_FwdAndBwd(0.21); //drive forwards to thingy
            //swerveDrive(-0.2, 0, 0);
            System.out.println("forward");
        }else if (timerInterval_Auto(3.01, 6)){
            swerveDrive(0,0,0);
            armRotate_encoderIf_upAndDown(-1.80); //lift arm
            System.out.println("arm up");
        }else if (timerInterval_Auto(6.01, 8)){
            armExtend_encoderIf_outAndIn(0.75); // extend arm
            System.out.println("extend");
        }else if (timerInterval_Auto(8.01, 8.5)){
            dSolenoidClaw.set(Value.kForward); //open claw
            System.out.println("drop");
        }else if (timerInterval_Auto(8.51, 10)){
            armExtend_encoderIf_outAndIn(0); //retract
            dSolenoidClaw.set(Value.kReverse); //close claw
            System.out.println("arm down");
        }else if (timerInterval_Auto(10.01, 12)){
            armRotate_encoderIf_upAndDown(-0.1); //lower arm
            System.out.println("retract");
        }else if (timerInterval_Auto(12.01, 15)){
            driveSwerve_EncoderIf_FwdAndBwd(-2.3); //drive backwards past line
            System.out.println("back it up");
        }else { //STOP!!!
            swerveDrive(0, 0, 0);
            armRotate.tankDrive(0, 0);      
            armTalonExtenstion.set(0);
            System.out.println("stop");
        }
    }

    public void autoTopAndBottom() {
        if (timerInterval_Auto(0, 2)){
            driveSwerve_EncoderIf_turnOnSpot(Math.PI-0.01); // face nodes
            System.out.println("turning");
        }else if (timerInterval_Auto(2.01, 3.50)){
            driveSwerve_EncoderIf_FwdAndBwd(0.21); //drive forwards to thingy
            frontLeftDrive.setSelectedSensorPosition(0);
            //swerveDrive(-0.3, 0, 0);
            System.out.println("forward");
        }else if (timerInterval_Auto(3.51, 6)){
            swerveDrive(0,0,0);
            armRotate_encoderIf_upAndDown(-1.80); //lift arm
            System.out.println("arm up");
        }else if (timerInterval_Auto(6.01, 8)){
            armExtend_encoderIf_outAndIn(0.75); // extend arm
            System.out.println("extend");
        }else if (timerInterval_Auto(8.01, 8.5)){
            dSolenoidClaw.set(Value.kForward); //open claw
            System.out.println("drop");
        }else if (timerInterval_Auto(8.51, 10)){
            armExtend_encoderIf_outAndIn(0); //retract
            dSolenoidClaw.set(Value.kReverse); //close claw
            System.out.println("arm down");
        }else if (timerInterval_Auto(10.01, 12)){
            armRotate_encoderIf_upAndDown(-0.1); //lower arm
            System.out.println("retract");
        }else if (timerInterval_Auto(12.01, 15)){
            driveSwerve_EncoderIf_FwdAndBwd(-4.3); //drive backwards past line
            System.out.println("back it up");
        }else { //STOP!!!
            swerveDrive(0, 0, 0);
            armRotate.tankDrive(0, 0);      
            armTalonExtenstion.set(0);
            System.out.println("stop");
        }
    }

    @Override
    public void autonomousExit() {
    }

    @Override
    public void teleopInit() {
    }

    @Override
    public void teleopPeriodic() {

        limitationArmRise(armRad_current);
        SmartDashboard.putBoolean("armAngleLimited: ", armAngleLimited);

        limitationArmExtend(extenstionEncoder_CurrentMetres);
        // SmartDashboard.putBoolean("armExtendLimited: ", armExtendLimited);
        // setMotorSpeed_limitinExtendWHenToFar(-armTalonExtenstionSpeed_autoRetreat);

        SmartDashboard.putBoolean("retractLimitSwitch: ", extendLimitSwitch.get());
        System.out.println(extendLimitSwitch.get());

        // swerve vvv (uses driving_xBoxCont)
        double contXSpeed = removeDeadzone(1) * XdriveSensitivity;
        double contYSpeed = removeDeadzone(0) * YdriveSensitivity;
        double contTurnSpeed = removeDeadzone(4) * turningSensitivity;

        // turn field oriented on/off
        if (true) {
            contXSpeedField = contXSpeed * Math.cos(botYaw_angleRad) - contYSpeed * Math.sin(botYaw_angleRad);
            contYSpeedField = contXSpeed * Math.sin(botYaw_angleRad) + contYSpeed * Math.cos(botYaw_angleRad);
        }

        swerveDrive(contXSpeedField, contYSpeedField, contTurnSpeed);
        // swerveDrive(contXSpeed, contYSpeed, contTurnSpeed);

        // robot arm
        double armDown = arm_xBoxCont.getRightTriggerAxis();
        double armUp = arm_xBoxCont.getLeftTriggerAxis();
        boolean claw_xBox = arm_xBoxCont.getBButtonPressed();
        boolean extendArm = arm_xBoxCont.getLeftBumper();
        boolean retractArm = arm_xBoxCont.getRightBumper();
        boolean expel = arm_xBoxCont.getAButton();
        boolean intake = arm_xBoxCont.getXButton();
        boolean clawIntake_and_Extend = arm_xBoxCont.getYButton();
        boolean slowClawWheels = driving_xBoxCont.getRightBumper();
        robotArm(armDown, armUp, claw_xBox, extendArm, retractArm, expel, intake, clawIntake_and_Extend,
                slowClawWheels);
    }

    public boolean timerInterval_Auto(double min, double max) {
        if (timerAuto.get() > min && timerAuto.get() < max) {
            return true;
        } else {
            return false;
        }
    }
    //asdf
}