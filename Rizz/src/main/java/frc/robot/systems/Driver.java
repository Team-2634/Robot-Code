package frc.robot.systems;

import com.studica.frc.AHRS;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class Driver {

    AHRS navx = new AHRS(AHRS.NavXComType.kMXP_SPI);

    PIDController pidFrontLeftTurn = new PIDController(Constants.kpDrive, Constants.kiDrive, Constants.kdDrive);
    PIDController pidFrontRightTurn = new PIDController(Constants.kpDrive, Constants.kiDrive, Constants.kdDrive);
    PIDController pidBackLeftTurn = new PIDController(Constants.kpDrive, Constants.kiDrive, Constants.kdDrive);
    PIDController pidBackRightTurn = new PIDController(Constants.kpDrive, Constants.kiDrive, Constants.kdDrive);
    PIDController[] pidArray = {pidFrontLeftTurn, pidFrontRightTurn, pidBackLeftTurn, pidBackRightTurn};

    public final TalonFX frontLeftDrive = new TalonFX(Constants.frontLeftDriveID);
    public final TalonFX frontRightDrive = new TalonFX(Constants.frontRightDriveID);
    public final TalonFX backLeftDrive = new TalonFX(Constants.backLeftDriveID);
    public final TalonFX backRightDrive = new TalonFX(Constants.backRightDriveID);
    public final TalonFX[] driveMotorArray = {frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive};


    public final TalonFX frontLeftSteer = new TalonFX(Constants.frontLeftSteerID);
    public final TalonFX frontRightSteer = new TalonFX(Constants.frontRightSteerID);
    public final TalonFX backLeftSteer = new TalonFX(Constants.backLeftSteerID);
    public final TalonFX backRightSteer = new TalonFX(Constants.backRightSteerID);
    public final TalonFX[] steerMotorArray = {frontLeftSteer, frontRightSteer, backLeftSteer, backRightSteer};

    public final CANcoder frontLeftAbsEncoder = new CANcoder(Constants.frontLeftAbsEncoderID);
    public final CANcoder frontRightAbsEncoder = new CANcoder(Constants.frontRightAbsEncoderID);
    public final CANcoder backLeftAbsEncoder = new CANcoder(Constants.backLeftAbsEncoderID);
    public final CANcoder backRightAbsEncoder = new CANcoder(Constants.backRightAbsEncoderID);
    public final CANcoder[] absEncoderArray = {frontLeftAbsEncoder, frontRightAbsEncoder, backLeftAbsEncoder, backRightAbsEncoder};

    private final double frontLeftAbsEncoderOffset = Constants.frontLeftAbsEncoderOffset;
    private final double frontRightEncoderOffset = Constants.frontRightAbsEncoderOffset;
    private final double backLeftAbsEncoderOffset = Constants.backLeftAbsEncoderOffset;
    private final double backRightAbsEncoderOffset = Constants.backRightAbsEncoderOffset;
    private final double[] absEncoderOffsetArray = {frontLeftAbsEncoderOffset, frontRightEncoderOffset, backLeftAbsEncoderOffset, backRightAbsEncoderOffset};
    
    public final CANcoderConfigurator frontLeftEncoderConfig = absEncoderArray[0].getConfigurator();
    public final CANcoderConfigurator frontRightEncoderConfig = absEncoderArray[1].getConfigurator();
    public final CANcoderConfigurator backLeftEncoderConfig = absEncoderArray[2].getConfigurator();
    public final CANcoderConfigurator backRightEncoderConfig = absEncoderArray[3].getConfigurator();
    public final CANcoderConfigurator[] encoderConfigArray = {frontLeftEncoderConfig, frontRightEncoderConfig, backLeftEncoderConfig, backRightEncoderConfig};

    Translation2d m_frontLeftLocation = new Translation2d(0.340, 0.285);
    Translation2d m_frontRightLocation = new Translation2d(0.340, -0.285);
    Translation2d m_backLeftLocation = new Translation2d(-0.340, 0.285);
    Translation2d m_backRightLocation = new Translation2d(-0.340, -0.285);

    final SwerveModulePosition frontLeftModulePosition = new SwerveModulePosition();
    final SwerveModulePosition frontRightModulePosition = new SwerveModulePosition();
    final SwerveModulePosition backLeftModulePosition = new SwerveModulePosition();
    final SwerveModulePosition backRightModulePosition = new SwerveModulePosition();
    final SwerveModulePosition[] modulePositionArray = {
        frontLeftModulePosition, frontRightModulePosition, backLeftModulePosition, backRightModulePosition
    };

    public final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
        m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

    //DO NOT REMOVE ANYTHING WITH POSE2D ESTIMATOR THIS DOES STUFF TO MAKE IT AUTO ALIGN

    SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
        m_kinematics, 
        navx.getRotation2d(), 
        modulePositionArray,
        new Pose2d()
    );

    //rotations counted by motor -> rotations wheel side -> distance travelled (meters) 
    public final double ticksToMetersDrive = Constants.kDriveMotorGearRatio * (Units.inchesToMeters(Constants.kWheelDiameterInches) * Math.PI);
    //rotations counted by motor -> rotations output side -> rads turned
    public final double ticksToRadsTurning = Constants.kTurningMotorGearRatio * 2 * Math.PI;

    private void initializeModule(int module) {

        steerMotorArray[module].setInverted(true);
        steerMotorArray[module].setNeutralMode(NeutralModeValue.Brake);
        steerMotorArray[module].setPosition(0);
        driveMotorArray[module].setNeutralMode(NeutralModeValue.Brake);
        driveMotorArray[module].setPosition(0);

        if (module == 3) {
        driveMotorArray[module].setInverted(true);
        } else {
            driveMotorArray[module].setInverted(false);
        }

        pidArray[module].reset();
        pidArray[module].enableContinuousInput(-Math.PI, Math.PI);

        CANcoderConfiguration defaultEncoderConfig = new CANcoderConfiguration();
        defaultEncoderConfig.MagnetSensor.MagnetOffset = absEncoderOffsetArray[module];
        defaultEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        encoderConfigArray[module].apply(defaultEncoderConfig);

    }

    public void initialize() {
        initializeModule(0);
        initializeModule(1);
        initializeModule(2);
        initializeModule(3);
        poseEstimator.resetPosition(navx.getRotation2d(), modulePositionArray, getPose());
        navx.reset();
    }

    public double readAbsEncoderRad(int module) {
        absEncoderArray[module].getPosition().refresh();
        return absEncoderArray[module].getAbsolutePosition().getValueAsDouble() * 2 * Math.PI;
    }

    /**
     * Gets encoder position of given encoder
     * @param encoder location id (0 is front left, 3 is back right)
     * @return value of requested encoder
     */
    public double readTurnEncoder(int encoder) {
        return steerMotorArray[encoder].getPosition().getValueAsDouble();        
    }

    /**
     * Gets encoder position of given encoder
     * @param encoder location id (0 is front left, 3 is back right)
     * @return value of requested encoder
     */
    public double readDriveEncoder(int encoder) {
        return driveMotorArray[encoder].getPosition().getValueAsDouble();
    }

    private SwerveModuleState[] swerveInputToModuleStates(double xSpeed, double ySpeed, double rotSpeed) {
        ChassisSpeeds desiredSpeeds = new ChassisSpeeds(xSpeed * Constants.maxSpeedMpS, ySpeed * Constants.maxSpeedMpS, rotSpeed);

        // make desiredSpeeds into speeds and angles for each module
        SwerveModuleState[] moduleStatesArray = m_kinematics.toSwerveModuleStates(desiredSpeeds);

        // normalize module values to remove impossible speed values
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStatesArray, Constants.maxSpeedMpS);

        return moduleStatesArray;
    }

    private SwerveModuleState swerveOptimizeModuleState(int id, SwerveModuleState moduleState) {
        double sensorPosition = readAbsEncoderRad(id); //readTurnEncoder(id) * ticksToRadsTurning;
        Rotation2d currentAngle = new Rotation2d(sensorPosition);    
        SwerveModuleState optimizedAngle = SwerveModuleState.optimize(moduleState, currentAngle);
        return optimizedAngle;
    }

    private void swerveModuleDrive(int module, SwerveModuleState moduleState) {
        SmartDashboard.putNumber("module" + module + " speed",moduleState.speedMetersPerSecond);
        SmartDashboard.putNumber("module" + module + " direction",moduleState.angle.getDegrees());

        SwerveModuleState optimizedState = swerveOptimizeModuleState(module, moduleState);
        SmartDashboard.putNumber("module" + module + " speedoptimised",optimizedState.speedMetersPerSecond);
        SmartDashboard.putNumber("module" + module + " directionoptimised",optimizedState.angle.getDegrees());


        double drivePower = optimizedState.speedMetersPerSecond / Constants.maxSpeedMpS;
        
        double turnPower = pidArray[module].calculate(
            readAbsEncoderRad(module), //steerMotorArray[module].getPosition().getValueAsDouble() * ticksToRadsTurning, 
            optimizedState.angle.getRadians()
        );
        SmartDashboard.putNumber("module" + module + " rawsensordata", steerMotorArray[module].getPosition().getValueAsDouble());
        SmartDashboard.putNumber("module" + module + " recordedturnposition", steerMotorArray[module].getPosition().getValueAsDouble() * ticksToRadsTurning);

        SmartDashboard.putNumber("module" + module + " drive power", drivePower);
        SmartDashboard.putNumber("module" + module + " turn power", turnPower);
            
        driveMotorArray[module].set(drivePower);
        steerMotorArray[module].set(turnPower);
        
    }

    public void swerveDrive(double xSpeed, double ySpeed, double rotSpeed) {
        SmartDashboard.putNumber("inputX", xSpeed);
        SmartDashboard.putNumber("inputY", ySpeed);
        SmartDashboard.putNumber("inputRot", rotSpeed);
        xSpeed = Constants.clamp(xSpeed, -1, 1);
        ySpeed = Constants.clamp(ySpeed, -1, 1);
        SwerveModuleState[] moduleStateArray = swerveInputToModuleStates(xSpeed, ySpeed, rotSpeed);

        swerveModuleDrive(0, moduleStateArray[0]);
        swerveModuleDrive(1, moduleStateArray[1]);
        swerveModuleDrive(2, moduleStateArray[2]);
        swerveModuleDrive(3, moduleStateArray[3]);
        
    }

    
    public final static double[] fieldOrient(double XSpeed, double YSpeed, AHRS navx) {
        double currentYawRadians = Math.toRadians(-navx.getYaw());
        double XSpeedField = XSpeed * Math.cos(currentYawRadians) - YSpeed * Math.sin(currentYawRadians);
        double YSpeedField = XSpeed * Math.sin(currentYawRadians) + YSpeed * Math.cos(currentYawRadians);
        double[] speeds = {XSpeedField, YSpeedField};
        return speeds;
    }

    SwerveModulePosition getModulePosition(int module) {
        return new SwerveModulePosition(
            readDriveEncoder(module) * Constants.driveRotsToMeter, 
            new Rotation2d(readAbsEncoderRad(module))
        );
    }

    SwerveModulePosition[] getModulePositionArray() {
        SwerveModulePosition[] swerveModulePositionArray = {
            getModulePosition(0),
            getModulePosition(1),
            getModulePosition(2),
            getModulePosition(3)
        };
        return swerveModulePositionArray;
    }

    public Pose2d updatePose() {
        return poseEstimator.update(navx.getRotation2d(), getModulePositionArray());
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void panicReset() {
        navx.reset();
        navx.zeroYaw();
        navx.setAngleAdjustment(0);
    }

    // public void resetTurnEncoders() {
    //     frontLeftSteer.setPosition(0);
    //     frontRightSteer.setPosition(0);
    //     backLeftSteer.setPosition(0);
    //     backRightSteer.setPosition(0);
    // }

    // public void resetTurnPIDs() {
    //     pidFrontLeftTurn.reset();
    //     pidFrontRightTurn.reset();
    //     pidBackLeftTurn.reset();
    //     pidBackRightTurn.reset();
    // }

    // public void setMotorBreaks() {
    //     frontLeftDrive.setNeutralMode(NeutralModeValue.Brake);
    //     frontRightDrive.setNeutralMode(NeutralModeValue.Brake);
    //     backLeftDrive.setNeutralMode(NeutralModeValue.Brake);
    //     backRightDrive.setNeutralMode(NeutralModeValue.Brake);

    //     frontLeftSteer.setNeutralMode(NeutralModeValue.Brake);
    //     frontRightSteer.setNeutralMode(NeutralModeValue.Brake);
    //     backLeftSteer.setNeutralMode(NeutralModeValue.Brake);
    //     backRightSteer.setNeutralMode(NeutralModeValue.Brake);
    // }

    // public void invertMotors() {
    //     frontLeftSteer.setInverted(true);
    //     frontRightSteer.setInverted(true);
    //     backLeftSteer.setInverted(true);
    //     backRightSteer.setInverted(true);

    //     frontLeftDrive.setInverted(true);
    //     frontRightDrive.setInverted(true);
    //     backLeftDrive.setInverted(true);
    //     backRightDrive.setInverted(true);
    // }

    // public void continouousInput() {
    //     pidFrontLeftTurn.enableContinuousInput(-Math.PI, Math.PI);
    //     pidFrontRightTurn.enableContinuousInput(-Math.PI, Math.PI);
    //     pidBackLeftTurn.enableContinuousInput(-Math.PI, Math.PI);
    //     pidBackRightTurn.enableContinuousInput(-Math.PI, Math.PI);
    // }


    // public void swerveSetTurnPower(int module, SwerveModuleState moduleState) {
    //     double power = pidArray[module].calculate(
    //         steerMotorArray[module].getPosition().getValue() * ticksToRadsTurning, moduleState.angle.getRadians());

    //     steerMotorArray[module].set(power);
    // }

    // public void swerveSetDrivePower(int module, SwerveModuleState moduleState) {
    //     double power = moduleState.speedMetersPerSecond / Constants.maxSpeedMpS;

    //     driveMotorArray[module].set(power);
    // }


    // //i hate this swerve drive mega function i want to break this down
    // public void swerveDrive(double xSpeed, double ySpeed, double rotSpeed) {
    //     ChassisSpeeds desiredSpeeds = new ChassisSpeeds(xSpeed * Constants.maxSpeedMpS, ySpeed * Constants.maxSpeedMpS, rotSpeed);

    //     // make desiredSpeeds into speeds and angles for each module
    //     SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(desiredSpeeds);

    //     // normalize module values to remove impossible speed values
    //     SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, maxSpeedMpS);

    //     SwerveModuleState frontLeftModule = moduleStates[0];
    //     SwerveModuleState frontRightModule = moduleStates[1];
    //     SwerveModuleState backLeftModule = moduleStates[2];
    //     SwerveModuleState backRightModule = moduleStates[3];

    //     // optimize wheel angles (ex. wheel is at 359deg and needs to go to 1deg. wheel
    //     // will now go 2deg instead of 358deg)

    //     double frontLeftSensorPos = frontLeftSteer.getSelectedSensorPosition() * kTurningEncoderTicksToRad;
    //     double frontRightSensorPos = frontRightSteer.getSelectedSensorPosition() * kTurningEncoderTicksToRad;
    //     double backLeftSensorPos = backLeftSteer.getSelectedSensorPosition() * kTurningEncoderTicksToRad;
    //     double backRightSensorPos = backRightSteer.getSelectedSensorPosition() * kTurningEncoderTicksToRad;

    //     var frontLeftCurrentAngle = new Rotation2d(frontLeftSensorPos);
    //     var frontRightCurrentAngle = new Rotation2d(frontRightSensorPos);
    //     var backLeftCurrentAngle = new Rotation2d(backLeftSensorPos);
    //     var backRightCurrentAngle = new Rotation2d(backRightSensorPos);

    //     var frontLeftOptimized = SwerveModuleState.optimize(frontLeftModule, frontLeftCurrentAngle);
    //     var frontRightOptimized = SwerveModuleState.optimize(frontRightModule, frontRightCurrentAngle);
    //     var backLeftOptimized = SwerveModuleState.optimize(backLeftModule, backLeftCurrentAngle);
    //     var backRightOptimized = SwerveModuleState.optimize(backRightModule, backRightCurrentAngle);

    //     // set steer motor power to the pid output of current position in radians and
    //     // desired position in radians
    //     double frontLeftTurnPower = pidFrontLeftTurn.calculate(
    //             frontLeftSteer.getSelectedSensorPosition() * kTurningEncoderTicksToRad,
    //             frontLeftOptimized.angle.getRadians());
    //     double frontRightTurnPower = pidFrontRightTurn.calculate(
    //             frontRightSteer.getSelectedSensorPosition() * kTurningEncoderTicksToRad,
    //             frontRightOptimized.angle.getRadians());
    //     double backLeftTurnPower = pidBackLeftTurn.calculate(
    //             backLeftSteer.getSelectedSensorPosition() * kTurningEncoderTicksToRad,
    //             backLeftOptimized.angle.getRadians());
    //     double backRightTurnPower = pidBackRightTurn.calculate(
    //             backRightSteer.getSelectedSensorPosition() * kTurningEncoderTicksToRad,
    //             backRightOptimized.angle.getRadians());

    //     // positive is clockwise (right side up)
    //     frontLeftSteer.set(frontLeftTurnPower);
    //     frontRightSteer.set(frontRightTurnPower);
    //     backLeftSteer.set(backLeftTurnPower);
    //     backRightSteer.set(backRightTurnPower);

    //     // set drive power to desired speed div max speed to get value between 0 and 1
    //     frontLeftDrive.set(frontLeftOptimized.speedMetersPerSecond / maxSpeedMpS);
    //     frontRightDrive.set(frontRightOptimized.speedMetersPerSecond / maxSpeedMpS);
    //     backLeftDrive.set(backLeftOptimized.speedMetersPerSecond / maxSpeedMpS);
    //     backRightDrive.set(backRightOptimized.speedMetersPerSecond / maxSpeedMpS);
    // }




}
