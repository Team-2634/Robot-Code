package frc.robot.systems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.kauailabs.navx.frc.AHRS;

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
    AHRS navx;
    public Driver(AHRS navx) {
        this.navx = navx;
    }

    final PIDController pidFrontLeftTurn = new PIDController(Constants.kpDrive, Constants.kiDrive, Constants.kdDrive);
    final PIDController pidFrontRightTurn = new PIDController(Constants.kpDrive, Constants.kiDrive, Constants.kdDrive);
    final PIDController pidBackLeftTurn = new PIDController(Constants.kpDrive, Constants.kiDrive, Constants.kdDrive);
    final PIDController pidBackRightTurn = new PIDController(Constants.kpDrive, Constants.kiDrive, Constants.kdDrive);
    final PIDController[] pidArray = {pidFrontLeftTurn, pidFrontRightTurn, pidBackLeftTurn, pidBackRightTurn};

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

    final Translation2d frontLeftWheelLocation = new Translation2d(0.325, 0.325);
    final Translation2d frontRightWheelLocation = new Translation2d(0.325, -0.325);
    final Translation2d backLeftWheelLocation = new Translation2d(-0.325, 0.325);
    final Translation2d backRightWheelLocation = new Translation2d(-0.325, -0.325);

    final SwerveModulePosition frontLeftModulePosition = new SwerveModulePosition();
    final SwerveModulePosition frontRightModulePosition = new SwerveModulePosition();
    final SwerveModulePosition backLeftModulePosition = new SwerveModulePosition();
    final SwerveModulePosition backRightModulePosition = new SwerveModulePosition();
    final SwerveModulePosition[] modulePositionArray = {
        frontLeftModulePosition, 
        frontRightModulePosition, 
        backLeftModulePosition, 
        backRightModulePosition
    };

    public final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        frontLeftWheelLocation, frontRightWheelLocation, backLeftWheelLocation, backRightWheelLocation);
        
    SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
        kinematics, 
        navx.getRotation2d(), 
        modulePositionArray,
        new Pose2d()
    );

    //rotations counted by motor -> rotations wheel side -> distance travelled (meters) 
    public final double ticksToMetersDrive = Constants.kDriveMotorGearRatio * (Units.inchesToMeters(Constants.kWheelDiameterInches) * Math.PI);
    //rotations counted by motor -> rotations output side -> rads turned
    public final double ticksToRadsTurning = Constants.kTurningMotorGearRatio * 2 * Math.PI;

    private void initializeModule(int module) {
        driveMotorArray[module].setNeutralMode(NeutralModeValue.Brake);
        driveMotorArray[module].setInverted(true);

        steerMotorArray[module].setNeutralMode(NeutralModeValue.Brake);
        steerMotorArray[module].setInverted(true);
        steerMotorArray[module].setPosition(0);

        pidArray[module].reset();
        pidArray[module].enableContinuousInput(-Math.PI, Math.PI);
    }

    public void initialize() {
        initializeModule(0);
        initializeModule(1);
        initializeModule(2);
        initializeModule(3);
    }

    /**
     * Gets encoder position of given encoder
     * @param encoder location id (0 is front left, 3 is back right)
     * @return value of requested encoder
     */
    public double readTurnEncoder(int encoder) {
        return steerMotorArray[encoder].getPosition().getValue();
    }

    /**
     * Gets encoder position of given encoder
     * @param encoder location id (0 is front left, 3 is back right)
     * @return value of requested encoder
     */
    public double readDriveEncoder(int encoder) {
        return driveMotorArray[encoder].getPosition().getValue();
    }

    private SwerveModuleState[] swerveInputToModuleStates(double xSpeed, double ySpeed, double turnSpeed) {
        ChassisSpeeds desiredSpeeds = new ChassisSpeeds(
            xSpeed * Constants.maxSpeedMpS, 
            ySpeed * Constants.maxSpeedMpS, 
            turnSpeed * Constants.maxSpeedRotation
        );

        // make desiredSpeeds into speeds and angles for each module
        SwerveModuleState[] moduleStatesArray = kinematics.toSwerveModuleStates(desiredSpeeds);

        // normalize module values to remove impossible speed values
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStatesArray, Constants.maxSpeedMpS);

        return moduleStatesArray;
    }
    
    /**
     * Optimize module state so that module does not have to turn more than 90 degrees.
     * @param id
     * @param moduleState
     * @return
     */
    private SwerveModuleState swerveOptimizeModuleState(int id, SwerveModuleState moduleState) {
        double sensorPosition = readTurnEncoder(id) * ticksToRadsTurning;
        Rotation2d currentAngle = new Rotation2d(sensorPosition);
        SwerveModuleState optimizedAngle = SwerveModuleState.optimize(moduleState, currentAngle);
        return optimizedAngle;
    }

    private void swerveModuleDrive(int module, SwerveModuleState moduleState) {

        SwerveModuleState optimizedState = swerveOptimizeModuleState(module, moduleState);

        double drivePower = optimizedState.speedMetersPerSecond / Constants.maxSpeedMpS;
        
        double turnPower = pidArray[module].calculate(
            steerMotorArray[module].getPosition().getValue() * ticksToRadsTurning, 
            optimizedState.angle.getRadians()
        );

        // SmartDashboard.putNumber("module" + module + " speed",moduleState.speedMetersPerSecond);
        // SmartDashboard.putNumber("module" + module + " direction",moduleState.angle.getDegrees());

        // SmartDashboard.putNumber("module" + module + " speedoptimised",optimizedState.speedMetersPerSecond);
        // SmartDashboard.putNumber("module" + module + " directionoptimised",optimizedState.angle.getDegrees());

        // SmartDashboard.putNumber("module" + module + " rawsensordata", steerMotorArray[module].getPosition().getValue());
        // SmartDashboard.putNumber("module" + module + " recordedturnposition", steerMotorArray[module].getPosition().getValue() * ticksToRadsTurning);

        // SmartDashboard.putNumber("module" + module + " drive power", drivePower);
        // SmartDashboard.putNumber("module" + module + " turn power", turnPower);
            
        driveMotorArray[module].set(drivePower);
        steerMotorArray[module].set(turnPower);
    }

    public void swerveDrive(double xSpeed, double ySpeed, double rotSpeed) {
        // SmartDashboard.putNumber("inputX", xSpeed);
        // SmartDashboard.putNumber("inputY", ySpeed);
        // SmartDashboard.putNumber("inputRot", rotSpeed);
        xSpeed = Constants.clamp(xSpeed, -1, 1);
        ySpeed = Constants.clamp(ySpeed, -1, 1);
        SwerveModuleState[] moduleStateArray = swerveInputToModuleStates(xSpeed, ySpeed, rotSpeed);

        swerveModuleDrive(0, moduleStateArray[0]);
        swerveModuleDrive(1, moduleStateArray[1]);
        swerveModuleDrive(2, moduleStateArray[2]);
        swerveModuleDrive(3, moduleStateArray[3]);
        
    }

    
    public final double[] fieldOrient(double XSpeed, double YSpeed) {
        double currentYawRadians = Math.toRadians(navx.getYaw());
        double XSpeedField = XSpeed * Math.cos(currentYawRadians) - YSpeed * Math.sin(currentYawRadians);
        double YSpeedField = XSpeed * Math.sin(currentYawRadians) + YSpeed * Math.cos(currentYawRadians);
        double[] speeds = {XSpeedField, YSpeedField};
        return speeds;
    }

    SwerveModulePosition getModulePosition(int module) {
        return new SwerveModulePosition(
            readDriveEncoder(module) * ticksToMetersDrive, 
            new Rotation2d(readTurnEncoder(module) * ticksToRadsTurning)
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
        return poseEstimator.update(new Rotation2d(navx.getAngle()), getModulePositionArray());
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }
}