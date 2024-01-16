package frc.robot;

public class Driver {
    
    
    PIDController pidFrontLeftTurn = new PIDController(Constants.kpDrive, Constants.kiDrive, Constants.kdDrive);
    PIDController pidFrontRightTurn = new PIDController(Constants.kpDrive, Constants.kiDrive, Constants.kdDrive);
    PIDController pidBackLeftTurn = new PIDController(Constants.kpDrive, Constants.kiDrive, Constants.kdDrive);
    PIDController pidBackRightTurn = new PIDController(Constants.kpDrive, Constants.kiDrive, Constants.kdDrive);

    public final WPI_TalonFX frontLeftDrive = new WPI_TalonFX(7);
    public final WPI_TalonFX frontRightDrive = new WPI_TalonFX(1);
    public final WPI_TalonFX backLeftDrive = new WPI_TalonFX(5);
    public final WPI_TalonFX backRightDrive = new WPI_TalonFX(3);

    public final WPI_TalonFX frontLeftSteer = new WPI_TalonFX(6);
    public final WPI_TalonFX frontRightSteer = new WPI_TalonFX(0);
    public final WPI_TalonFX backLeftSteer = new WPI_TalonFX(4);
    public final WPI_TalonFX backRightSteer = new WPI_TalonFX(2);

    public final WPI_CANCoder frontLeftAbsEncoder = new WPI_CANCoder(3);
    public final WPI_CANCoder frontRightAbsEncoder = new WPI_CANCoder(0);
    public final WPI_CANCoder backLeftAbsEncoder = new WPI_CANCoder(2);
    public final WPI_CANCoder backRightAbsEncoder = new WPI_CANCoder(1);

    public final double ticksToMetersDrive = (frontLeftDrive.getSelectedSensorPosition / Constants.talonEncoder_TicksPerRev) * Constants.kDriveMotorGearRatio * Units.inchesToMeters(Constants.kWheelDiameterInches) * Math.PI
    public final double ticksToRadsTurning =  



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




}
