package rizzler;

import java.util.HashMap;
import globalFunctions;
import globalFunctions.toDashboard;
import globalFunctions.getEncoderTicksMPS;
import globalFunctions.getEncoderTicksMPS_steer;
import globalFunctions.removeDeadzone;

public class newMain {
    SwerveDriveTrain driveTrain;

    final XboxController driving_xBoxCont = new XboxController(0);
    final XboxController arm_xBoxCont = new XboxController(1);

    //TODO what is this?
    public static final HashMap<MotorLocation, Translation2d> motorLocations = initMotorLocations();
    //This lets you initialze a static hashmap at runtime
    private static HashMap<MotorLocation, Translation2d> initMotorLocations() { 
        HashMap<MotorLocation, Translation2d> motorLocationsInit = new HashMap<>();
        motorLocationsInit.put(MotorLocation.FrontLeft_Drive, new Translation2d(0.340, 0.285));
        motorLocationsInit.put(MotorLocation.FrontRight_Drive, new Translation2d(0.340, -0.285));
        motorLocationsInit.put(MotorLocation.BackRight_Drive, new Translation2d(-0.340, 0.285));
        motorLocationsInit.put(MotorLocation.BackRight_Drive, new Translation2d(-0.340, -0.285));
        return Collections.unmodifiableMap(motorLocationsInit);
    }

    /*/
    public static final HashMap<MotorType, <T>> motorTypes = initMotorTypes();
    private static HashMap<MotorLocation, ArrayList<double>> initMotorTypes() { 
        HashMap<MotorType, ArrayList<double>> motorTypesInit = new HashMap<MotorType, ArrayList<double>>();
        motorTypesInit.put(MotorType.WPI_TalonFX, new ArrayList<double>(0.340, 0.285));
        motorTypesInit.put(MotorType.CANSparkMax, new ArrayList<double>(0.340, 0.285));
        return Collections.unmodifiableMap(motorTypesInit);
    }
    */
    //TODO make use of, remove, fix value...

    public newMain(){
        //Tuple offset = new Tuple(0.340, 0.285);


        Motors[] motors= {
            //TODO does this work? 
            new Motor<MotorType.WPI_TalonFX>(7, MotorType.WPI_TalonFX, MotorLocation.FrontLeft_Drive),
            new Motor<MotorType.WPI_TalonFX>(1, MotorType.WPI_TalonFX, MotorLocation.FrontRight_Drive),
            new Motor<MotorType.WPI_TalonFX>(5, MotorType.WPI_TalonFX, MotorLocation.BackRight_Drive),
            new Motor<MotorType.WPI_TalonFX>(3, MotorType.WPI_TalonFX, MotorLocation.BackRight_Drive),

            new Motor<MotorType.WPI_TalonFX>(6, MotorType.WPI_TalonFX, MotorLocation.FrontLeft_Steer),
            new Motor<MotorType.WPI_TalonFX>(0, MotorType.WPI_TalonFX, MotorLocation.FrontRight_Steer),
            new Motor<MotorType.WPI_TalonFX>(4, MotorType.WPI_TalonFX, MotorLocation.BackRight_Steer),
            new Motor<MotorType.WPI_TalonFX>(2, MotorType.WPI_TalonFX, MotorLocation.BackRight_Steer)
        }

        /*
            public final WPI_CANCoder frontLeftAbsEncoder = new WPI_CANCoder(3);
            public final WPI_CANCoder frontRightAbsEncoder = new WPI_CANCoder(0);
            public final WPI_CANCoder backLeftAbsEncoder = new WPI_CANCoder(2);
            public final WPI_CANCoder backRightAbsEncoder = new WPI_CANCoder(1);

            public final double frontLeftAbsOffset = 197.19;
            public final double frontRightAbsOffset = 31.8;
            public final double backLeftAbsOffset = 285.205;
            public final double backRightAbsOffset = 50.4;
         */

        //TODO make PID vals variable by gamemode
        this.driveTrain = new SwerveDriveTrain(motors, Config.teleopSteerP, Config.teleopSteerI, Config.teleopSteerD);
        //this.driveTrain.setInverted(true);
    }


    //TODO move to main
    // execution Functions vvvvv
    @Override
    public void robotInit() {
        timerRobot.reset();
        timerRobot.start();

        UsbCamera camera0 = CameraServer.startAutomaticCapture(0); //these shouldnt be defined here?
        UsbCamera camera1 = CameraServer.startAutomaticCapture(1);

        driveTrain.setMotorBreaks();
        //arm.setMotorBreaks();

        driveTrain.setInverted(true);
        driveTrain.setContinouousInput();

        navx.calibrate();
        navx.reset();

        driveTrain.resetEncoders();
        //arm.resetEncoders();

        driveTrain.resetPIDs();
        driveTrain.setDriveSensorPosition(0);
    }

    @Override
    public void robotPeriodic() {
        // encoder drive variables vvv
        double frontLeftDriveDisplacement_Meters = driveTrain.getDriveSensorPosition(MotorLocation.frontLeftDrive) * getEncoderTicksMPS();
        toDashboard("Drive_Distance_Meters x axis: ", frontLeftDriveDisplacement_Meters);

        double frontRightDriveDisplacement_Meters = driveTrain.getSelectedSensorPosition(MotorLocation.frontRightDrive) * getEncoderTicksMPS();
        toDashboard("Drive_Distance_Meters yaxis: ", frontRightDriveDisplacement_Meters);

        double frontLeftSteer_Rad = driveTrain.getDriveSensorPosition(MotorLocation.frontLeftSteer) * getEncoderTicksMPS_steer();
        toDashboard("Drive_Rotation_Radians: ", frontLeftSteer_Rad);

        // navX2 vvv
        //TODO, clean up and un-duplicate
        toDashboard("Yaw_Deg", navx.getYaw());
        toDashboard("navxYaw_Rad ", Units.degreesToRadians(navx.getYaw()));
        toDashboard("angleRad: ", Math.toRadians(navx.getAngle()));
        toDashboard("Pitch_deg", navx.getPitch());
        toDashboard("Roll_Deg", navx.getRoll());
        toDashboard(navx);

        // encoders vvv
        //TODO
        toDashboard("frontLeftAbs Offset", frontLeftAbsEncoder.getAbsolutePosition());
        toDashboard("frontRightAbs Offset", frontRightAbsEncoder.getAbsolutePosition());
        toDashboard("backLeftAbs Offset", backLeftAbsEncoder.getAbsolutePosition());
        toDashboard("backRightAbs Offset", backRightAbsEncoder.getAbsolutePosition());

        // pid charts vvv
        // auto vvv
        //toDashboard("drive PID (atm used for x, y and turn of autonomous)", drive.getPositionError());
        //toDashboard("PID_armAngle", PID_armAngle.getPositionError());

        // aline vvv
        //TODO
        toDashboard("pid_frontLeft_Error", pidFrontLeftTurn.getPositionError());
        toDashboard("pid_frontRight_Error", pidFrontRightTurn.getPositionError());
        toDashboard("pid_backLeft_Error", pidBackLeftTurn.getPositionError());
        toDashboard("pid_backRight_Error", pidBackRightTurn.getPositionError());
    }

    @Override
    public void autonomousInit() {
        timerAuto.reset();
        timerAuto.start();

        //dSolenoidClaw.set(Value.kReverse);
        //Collections.fill(commandFlags, Boolean.FALSE);
    }

    @Override
    public void autonomousPeriodic() {
        // autoTopCone(); //balance
        autoTopAndBottom(); //not balance
    } 

    @Override
    public void teleopPeriodic() {

        //limitationArmRise(armRad_current);
        //toDashboard("armAngleLimited: ", armAngleLimited);

        //limitationArmExtend(extenstionEncoder_CurrentMetres);
        // toDashboard("armExtendLimited: ", armExtendLimited);
        // setMotorSpeed_limitinExtendWHenToFar(-armTalonExtenstionSpeed_autoRetreat);

        //toDashboard("retractLimitSwitch: ", extendLimitSwitch.get());
        //System.out.println(extendLimitSwitch.get());


        
        //// Swerve vvv
        //Get scaled Axis inputs from controller
        double contXSpeed = removeDeadzone(this.driving_xBoxCont, 1) * Config.XdriveSensitivity;
        double contYSpeed = removeDeadzone(this.driving_xBoxCont, 0) * Config.YdriveSensitivity;
        double contTurnSpeed = removeDeadzone(this.driving_xBoxCont, 4) * Config.turningSensitivity;
 
        if (Config.useFieldOrientation) {
            double botYawAngle_Rad = Math.toRadians(navx.getAngle());
            contXSpeed = contXSpeed * Math.cos(botYawAngle_Rad) - contYSpeed * Math.sin(botYawAngle_Rad);
            contYSpeed = contXSpeed * Math.sin(botYawAngle_Rad) + contYSpeed * Math.cos(botYawAngle_Rad);
        }
        this.SwerveDriveTrain.GO(contXSpeed, contYSpeed, contTurnSpeed);

        // Button inputs: //TODO cleanup, set up blanks, controlSchemes, use better namings
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
        robotArm(armDown, armUp, claw_xBox, extendArm, retractArm, expel, intake, clawIntake_and_Extend, slowClawWheels);
    }
}
